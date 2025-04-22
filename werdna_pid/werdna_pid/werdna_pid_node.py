import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist
from werdna_msgs.msg import JoyCtrlCmds

from scipy.spatial.transform import Rotation as R

import os
from datetime import datetime
import pandas as pd
import yaml

import math

class PIDConfig:
    def __init__(self):
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.iratelimit = -1.0
        self.ilimit = 0.0
        self.kpkd_limit = -1.0
        self.max_desired_rate = 0.0
        self.sign = 1.0


class PIDState:
    def __init__(self):
        self.integral = 0.
        # // When starting with desired rate limits in place, we by default
        # // always accept the first desired command with no limiting (users
        # // can of course override this value if they want to start from
        # // some predetermined value).
        self.desired = float('nan')
        self.error = 0.0
        self.error_rate = 0.0
        self.p = 0.0
        self.d = 0.0
        self.pd = 0.0
        self.command = 0.0

    def clear(self):
        self.integral = 0.0
        self.desired = float('nan')
        self.error = 0.0
        self.error_rate = 0.0
        self.p = 0.0
        self.d = 0.0
        self.pd = 0.0
        self.command = 0.0


class PID:
    def __init__(self, config: PIDConfig, state: PIDState):
        self.config = config
        self.state = state

    def apply(self, measured, input_desired, measured_rate, input_desired_rate, rate_hz, kp_scale=1.0, kd_scale=1.0):
        cfg = self.config
        st = self.state

        # Desired rate limiting
        if cfg.max_desired_rate != 0.0 and not math.isnan(st.desired):
            max_step = cfg.max_desired_rate / rate_hz
            proposed_step = input_desired - st.desired
            actual_step = max(min(proposed_step, max_step), -max_step)
            desired = st.desired + actual_step
            desired_rate = max(min(input_desired_rate, cfg.max_desired_rate), -cfg.max_desired_rate)
        else:
            desired = input_desired
            desired_rate = input_desired_rate

        st.desired = desired
        st.error = measured - desired
        st.error_rate = measured_rate - desired_rate

        # Integral update
        to_update_i = st.error * cfg.ki / rate_hz
        max_i_update = cfg.iratelimit / rate_hz if cfg.iratelimit > 0.0 else None
        if max_i_update is not None:
            to_update_i = max(min(to_update_i, max_i_update), -max_i_update)

        st.integral += to_update_i
        if cfg.ilimit > 0.0:
            st.integral = max(min(st.integral, cfg.ilimit), -cfg.ilimit)

        # Compute PD
        st.p = kp_scale * cfg.kp * st.error
        st.d = kd_scale * cfg.kd * st.error_rate
        st.pd = st.p + st.d

        if cfg.kpkd_limit >= 0.0:
            st.pd = max(min(st.pd, cfg.kpkd_limit), -cfg.kpkd_limit)

        # Final command
        st.command = cfg.sign * (st.pd + st.integral)
        return st.command


class ControlNode(Node):

    def __init__(self):
        super().__init__("control_node")
        
        self.get_logger().info("Initializing Werdna Control Node...")

        # Subscribers
        self.get_logger().info("Setting up subscribers...")
        self.imu_subscriber = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.joint_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.command_subscriber = self.create_subscription(JoyCtrlCmds, '/werdna_control', self.command_callback, 10)
        self.nav2_subscriber = self.create_subscription(Twist, "cmd_vel", self.nav2_callback, 10)
        self.get_logger().info("Subscribers initialized")

        # Publishers
        self.get_logger().info("Setting up publishers...")
        self.wheel_controller = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 10)
        self.legs_controller = self.create_publisher(Float64MultiArray, "/position_controller/commands", 10)
        self.get_logger().info("Publishers initialized")

        self.target_joints = ["left_hip_motor_joint", "right_hip_motor_joint", "left_knee_joint", "right_knee_joint", "left_wheel_joint", "right_wheel_joint"]
        self.get_logger().info(f"Target joints configured: {self.target_joints}")

        # Robot state variables
        self.current_state = False
        self.left_height = 0
        self.right_height = 0
        self.left_wheel = 0
        self.right_wheel = 0
        #joystick
        self.linear_x_joy = 0
        self.angular_z_joy = 0
        #nav2
        self.linear_x_nav2 = 0
        self.angular_z_nav2 = 0

        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.linear_accelerations = np.array([0.0, 0.0, 0.0])
        self.joint_positions = {joint: 0.0 for joint in self.target_joints}
        self.joint_velocities = {joint: 0.0 for joint in self.target_joints}

        self.wheel_radius = 0.0855
        self.wheel_circumference = np.pi * self.wheel_radius
        
        # Safety parameters
        self.pitch_threshold = 0.7  
        self.filtered_angular_velocity = np.zeros(3)
        self.safety_triggered = False
        self.yaw_offset = None
        
        self.get_logger().info("Safety parameters configured:")
        self.get_logger().info(f"  - Pitch threshold: {self.pitch_threshold:.2f} radians ({np.degrees(self.pitch_threshold):.1f} degrees)")
        
        # Internal PID state
        self.pitch = 0.0
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch_vel = 0.0
        self.yaw_vel = 0.0

        # Control loop rate (Hz)
        self.control_rate = 100  # 100Hz or 0.01s period
        
        # Setup PID controllers
        self.velocity_pid_config = PIDConfig()
        self.velocity_pid_state = PIDState()
        self.velocity_pid = PID(self.velocity_pid_config, self.velocity_pid_state)
        
        self.balance_pid_config = PIDConfig()
        self.balance_pid_state = PIDState()
        self.balance_pid = PID(self.balance_pid_config, self.balance_pid_state)
        
        self.steer_pid_config = PIDConfig()
        self.steer_pid_state = PIDState()
        self.steer_pid = PID(self.steer_pid_config, self.steer_pid_state)
        
        # Load PID configurations
        config_file = "/home/andrew/werdna_ws/src/werdna_ros2/pid.yaml"
        self.load_config(config_file)

        timer_period = 0.01  # 100Hz
        self.runtime = self.create_timer(timer_period, self.runtime_callback)
        
        self.get_logger().info("Werdna Control Node initialization complete!")

        # Add data logging for Excel
        self.data_log = []
        self.log_columns = [
            'timestamp', 
            'roll',
            'roll_vel'
            'pitch', 
            'pitch_vel',
            'yaw',
            'yaw_vel', 
            'left_wheel_torque', 
            'right_wheel_torque',
            'left_wheel_velocity', 
            'right_wheel_velocity',
            'avg_velocity',
            'desired_linear_x', 
            'desired_angular_z',
            'left_height',
            'right_height',
            'average_height'
        ]
        
        # Configure logging frequency (e.g., log every 5 control cycles)
        self.log_frequency = 5
        self.log_counter = 0
        
        # Prepare for Excel logging
        self.log_directory = os.path.expanduser("~/werdna_logs")
        if not os.path.exists(self.log_directory):
            os.makedirs(self.log_directory)
            self.get_logger().info(f"Created log directory: {self.log_directory}")
        
        # Auto-save every N data points to prevent data loss
        self.auto_save_threshold = 500
        
        # Add shutdown hook for saving data
        self.get_logger().info("Data logging initialized")
        
        # Add a timer to periodically save data (every 60 seconds)
        self.save_timer = self.create_timer(15.0, self.save_data_timer_callback)
    
    def load_config(self, config_file):
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            # Load velocity PID configuration
            self.velocity_pid_config.kp = config.get("velocity_kp", 0.0)
            self.velocity_pid_config.ki = config.get("velocity_ki", 0.0)
            # self.velocity_pid_config.kd = config.get("velocity_kd", 0.0)
            self.velocity_pid_config.ilimit = config.get("velocity_ilimit", 0.5)
            # self.velocity_pid_config.iratelimit = config.get("velocity_iratelimit", 0.1)
            # self.velocity_pid_config.kpkd_limit = config.get("velocity_kpkd_limit", 0.15)
            # self.velocity_pid_config.max_desired_rate = config.get("velocity_max_rate", 1.0)
            # self.velocity_pid_config.sign = 1.0
            
            # Load balance PID configuration
            self.balance_pid_config.kp = config.get("balance_kp", 0.0)
            self.balance_pid_config.ki = config.get("balance_ki", 0.0)
            self.balance_pid_config.kd = config.get("balance_kd", 0.0)
            self.balance_pid_config.ilimit = config.get("balance_ilimit", 0.0)
            # self.balance_pid_config.iratelimit = config.get("balance_iratelimit", -1.0)
            # self.balance_pid_config.kpkd_limit = config.get("balance_kpkd_limit", 1.0)
            # self.balance_pid_config.max_desired_rate = config.get("balance_max_rate", 0.0)
            # self.balance_pid_config.sign = 1.0
            
            # Load steering PID configuration
            self.steer_pid_config.kp = config.get("steer_kp", 0.0)
            # self.steer_pid_config.ki = config.get("steer_ki", 0.0)
            self.steer_pid_config.kd = config.get("steer_kd", 0.0)
            self.steer_pid_config.ilimit = config.get("steer_ilimit", 0.0)
            # self.steer_pid_config.iratelimit = config.get("steer_iratelimit", -1.0)
            # self.steer_pid_config.kpkd_limit = config.get("steer_kpkd_limit", 0.5)
            # self.steer_pid_config.max_desired_rate = config.get("steer_max_rate", 0.0)
            # self.steer_pid_config.sign = 1.0
            
            # Additional parameters
            self.pitch_offset = config.get("offset", 0.0)
            self.accel_gain = config.get("accel_gain", 0.0)
            self.logdata = config.get("SaveData", False)
            
            self.get_logger().info(f"PID configuration loaded from {config_file}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load PID configuration: {str(e)}")
            # Set default values
            # self.balance_pid_config.kp = 0.5
            # self.balance_pid_config.kd = 0.1
            # self.steer_pid_config.kp = 0.2
            # self.steer_pid_config.kd = 0.05
            # self.velocity_pid_config.kp = 0.3
            # self.velocity_pid_config.ki = 0.05
            # self.pitch_offset = 0.0
    
    def runtime_callback(self):
        if not self.current_state:
            self.step(np.array([0.0, 0.0]))
            return

        if self.safety_triggered:
            hip, knee = self.inverse_kinematics(0.0, 0.01)
            leg_cmd = Float64MultiArray()
            leg_cmd.data = [float(hip), float(hip)]
            self.legs_controller.publish(leg_cmd)

            wheel_cmd = Float64MultiArray()
            wheel_cmd.data = [0.0, 0.0]
            self.wheel_controller.publish(wheel_cmd)
            return
        
        desired_linear_x = 0
        desired_angular_z = 0
        
        if (abs(self.linear_x_joy) <= 0.005 and abs(self.angular_z_joy) <= 0.05):
            desired_linear_x = self.linear_x_joy
            desired_angular_z = self.angular_z_joy
        else:
            desired_linear_x = self.linear_x_nav2
            desired_angular_z = self.angular_z_nav2

        # === OUTER VELOCITY LOOP ===
        wheel_radius = 0.0855  # Adjust as needed for your robot
        left_vel = self.joint_velocities["left_wheel_joint"] * wheel_radius
        right_vel = self.joint_velocities["right_wheel_joint"] * wheel_radius
        avg_velocity = (0.5 * (left_vel + right_vel)) 

        # Apply velocity PID controller

        target_pitch = self.velocity_pid.apply(
            avg_velocity,  
            desired_linear_x, 
            0.0,
            0.0, 
            self.control_rate  
        ) 
        
        # Clamp the target pitch to reasonable values
        target_pitch = np.clip(target_pitch, -0.08, 0.08)

 
        balance_output = self.balance_pid.apply(
            self.pitch, 
            target_pitch + self.pitch_offset,  
            self.pitch_vel,  
            0.0,  
            self.control_rate 
        )

        # === YAW STEERING CONTROLLER ===
        if self.yaw_offset is not None:
            # Apply steering PID controller
            steer_output = self.steer_pid.apply(
                self.yaw_vel, 
                desired_angular_z,
                0.0,  
                0.0,  
                self.control_rate  
            )
        else:
            steer_output = 0.0

        # === TORQUE COMMANDS ===
        left_wheel_torque = balance_output - steer_output
        right_wheel_torque = balance_output + steer_output
        action = np.array([left_wheel_torque, right_wheel_torque])

        total_height = 0.5 * (self.left_height + self.right_height)

        if self.logdata:
            # Log data for Excel export
            self.log_counter += 1
            if self.log_counter >= self.log_frequency:
                self.log_counter = 0
                self.log_data_point(
                    self.roll,
                    self.roll_vel,
                    self.pitch,
                    self.pitch_vel,
                    self.yaw,
                    self.yaw_vel,
                    left_wheel_torque,
                    right_wheel_torque,
                    left_vel,
                    right_vel,
                    avg_velocity,
                    desired_linear_x,
                    desired_angular_z,
                    self.left_height,
                    self.right_height,
                    total_height
                )

        self.step(action)

        # === DEBUG INFO ===
        # Only log at 5Hz to avoid flooding the console
        # if int(time.time() * 5) % 5 == 0:  # Log at approximately 1Hz
        #     self.get_logger().info(
        #         "\n========== PID CONTROL ==========\n"
        #         f"Target Pitch        : {target_pitch:.3f} rad\n"
        #         f"Pitch               : {self.pitch:.3f} rad\n"
        #         f"Pitch Error         : {self.balance_pid_state.error:.3f} rad\n"
        #         f"Pitch P Term        : {self.balance_pid_state.p:.3f}\n"
        #         f"Pitch D Term        : {self.balance_pid_state.d:.3f}\n"
        #         f"Pitch Rate          : {self.pitch_vel:.3f} rad/s\n"
        #         f"Yaw Velocity        : {self.yaw_vel:.3f} rad/s\n"
        #         f"Desired Lin Vel X   : {self.desired_linear_x:.2f}\n"
        #         f"Estimated Avg Vel   : {avg_velocity:.2f}\n"
        #         f"Velocity Error      : {self.velocity_pid_state.error:.2f}\n"
        #         f"Velocity Integral   : {self.velocity_pid_state.integral:.2f}\n"
        #         f"Wheel Commands      : [{action[0]:.2f}, {action[1]:.2f}]\n"
        #         "=================================="
        #     )


    def step(self, action):
        
        # velocity_des = np.zeros(2)
        exec_actions = np.clip(action, -1.0, 1.0)

        left_hip, left_knee = self.inverse_kinematics(0, self.left_height)
        right_hip, right_knee = self.inverse_kinematics(0, self.right_height)

        if (left_hip >= 0.32):
            left_hip = 0.32
        if (right_hip >= 0.32):
            right_hip = 0.32

        wheel_cmd = Float64MultiArray()
        leg_cmd = Float64MultiArray()

        # First two actions control wheels
        wheel_cmd.data = [float(exec_actions[0] * 1.0), float(exec_actions[1] * 1.0)]
        
        # Remaining actions control the leg joints
        leg_cmd.data = [float(left_hip), float(right_hip)]
        
        self.wheel_controller.publish(wheel_cmd)
        self.legs_controller.publish(leg_cmd)
    
    def inverse_kinematics(self, x=0, y=0):
        try:
            L1 = 0.148
            L2 = 0.148
            r = np.sqrt(x**2 + y**2)

            # Check reachability
            if r > (L1 + L2) or r < abs(L1 - L2) or y < 0:
                raise ValueError("Target position is out of reach.")
            
            # Handle special case when y == 0
            if y == 0:
                if x >= 0:
                    # Fully stretched along the x-axis
                    hip_theta = 0.01
                    knee_theta = 0.01
                else:
                    # Fully stretched in the opposite direction along the x-axis
                    hip_theta = 0.01
                    knee_theta = 0.01
                return hip_theta, knee_theta

            # Compute joint angles
            cos_knee = (L1**2 + L2**2 - r**2) / (2 * L1 * L2)
            knee_theta = np.arccos(np.clip(cos_knee, -1, 1))

            cos_hip = (L1**2 + r**2 - L2**2) / (2 * L1 * r)
            hip_theta = np.arctan2(y, x) - np.arccos(np.clip(cos_hip, -1, 1))

            return hip_theta, knee_theta
        except Exception as e:
            self.get_logger().error(f"Error in inverse_kinematics with x={x}, y={y}: {e}")
            return 0, 0
    
    def imu_callback(self, msg):
        base_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        raw_angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        # Correct the typo: use 'linear_accelerations'
        self.linear_accelerations = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Apply low-pass filter
        self.filtered_angular_velocity = raw_angular_velocity

        self.angular_velocity = self.filtered_angular_velocity.copy()

        # Extract roll and pitch from quaternion for safety checks
        euler_angles = R.from_quat(base_quat).as_euler('xyz')
        self.roll = euler_angles[0] 
        self.pitch = euler_angles[1] 
        self.yaw = euler_angles[2] 

        if self.yaw_offset is None:
            self.yaw_offset = self.yaw
            self.get_logger().info(f"Yaw offset initialized to {np.degrees(self.yaw_offset):.2f}°")

        # Safety check
        if abs(self.pitch) > self.pitch_threshold and not self.safety_triggered:
            self.safety_triggered = True
            self.get_logger().warn(f"SAFETY ALERT: Pitch angle ({np.degrees(self.pitch):.1f}°) exceeds threshold. Stopping robot!")
        elif abs(self.pitch) <= self.pitch_threshold and self.safety_triggered:
            self.safety_triggered = False
            self.get_logger().info(f"Robot back within safe pitch range ({np.degrees(self.pitch):.1f}°). Movement enabled.")

        self.roll_vel = self.angular_velocity[0]
        self.pitch_vel = self.angular_velocity[1]  # y-axis angular velocity
        self.yaw_vel = self.angular_velocity[2]    # z-axis angular velocity

    def joint_callback(self, msg):
        # The order in which joint_states comes in
        received_order = ["left_hip_motor_joint", "left_knee_joint", "left_wheel_joint", 
                          "right_hip_motor_joint", "right_knee_joint", "right_wheel_joint"]
        
        # Extract positions and velocities in the received order
        positions = {joint: msg.position[i] for i, joint in enumerate(received_order) if i < len(msg.position)}
        velocities = {joint: msg.velocity[i] for i, joint in enumerate(received_order) if i < len(msg.velocity)}
        
        # Now save them in our target order
        for joint in self.target_joints:
            if joint in positions:
                self.joint_positions[joint] = positions[joint]
                self.joint_velocities[joint] = velocities[joint]

    def nav2_callback(self, msg):
        self.linear_x_nav2 = msg.linear.x * 0.05
        self.angular_z_nav2 = msg.angular.z * 1.0
    
    def command_callback(self, msg):
        # Store previous values for change detection
        prev_left_height = self.left_height
        prev_right_height = self.right_height
        prev_linear_x = self.linear_x_joy
        prev_angular_z = self.angular_z_joy
        prev_state = hasattr(self, 'prev_state') and self.prev_state
        
        # Update current values
        self.left_height = msg.left_height
        self.right_height = msg.right_height
        self.linear_x_joy = msg.linear.x * 0.05
        self.angular_z_joy = msg.angular.z * 1.0
        self.prev_state = msg.state
        
        # Log commands when they change
        if (prev_left_height != self.left_height or
            prev_right_height != self.right_height or  
            prev_linear_x != self.linear_x_joy or 
            prev_angular_z != self.angular_z_joy or
            prev_state != msg.state):

            self.current_state = msg.state
            
            # self.get_logger().info(f"Received command - Height: {self.height:.2f}, Linear X: {self.desired_linear_x:.2f}, Angular Z: {self.desired_angular_z:.2f}, State: {msg.state}")
        
    def log_data_point(self, roll, roll_vel,pitch, pitch_vel, yaw, yaw_vel, left_torque, right_torque, 
                      left_vel, right_vel, avg_vel, desired_lin_x, desired_ang_z, left_height, right_height, average_height):
        """Log a single data point to the data_log list"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        
        data_point = {
            'timestamp': timestamp,
            'roll': roll,
            'roll_vel': roll_vel,
            'pitch': pitch,
            'pitch_vel': pitch_vel,
            'yaw': yaw,
            'yaw_vel': yaw_vel,
            'left_wheel_torque': left_torque,
            'right_wheel_torque': right_torque,
            'left_wheel_velocity': left_vel,
            'right_wheel_velocity': right_vel,
            'avg_velocity': avg_vel,
            'desired_linear_x': desired_lin_x,
            'desired_angular_z': desired_ang_z,
            'left_height': left_height,
            'right_height': right_height,
            'height': average_height
        }
        
        self.data_log.append(data_point)
        
        # Auto-save if we've reached the threshold
        if len(self.data_log) >= self.auto_save_threshold:
            self.save_data()
            
    def save_data_timer_callback(self):
        """Periodically save data to prevent loss"""
        if len(self.data_log) > 0 and self.logdata:
            self.save_data()
            
    def save_data(self):
        """Save the logged data to an Excel file"""
        if not self.data_log:
            self.get_logger().info("No data to save")
            return
            
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"werdna_log_{timestamp}.xlsx"
            filepath = os.path.join(self.log_directory, filename)
            
            # Convert to DataFrame and save
            df = pd.DataFrame(self.data_log)
            df.to_excel(filepath, index=False)
            
            self.get_logger().info(f"Saved {len(self.data_log)} data points to {filepath}")
            
            # Clear the log after saving
            self.data_log = []
            
        except Exception as e:
            self.get_logger().error(f"Failed to save data: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
