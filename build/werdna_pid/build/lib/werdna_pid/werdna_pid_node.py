import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from werdna_msgs.msg import JoyCtrlCmds

from scipy.spatial.transform import Rotation as R

import os
import time
import yaml

class ControlNode(Node):

    def __init__(self):
        super().__init__("control_node")
        
        self.get_logger().info("Initializing Werdna Control Node...")

        # Subscribers
        self.get_logger().info("Setting up subscribers...")
        self.imu_subscriber = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.joint_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.command_subscriber = self.create_subscription(JoyCtrlCmds, '/werdna_control', self.command_callback, 10)
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
        self.height = 0
        self.left_wheel = 0
        self.right_wheel = 0
        self.desired_linear_x = 0
        self.desired_angular_z = 0

        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.projected_gravity = np.array([0.0, 0.0, 0.0])
        self.joint_positions = {joint: 0.0 for joint in self.target_joints}
        self.joint_velocities = {joint: 0.0 for joint in self.target_joints}
        self.previous_action = np.zeros(2)  # 2 wheels
        self.velocity_des = np.zeros(2)

        
        # Safety parameters

        self.pitch_threshold = 0.7  
        self.filtered_angular_velocity = np.zeros(3)
        self.alpha = 0.3  # Tune between 0.1 and 0.3
        self.velocity_integral = 0.0
        self.velocity_integral_max = 0.5  # Anti-windup clamp value (adjust as needed)


        self.safety_triggered = False
        self.yaw_offset = None
        
        self.get_logger().info("Safety parameters configured:")
        self.get_logger().info(f"  - Pitch threshold: {self.pitch_threshold:.2f} radians ({np.degrees(self.pitch_threshold):.1f} degrees)")
        
        self.get_logger().info("Werdna Control Node initialization complete!")

        # PID Controller parameters
        self.balance_kp = 0
        self.balance_kd = 0
        self.steer_kp = 0.0
        self.steer_kd = 0.0
        self.pitch_offset = 0.0

        # Internal PID state
        self.pitch = 0.0
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch_vel = 0.0
        self.yaw_vel = 0.0

        config_file = "/home/andrew/werdna_ws/src/werdna_ros2/pid.yaml"

        self.load_config(config_file)

        timer_period = 0.01 # every 0.02 seconds //50Hz
        self.runtime = self.create_timer(timer_period, self.runtime_callback)
    
    def load_config(self, config_file):
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)

        # PID Controller parameters
        self.balance_kp = config["balance_kp"]
        self.balance_kd = config["balance_kd"]
        self.steer_kp = config["steer_kp"]
        self.steer_kd = config["steer_kd"]
        self.velocity_kp = config["drive_kp"]
        self.velocity_ki = config["drive_ki"]
        self.pitch_offset = config["offset"]
    
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

        # === OUTER VELOCITY LOOP ===
        # Get current wheel velocities (convert wheel angular speeds to linear velocity using wheel_radius)
        wheel_radius = 0.0855  # Adjust as needed for your robot
        left_vel = self.joint_velocities["left_wheel_joint"]
        right_vel = self.joint_velocities["right_wheel_joint"]
        avg_velocity = 0.5 * (left_vel + right_vel) 

        velocity_error = self.desired_linear_x - avg_velocity

        self.accel_gain = 0.01  # Tuning parameter; adjust to your needs

        # Compute feedforward pitch contribution from acceleration
        pitch_ff = self.accel_gain * self.linear_accelerations[0]

        # Instead of integrating velocity error, here we combine the velocity error feedback with feedforward:
        target_pitch = velocity_error * self.velocity_kp + self.velocity_ki * self.velocity_integral + pitch_ff
        target_pitch = np.clip(target_pitch, -0.1, 0.1)


        # === INNER PITCH BALANCE CONTROLLER ===
        pitch_error = (target_pitch + self.pitch_offset) - self.pitch
        pitch_rate_error = -self.pitch_vel
        balance_output = self.balance_kp * pitch_error + self.balance_kd * pitch_rate_error

        # === YAW STEERING CONTROLLER ===
        if self.yaw_offset is not None:
            yaw_error = self.desired_angular_z - self.yaw_vel
            steer_output = self.steer_kp * yaw_error + self.steer_kd * (-self.yaw_vel)
        else:
            steer_output = 0.0

        # === TORQUE COMMANDS ===
        left_wheel_torque = balance_output - steer_output
        right_wheel_torque = balance_output + steer_output
        action = np.array([left_wheel_torque, right_wheel_torque])

        self.step(action)

        # === DEBUG INFO ===
        self.get_logger().info(
            "\n========== PID CONTROL ==========\n"
            f"Target Pitch        : {target_pitch:.3f} rad\n"
            f"Pitch               : {self.pitch:.3f} rad\n"
            f"Pitch Error         : {pitch_error:.3f} rad\n"
            f"Pitch Rate          : {self.pitch_vel:.3f} rad/s\n"
            f"Yaw Velocity        : {self.yaw_vel:.3f} rad/s\n"
            f"Desired Lin Vel X   : {self.desired_linear_x:.2f}\n"
            f"Estimated Avg Vel   : {avg_velocity:.2f}\n"
            f"Wheel Commands      : {action}\n"
            "=================================="
        )


    def step(self, action):
        
        # velocity_des = np.zeros(2)
        exec_actions = np.clip(action, -1.0, 1.0)

        hip, knee = self.inverse_kinematics(0, 0.0)

        wheel_cmd = Float64MultiArray()
        leg_cmd = Float64MultiArray()

        # First two actions control wheels
        wheel_cmd.data = [float(exec_actions[0] * 1.0), float(exec_actions[1] * 1.0)]
        
        # Remaining actions control the leg joints
        leg_cmd.data = [hip, hip]
        
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
        self.filtered_angular_velocity = (
            self.alpha * raw_angular_velocity + (1 - self.alpha) * self.filtered_angular_velocity
        )

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


    def command_callback(self, msg):
        # Store previous values for change detection
        prev_height = self.height
        prev_linear_x = self.desired_linear_x
        prev_angular_z = self.desired_angular_z
        prev_state = hasattr(self, 'prev_state') and self.prev_state
        
        # Update current values
        self.height = msg.height
        self.desired_linear_x = msg.linear.x
        self.desired_angular_z = msg.angular.z
        self.prev_state = msg.state
        
        # Log commands when they change
        if (prev_height != self.height or 
            prev_linear_x != self.desired_linear_x or 
            prev_angular_z != self.desired_angular_z or
            prev_state != msg.state):

            self.current_state = msg.state
            
            self.get_logger().info(f"Received command - Height: {self.height:.2f}, Linear X: {self.desired_linear_x:.2f}, Angular Z: {self.desired_angular_z:.2f}, State: {msg.state}")
        
    
def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
