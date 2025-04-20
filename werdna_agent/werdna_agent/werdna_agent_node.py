import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from werdna_msgs.msg import JoyCtrlCmds

# Import torch for model inference
import torch
from scipy.spatial.transform import Rotation as R

import os
from datetime import datetime
import pandas as pd
import time

class ControlNode(Node):

    def __init__(self):
        super().__init__("control_node")
        
        self.get_logger().info("Initializing Werdna Control Node...")

        # Subscribers
        self.get_logger().info("Setting up subscribers...")
        self.imu_subscriber = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.joint_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.command_subscriber = self.create_subscription(JoyCtrlCmds, '/werdna_control', self.command_callback, 10)
        self.get_logger().info("Subscribers initialized")

        # Publishers
        self.get_logger().info("Setting up publishers...")
        self.wheel_controller = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 10)
        self.legs_controller = self.create_publisher(Float64MultiArray, "/position_controller/commands", 10)
        self.get_logger().info("Publishers initialized")

        # Load PyTorch model
        self.model_file = "/home/andrew/werdna_ws/src/werdna_ros2/policy_velocity.pt"
        self.get_logger().info(f"Loading TorchScript model from: {self.model_file}")
        
        try:
            # Load TorchScript Model
            start_time = time.time()
            self.policy_model = torch.jit.load(self.model_file, map_location="cpu")
            self.policy_model.eval()
            load_time = time.time() - start_time
            self.get_logger().info(f"Model successfully loaded in {load_time:.2f} seconds!")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            self.policy_model = None

        self.target_joints = ["left_hip_motor_joint", "right_hip_motor_joint", "left_knee_joint", "right_knee_joint", "left_wheel_joint", "right_wheel_joint"]
        self.get_logger().info(f"Target joints configured: {self.target_joints}")

        # Robot state variables
        self.current_state = False
        self.height = 0
        self.left_wheel = 0
        self.right_wheel = 0
        self.desired_linear_x = 0
        self.desired_angular_z = 0

        # State Spaces
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.velocity_des = np.zeros(2)
        self.projected_gravity = np.array([0.0, 0.0, 0.0])
        self.joint_positions = {joint: 0.0 for joint in self.target_joints}
        self.joint_velocities = {joint: 0.0 for joint in self.target_joints}
        self.previous_action = np.zeros(2)  # 2 wheels

        # Parameters
        self.wheel_joint_torque_limit = 2.0
        self.wheel_joint_damping = 0.5
        
        # Safety parameters
        self.pitch = 0.0
        self.pitch_threshold = 0.7  
        self.safety_triggered = False
        self.yaw_offset = None
        
        self.get_logger().info("Safety parameters configured:")
        self.get_logger().info(f"  - Pitch threshold: {self.pitch_threshold:.2f} radians ({np.degrees(self.pitch_threshold):.1f} degrees)")
        
        self.get_logger().info("Werdna Control Node initialization complete!")

        timer_period = 0.02 # every 0.02 seconds //50Hz
        self.runtime = self.create_timer(timer_period, self.runtime_callback)

        self.get_logger().info("Werdna Control Node initialization complete!")

        # Add data logging for Excel
        self.data_log = []
        self.log_columns = [
            'timestamp', 
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
            'height'
        ]
        
        # Configure logging frequency (e.g., log every 5 control cycles)
        self.logdata = False
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
    
    def runtime_callback(self):
        if self.current_state:
            # Check if safety stop is triggered due to excessive pitch
            if self.safety_triggered:
                hip, knee = self.inverse_kinematics(0.0,0.01) 
                leg_cmd = Float64MultiArray()
                leg_cmd.data = [float(hip), float(hip)]
                self.legs_controller.publish(leg_cmd)

                wheel_cmd = Float64MultiArray()
                wheel_cmd.data = [0.0, 0.0]
                self.wheel_controller.publish(wheel_cmd)
                
                self.previous_action = np.zeros(2)
                return

            if self.policy_model is None:
                self.get_logger().error("Cannot execute command: Model not loaded correctly")
                return
                
            # action = None
            obs = self.get_obs()
            obs_tensor = torch.tensor(obs).unsqueeze(0)

            wheel_radius = 0.0855  # Adjust as needed for your robot
            left_vel = self.joint_velocities["left_wheel_joint"] * wheel_radius
            right_vel = self.joint_velocities["right_wheel_joint"] * wheel_radius
            avg_velocity = (0.5 * (left_vel + right_vel)) 
            
            # Measure inference time
            start_time = time.time()
            with torch.no_grad():
                action = self.policy_model(obs_tensor).numpy().flatten()
            inference_time = time.time() - start_time
            
            if self.logdata:
                # Log data for Excel export
                self.log_counter += 1
                if self.log_counter >= self.log_frequency:
                    self.log_counter = 0
                    self.log_data_point(
                        self.pitch,
                        self.pitch_vel,
                        self.yaw,
                        self.yaw_vel,
                        self.previous_action[0],
                        self.previous_action[1],
                        left_vel,
                        right_vel,
                        avg_velocity,
                        self.desired_linear_x,
                        self.desired_angular_z,
                        self.height
                    )
           
            self.step(action)
        else:
            self.step(np.array([0.0,0.0]))
            return
        
         # Log inference time occasionally
        if hasattr(self, 'last_inference_log_time') and time.time() - self.last_inference_log_time < 1.0:
            pass  # Skip logging if less than 5 seconds has passed
        else:
            self.last_inference_log_time = time.time()
            self.get_logger().info(
                "\n========== POLICY INFERENCE ==========\n"

                f"Actions: {action}\n"
                f"Velocity Desired: {self.velocity_des}\n"
                f"Observations:\n"
                f"  - Angular Velocity   : {self.angular_velocity}\n"
                f"  - Projected Gravity  : {self.projected_gravity}\n"
                f"  - Roll Pitch Yaw     : {self.roll:.2f}, {self.pitch:.2f}, {self.yaw:.2f} \n"
                f"  - Desired Linear X   : {self.desired_linear_x:.2f}\n"
                f"  - Desired Angular Z  : {self.desired_angular_z:.2f}\n"
                f"  - Joint Positions    : {[round(self.joint_positions[j], 3) for j in self.target_joints[:4]]}\n"
                f"  - Joint Velocities   : {[round(self.joint_velocities[j], 3) for j in self.target_joints]}\n"
                f"  - Previous Actions   : {self.previous_action}\n"
                "======================================"
            )

    def step(self, action):
        
        # velocity_des = np.zeros(2)
        exec_actions = np.clip(action, -100.0, 100.0)

        # current wheel velocities
        joint_vel = np.array([self.joint_velocities["left_wheel_joint"], self.joint_velocities["right_wheel_joint"]])

        for i in range(len(joint_vel)):
            action_min = joint_vel[i] - self.wheel_joint_torque_limit/self.wheel_joint_damping
            action_max = joint_vel[i] + self.wheel_joint_torque_limit/self.wheel_joint_damping
            self.previous_action[i] = exec_actions[i]
            exec_actions[i] = max(action_min/self.wheel_joint_damping, min(action_max / self.wheel_joint_damping, exec_actions[i]))
            self.velocity_des[i] = exec_actions[i] * self.wheel_joint_damping

        hip, knee = self.inverse_kinematics(0, self.height)

        wheel_cmd = Float64MultiArray()
        leg_cmd = Float64MultiArray()

        # First two actions control wheels
        wheel_cmd.data = [float(self.velocity_des[0]), float(self.velocity_des[1])]
        
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
        gravity_vector = np.array([0, 0, -1])

        # Store raw angular velocity
        self.angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        
        # Extract Euler angles
        euler_angles = R.from_quat(base_quat).as_euler('xyz')
        self.roll = euler_angles[0]
        self.pitch = euler_angles[1]
        raw_yaw = euler_angles[2]  # Store raw yaw before adjustment
        
        # Initialize yaw offset once
        if self.yaw_offset is None:
            self.yaw_offset = raw_yaw
            self.get_logger().info(f"Yaw Offset Initialized: {np.degrees(self.yaw_offset):.2f}")
        
        # Calculate corrected yaw (this is just for display and control purposes)
        self.yaw = raw_yaw - self.yaw_offset
        
        # Compute rotation matrix from quaternion (represents current orientation)
        rotation_matrix = R.from_quat(base_quat).as_matrix()
        
        # Project global gravity vector into robot frame
        robot_frame_gravity = rotation_matrix.T @ gravity_vector
        
        # Create a rotation matrix for yaw correction only
        # This rotates vectors around the z-axis by -yaw_offset
        # We use the negative of yaw_offset because we want to undo the initial yaw
        cos_offset = np.cos(-self.yaw_offset)
        sin_offset = np.sin(-self.yaw_offset)
        yaw_correction = np.array([
            [cos_offset, -sin_offset, 0],
            [sin_offset, cos_offset, 0],
            [0, 0, 1]
        ])
        
        # Apply yaw correction to get gravity in yaw-corrected frame
        self.projected_gravity = yaw_correction @ robot_frame_gravity
        
        # Assign angular velocities for control use
        self.pitch_vel = self.angular_velocity[1]  # y-axis angular velocity
        self.yaw_vel = self.angular_velocity[2]    # z-axis angular velocity
        
        # Safety check
        if abs(self.pitch) > self.pitch_threshold and not self.safety_triggered:
            self.safety_triggered = True
            self.get_logger().warn(f"SAFETY ALERT: Pitch angle ({np.degrees(self.pitch):.1f}°) exceeds threshold. Stopping robot!")
        elif abs(self.pitch) <= self.pitch_threshold and self.safety_triggered:
            self.safety_triggered = False
            self.get_logger().info(f"Robot back within safe pitch range ({np.degrees(self.pitch):.1f}°). Movement enabled.")

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


    def get_obs(self):
        obs = np.concatenate([
            self.angular_velocity,  # Angular velocity (x, y, z)
            self.projected_gravity,
            np.array([self.desired_linear_x, self.desired_angular_z]),  # Desired commands
            np.array([self.joint_positions[joint] for joint in self.target_joints[:4]]),
            np.array([self.joint_velocities[j] for j in self.target_joints]),
            self.previous_action,  # Previous actions
        ]).astype(np.float32)
        return obs

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

    def log_data_point(self, pitch, pitch_vel, yaw, yaw_vel, left_torque, right_torque, 
                      left_vel, right_vel, avg_vel, desired_lin_x, desired_ang_z, height):
        """Log a single data point to the data_log list"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        
        data_point = {
            'timestamp': timestamp,
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
            'height': height
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
