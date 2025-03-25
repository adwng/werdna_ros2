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

class ControlNode(Node):

    def __init__(self):
        super().__init__("control_node")

        # Subscribers
        self.imu_subscriber = self.create_subscription(Imu, 'odometry_broadcaster/imu', self.imu_callback, 10)
        # self.odom_subscriber = self.create_subscription(Odometry, 'odometry_broadcaster/odom', self.odom_callback, 10)
        self.joint_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.command_subscriber = self.create_subscription(JoyCtrlCmds, '/werdna_control', self.command_callback, 10)

        # Publishers
        # self.wheel_controller = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 10)
        self.legs_controller = self.create_publisher(Float64MultiArray, "/position_controller/commands", 10)

        # Load PyTorch model
        self.model_file = "model_2000.pt"
        self.device = torch.device('cpu')  # RPi4 will use CPU
        self.policy = None  # Placeholder for policy object

        # Attempt to load the model
        if self.load_model(self.model_file):
            self.policy = self.get_inference_policy(self.device)

        self.target_joints = ["left_hip_motor_joint", "right_hip_motor_joint", "left_knee_joint", "right_knee_joint", "left_wheel_joint", "right_wheel_joint"]

        # Robot state variables
        self.height = 0
        self.left_wheel = 0
        self.right_wheel = 0
        self.desired_linear_x = 0
        self.desired_angular_z = 0

        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.projected_gravity = np.array([0.0, 0.0, 0.0])
        self.joint_positions = {joint: 0.0 for joint in self.target_joints}
        self.joint_velocities = {joint: 0.0 for joint in self.target_joints}
        self.previous_action = np.zeros(2)  # 4 joints + 2 wheels
    
    def load_model(self, path, load_optimizer=True):
        try:
            loaded_dict = torch.load(path, map_location=self.device)
            
            # Ensure the loaded dictionary contains expected keys
            # if isinstance(loaded_dict, dict) and 'model_state_dict' in loaded_dict:
            #     if hasattr(self, 'policy') and hasattr(self.policy, 'load_state_dict'):
            #         self.policy.load_state_dict(loaded_dict['model_state_dict'])
            #     else:
            #         self.get_logger().error("Loaded model lacks a valid policy structure.")
            #         return False
                
            #     if load_optimizer and hasattr(self, 'optimizer'):
            #         if 'optimizer_state_dict' in loaded_dict:
            #             self.optimizer.load_state_dict(loaded_dict['optimizer_state_dict'])
            #         else:
            #             self.get_logger().warning("Optimizer state not found in checkpoint.")
                
            #     self.current_learning_iteration = loaded_dict.get('iter', 0)
            #     self.get_logger().info(f"Model successfully loaded from {path}, iteration {self.current_learning_iteration}")
            #     return True
            # else:
            #     self.get_logger().error("Invalid model file structure. Missing 'model_state_dict'.")
            #     return False

            self.policy.load_state_dict(loaded_dict["model_state_dict"])
            if load_optimizer:
                self.policy.load_state_dict(loaded_dict['optimizer_state_dict'])
            self.current_learning_iteration = loaded_dict['iter']
        except Exception as e:
            self.get_logger().error(f"Failed to load PyTorch model: {e}")
            return False

    def get_inference_policy(self, device=None):
        self.policy.eval()  # Set to evaluation mode (important for dropout, batch norm, etc.)
        if device is not None:
            self.policy.to(device)
        return self.policy.act_inference 
    
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

        self.angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        rotation_matrix = R.from_quat(base_quat).as_matrix()

        # Compute the projected gravity vector by applying the inverse rotation
        self.projected_gravity = rotation_matrix.T @ gravity_vector

    def odom_callback(self, msg):
        self.linear_velocity = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        self.angular_velocity = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])

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
            np.array([self.joint_positions[j] for j in self.target_joints[:4]]),
            np.array([self.joint_velocities[j] for j in self.target_joints]),
            self.previous_action,  # Previous actions
        ])
        return obs

    def step(self, action):
        exec_actions = np.clip(action, -2, 2)
        self.previous_action = exec_actions

        hip, knee = self.inverse_kinematics(0, self.height)

        self.get_logger().info(f"Actions (1): {exec_actions[0]}, Actions (2): {exec_actions[1]}")

        # wheel_cmd = Float64MultiArray()
        leg_cmd = Float64MultiArray()

        # First two actions control wheels
        # wheel_cmd.data = [action[0], action[1]]
        
        # Remaining actions control the leg joints
        leg_cmd.data = [hip, knee, hip, knee]
        
        # self.wheel_controller.publish(wheel_cmd)
        self.legs_controller.publish(leg_cmd)

    def command_callback(self, msg):
        self.height = msg.height
        self.desired_linear_x = msg.linear.x
        self.desired_angular_z = msg.angular.z 

        if msg.state:
            obs = self.get_obs()
            
            try:
                # Convert numpy array to torch tensor
                with torch.no_grad():  # Disable gradient calculations for inference
                    input_tensor = torch.FloatTensor(obs).to(self.device)
                    
                    # Based on the training code, policy might be the act_inference method
                    if callable(self.policy):
                        # Try to get actions using the inference policy
                        action = self.policy(input_tensor)
                        
                        # Convert to numpy if it's a tensor
                        if isinstance(action, torch.Tensor):
                            action = action.cpu().numpy()
                            
                        # If action is a tuple or dict (might contain actions, values, etc.)
                        if isinstance(action, tuple) and len(action) > 0:
                            action = action[0]  # First element is typically the actions
                        elif isinstance(action, dict) and 'actions' in action:
                            action = action['actions']
                            
                        # Make sure it's flattened
                        if hasattr(action, 'flatten'):
                            action = action.flatten()
                        
                        self.get_logger().info(f"Model inference successful. Action shape: {action.shape}")
                    else:
                        self.get_logger().error("Policy is not callable")
                        action = np.array([0, 0])  # Default safe action
                
                self.step(action)
            except Exception as e:
                self.get_logger().error(f"Error running inference: {e}")
                # Fall back to a safe action
                self.step(np.array([0, 0]))

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()