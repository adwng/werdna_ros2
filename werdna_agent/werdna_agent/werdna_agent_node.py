import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from werdna_msgs.msg import JoyCtrlCmds

# import onnx
# import onnxruntime as ort
from scipy.spatial.transform import Rotation as R

import os

class ControlNode(Node):

    def __init__(self):
        super().__init__("control_node")

        # Subscribers
        self.imu_subscriber = self.create_subscription(Imu, 'werdna_odometry_broadcaster/imu', self.imu_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, 'werdna_odometry_broadcaster/odom', self.odom_callback, 10)
        self.joint_subscriber = self.create_subscription(JointState, 'werdna_odometry_broadcaster/joint_state', self.joint_callback, 10)
        self.command_subscriber = self.create_subscription(JoyCtrlCmds, '/werdna_control', self.command_callback, 10)

        # Publishers
        # self.wheel_controller = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 10)
        self.legs_controller = self.create_publisher(Float64MultiArray, "/position_controller/commands", 10)

        # self.model_file = "home/andrew/policy.onnx"
        
        # Load ONNX model
        # self.policy_session = ort.InferenceSession(self.model_file)
        # self.policy_input_names = [self.policy_session.get_inputs()[0].name]
        # self.policy_output_names = [self.policy_session.get_outputs()[0].name]

        self.target_joints = ["left_hip_motor_joint", "right_hip_motor_joint", "left_knee_joint", "right_knee_joint", "left_wheel_joint", "right_wheel_joint"]

        # Robot state variables
        self.height = 0
        self.left_wheel = 0
        self.right_wheel = 0
        self.desired_linear_x = 0
        self.desired_angular_z = 0

        self.linear_velocity = np.array([0.0, 0.0, 0.0])
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.projected_gravity = np.array([0.0, 0.0, 0.0])
        self.joint_positions = {joint: 0.0 for joint in self.target_joints}
        self.joint_velocities = {joint: 0.0 for joint in self.target_joints}
        self.previous_action = np.zeros(2)  # 4 joints + 2 wheels
    
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
            print(f"Error in inverse_kinematics with x={x}, y={y}: {e}")
            return 0, 0
    
    def imu_callback(self, msg):
        base_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        gravity_vector = np.array([0, 0, -1])

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = R.from_quat(base_quat).as_euler('xyz', degrees=True)  # Convert to degrees for better readability

        # Compute projected gravity vector
        # qwi = R.from_quat(base_quat).as_euler('zyx')
        # inverse_rot = R.from_euler('zyx', qwi).inv().as_matrix()
        # self.projected_gravity = np.dot(inverse_rot, gravity_vector)

        rotation_matrix = R.from_quat(base_quat).as_matrix()

        # Compute the projected gravity vector by applying the inverse rotation
        self.projected_gravity = rotation_matrix.T @ gravity_vector

        # Log the IMU data
        self.get_logger().info(f"IMU Data - Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

    def odom_callback(self, msg):
        self.linear_velocity = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        self.angular_velocity = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])

        # Log the odometry data
        self.get_logger().info(
            f"Odom - Linear Vel: x={self.linear_velocity[0]:.2f}, y={self.linear_velocity[1]:.2f}, z={self.linear_velocity[2]:.2f} | "
            f"Angular Vel: x={self.angular_velocity[0]:.2f}, y={self.angular_velocity[1]:.2f}, z={self.angular_velocity[2]:.2f}"
        )


    def joint_callback(self, msg):
        # Define the order in which JointState reports data
        received_order = ["left_hip_motor_joint", "left_knee_joint", "left_wheel_joint",
                        "right_hip_motor_joint", "right_knee_joint", "right_wheel_joint"]

        # Define the target order you want
        target_order = ["left_hip_motor_joint", "right_hip_motor_joint",
                        "left_knee_joint", "right_knee_joint",
                        "left_wheel_joint", "right_wheel_joint"]

        # Create a mapping of joint names to indices in received data
        joint_indices = {name: i for i, name in enumerate(received_order)}

        # Reorder joint positions and velocities based on the target order
        self.joint_positions = {joint: msg.position[joint_indices[joint]] for joint in target_order}
        self.joint_velocities = {joint: msg.velocity[joint_indices[joint]] for joint in target_order}



    def get_obs(self):
        obs = np.concatenate([
            self.linear_velocity,  # Linear velocity (x, y, z)
            self.angular_velocity,  # Angular velocity (x, y, z)
            self.projected_gravity,
            np.array([self.desired_linear_x, self.desired_angular_z]),  # Desired commands
            np.array([self.joint_positions[j] for j in self.target_joints[:4]]),
            np.array([self.joint_velocities[j] for j in self.target_joints]),
            self.previous_action,  # Previous actions
        ])
        return obs

    def step(self, action):
        # self.previous_action = action

        hip, knee = self.inverse_kinematics(0, self.height)

        # self.get_logger().info(f"Hip Angle: {hip}, Knee Angle: {knee}")

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
            # obs = self.get_obs()
            # action = self.policy_session.run(self.policy_output_names, {self.policy_input_names[0]: obs.reshape(1, -1)})[0].flatten()
            action  = 0
            self.step(action)
            

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
