import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from werdna_msgs.msg import JoyCtrlCmds

import onnxruntime

import os


class ControlNode(Node):

    def __init__(self):
        super().__init__("control_node")

        # Initialize variables to store required data
        self.imu_data = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "roll_rate": 0.0, "pitch_rate": 0.0, "yaw_rate": 0.0}
        self.odom_data = {"x_position": 0.0, "velocity": 0.0}
        self.joint_states = {
            "left_hip_motor_link": 0.0,
            "left_knee_link": 0.0,
            "right_hip_motor_link": 0.0,
            "right_knee_link": 0.0
        }

        # Action Spaces
        self.action_space = spaces.Box(
            low = np.array([-1.0, -1.0]),
            high = np.array([1.0, 1.0]),
            dtype=np.float32
        )

        # Observation Spaces
        self.observation_space = spaces.Box(
            np.array([-np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf]), 
            np.array([np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])
        )

        # Init Model
        model_filename = os.path.join("results", "werdna.zip")
        self.model = PPO.load(model_filename)

        # Subscribers
        self.imu_subscriber = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.joint_subscriber = self.create_subscription(JointState, '/joint_state', self.joint_callback, 10)
        self.command_subscriber = self.create_subscription(JoyCtrlCmds, '/werdna_control', self.command_callback, 10)

        # Publishers (if needed for control)
        self.wheel_controller = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 10)
        self.legs_controller = self.create_publisher(Float64MultiArray, "/position_controller/commands", 10)

        self.left_height = 0
        self.right_height = 0
        self.left_wheel = 0
        self.right_wheel = 0
        self.desired_pitch = 0
        self.desired_yaw = 0
        self.desired_position_x = 0

    def euler_from_quaternion(self, quaternion):
        """Convert quaternion to Euler angles."""
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(np.clip(sinp, -1, 1))  # Clip to handle numerical issues

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def imu_callback(self, msg):
        """Extract roll, pitch, yaw, and linear velocity from IMU data."""
        roll, pitch, yaw = self.euler_from_quaternion(msg.orientation)
        roll_rate, pitch_rate, yaw_rate = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z

        self.imu_data = {
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "roll_rate": roll_rate,
            "pitch_rate": pitch_rate,
            "yaw_rate": yaw_rate,
        }

    def odom_callback(self, msg):
        """Extract x position and velocity from odometry data."""
        self.odom_data["x_position"] = msg.pose.pose.position.x
        self.odom_data["velocity"] = msg.twist.twist.linear.x
        # self.get_logger().info(f"Odom data: X Position={self.odom_data['x_position']:.3f}, Velocity={self.odom_data['velocity']:.3f}")

    def joint_callback(self, msg):
        """Record joint positions for specific links."""
        joint_names = msg.name
        joint_positions = msg.position

        for name, position in zip(joint_names, joint_positions):
            if name in self.joint_states:
                self.joint_states[name] = position

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
                    hip_theta = 0
                    knee_theta = 0
                else:
                    # Fully stretched in the opposite direction along the x-axis
                    hip_theta = 0
                    knee_theta = 0
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
    
    def get_obs(self):
        
        """
        Generate the observation array for the agent.
        """
        roll = self.imu_data["roll"]
        pitch = self.imu_data["pitch"]
        yaw = self.imu_data["yaw"]
        roll_rate = self.imu_data["roll_rate"]
        pitch_rate = self.imu_data["pitch_rate"]
        yaw_rate = self.imu_data["yaw_rate"]

        x_position = self.odom_data["x_position"]
        velocity = self.odom_data["velocity"]

        if self.state2:
            self.desired_position_x = x_position

        # Construct the observation array
        state_spaces = np.array([
            0.0 - roll,  # Roll deviation from 0
            self.desired_pitch - pitch,  # Desired pitch vs current pitch
            self.desired_yaw - yaw,  # Desired yaw vs current yaw
            0.0 - roll_rate,  # Roll rate deviation from 0
            0.0 - pitch_rate,  # Pitch rate deviation from 0
            0.0 - yaw_rate,  # Yaw rate deviation from 0
            self.desired_position_x - x_position,  # Desired x position vs current x position
            0.0 - velocity  # Velocity deviation from 0
        ], dtype=np.float32)

        return state_spaces
    
    def step(self, action):

        self.left_wheel = action[0]*3
        self.right_wheel = action[1]*3

        self.left_height = np.clip(self.left_height, 0, 0.148)
        self.right_height = np.clip(self.right_height, 0, 0.148)

        left_hip, left_knee = self.inverse_kinematics(0, self.left_height)
        right_hip, right_knee = self.inverse_kinematics(0, self.right_height)

        wheel_msg = Float64MultiArray()

        legs_msg = Float64MultiArray()

        wheel_msg.data = [self.left_wheel, self.right_wheel]

        legs_msg.data = [left_hip, left_knee, right_hip, right_knee]

        self.wheel_controller.publish(wheel_msg)
        self.legs_controller.publish(legs_msg)
    
    def command_callback(self, msg):
        self.state = msg.state
        self.state2 = msg.state2
        self.left_height = msg.height
        self.right_height = msg.height
        self.desired_pitch = msg.pitch
        self.desired_yaw = msg.yaw 

        if (self.state):
            obs = self.get_obs()

            actions, _ = self.model.predict(obs)

            self.step(actions)

            
def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
