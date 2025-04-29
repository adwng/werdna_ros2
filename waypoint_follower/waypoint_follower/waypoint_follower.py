# ROS2 related stuff
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator as navigator
from nav2_simple_commander.robot_navigator import TaskResult

# Python stuff
import time
import json
from datetime import datetime
import math
import matplotlib.pyplot as plt
import cv2
import yaml
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from scipy.spatial.transform import Rotation
from tkinter import ttk
import threading
from rclpy.executors import MultiThreadedExecutor

class GUI(Node):
    def __init__(self, map_path, details_path):
        super().__init__('Web_GUI')

        # Initialize root frame
        self.root = tk.Tk()
        self.root.title("DASHBOARD")
        self.root.configure(bg='#2e3b4e')  # Background color

        # Create main frames
        self.control_frame = ttk.Frame(self.root, padding=10)
        self.control_frame.pack(side=tk.TOP, fill=tk.X)

        # Creae map frame
        self.map_frame = ttk.Frame(self.root, padding=10)
        self.map_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # Button styling
        style = ttk.Style()
        style.configure("TButton", font=('Helvetica', 12), padding=5)

        # Create buttons with padding and colors
        ttk.Button(
            self.control_frame, text="Normal Mode", command=self.normal_mode, style="TButton"
        ).pack(side=tk.LEFT, padx=5)


        # Read map params
        self.pgm_data = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
        self.flip_pgm_data = cv2.rotate(self.pgm_data, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # Load the YAML metadata
        with open(details_path, 'r') as file:
            self.map_metadata = yaml.safe_load(file)

        # Initiliaze navigator object and Create initial pose estimate for AMCL localization
        self.navigator = navigator()

        # List to store waypoints
        self.waypoints = []

        # Figure and axis for the plot
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.fig.tight_layout()
        self.canvas = None

        # Additional buttons states
        self.navigate_button = None
        self.navigate_init = False

        # Create publishers for lidar and camera modes
        self.velocity_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.current_odom_pose = None
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    '''
    Convert pixel coordinates to real world scale
    '''
    def pixel_to_world(self, x_pixel, y_pixel):
        resolution = self.map_metadata['resolution']
        origin = self.map_metadata['origin']
        x_world = x_pixel * resolution + origin[0]
        y_world = (self.pgm_data.shape[0] - y_pixel) * resolution + origin[1]  # y inverted
        return (x_world, y_world)

    def _wait_for_nav2(self):
        if self.navigate_init == False:

            self.navigator.waitUntilNav2Active()

            self.navigate_init = True
    
    # Navigation Mode
    def normal_mode(self):
        self.get_logger().info("Normal Mode Activated")
        thread = threading.Thread(target=self._wait_for_nav2, daemon=True)
        thread.start()

        # Remove existing canvas if it exists to prevent duplication
        if self.canvas is not None:
            self.canvas.get_tk_widget().destroy()

        # Instantiate navigation button if is none
        if self.navigate_button is None:
            self.navigate_button = ttk.Button(self.control_frame, text="Navigate to Waypoint", command=self.navigate_to_waypoint)
            self.navigate_button.pack(side=tk.BOTTOM, padx=5)
            self.navigate_button['state'] = "enabled"

        # Clear and reset plot
        self.ax.clear()
        self.ax.imshow(self.pgm_data, cmap='gray')
        self.ax.set_xlabel(f"Resolution: {self.map_metadata['resolution']} meters/pixel", color='white')
        self.ax.set_ylabel("Map height (pixels)", color='white')
        self.ax.set_title("MAP", color='white')

        # Connect event handlers for interaction
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect("button_release_event", self.on_release)

        # Embed the plot in the Tkinter window
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)


    '''
    Translate on click message to actual coordinates
    '''
    def on_click(self, event):
        if event.xdata is not None and event.ydata is not None:
            global x_start, y_start
            x_start, y_start = int(event.xdata), int(event.ydata)
            print(f"Clicked at pixel: ({x_start}, {y_start})")

            # Store the initial click point as a waypoint
            self.waypoints.append({"x": x_start, "y": y_start, "yaw": 0.0})

            # Draw a point where clicked
            self.ax.plot(x_start, y_start, 'ro')
            self.fig.canvas.draw()

    '''
    Translate the released direction as orientation angle
    '''
    def on_release(self, event):
        if event.xdata is not None and event.ydata is not None:
            global x_start, y_start
            x_pixel, y_pixel = int(event.xdata), int(event.ydata)

            delta_x = x_pixel - x_start
            delta_y = y_pixel - y_start
            yaw = math.atan2(delta_y, delta_x)

            # Convert pixel coordinates to world coordinates
            waypoint_start = self.pixel_to_world(x_start, y_start)

            # Update the last waypoint with world coordinates and yaw
            self.waypoints[-1]['x'] = waypoint_start[0]
            self.waypoints[-1]['y'] = waypoint_start[1]
            self.waypoints[-1]["yaw"] = yaw

            # Visualize the arrow for orientation
            self.ax.arrow(x_start, y_start, delta_x, delta_y, head_width=5, head_length=5, fc='blue', ec='blue')
            self.fig.canvas.draw()

            print(f"Waypoint at world coordinates: {waypoint_start}, with yaw: {yaw} radians")

    '''
    Main Navigation Function
    '''
    def navigate_to_waypoint(self):
        if self.waypoints:
            self.get_logger().info("Starting navigation to waypoints...")
            self.navigate_button['state'] = 'disabled'

        results = []

        poses = [self.create_pose(wp['x'], wp['y'], wp['yaw'], nav=self.navigator) for wp in self.waypoints]

        for i, pose in enumerate(poses):
            start_time = time.time()
            self.navigator.followWaypoints([pose])

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    print(f'Navigating to waypoint {i + 1}/{len(poses)}')
                    

            time_taken = time.time() - start_time
            result = self.navigator.getResult()

            if self.current_odom_pose:
                p = self.current_odom_pose.position
                o = self.current_odom_pose.orientation
                final_x, final_y = p.x, p.y
                final_yaw = self.get_yaw_from_quaternion(o)
            else:
                final_x = final_y = final_yaw = None

            results.append({
                'waypoint': {
                    'x': pose.pose.position.x,
                    'y': pose.pose.position.y,
                    'yaw': self.get_yaw_from_quaternion(pose.pose.orientation),
                },
                'status': str(result),
                'time_taken': time_taken,
                'final_pose': {
                    'x': final_x,
                    'y': final_y,
                    'yaw': final_yaw,
                } 
            })

            print(f"Waypoint {i + 1} result: {results[-1]}")

        # Save results
        self.save_navigation_log(results)
        self.waypoints.clear()
        self.stop_navigation()

    def get_yaw_from_quaternion(self, q):
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        roll, pitch, yaw = rot.as_euler('xyz')
        return yaw
        
    def save_navigation_log(self, data):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'navigation_log_{timestamp}.json'
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Navigation log saved to {filename}")

    '''
    Create pose messages    
    '''
    def create_pose(self, x, y, yaw, nav):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = nav.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y

        # Convert yaw to quaternion
        qx, qy, qz, qw = self.get_quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.y = qy
        pose.pose.orientation.x = qx
        pose.pose.orientation.w = qw

        return pose

    '''
    Convert euler angles to quartenion
    '''
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input:
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output:
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return qx, qy, qz, qw

    '''
    Cancel GUI and Navigation related processes
    '''
    def stop_navigation(self):
        self.get_logger().info("Stopping navigation...")
        self.navigator.cancelTask()

        # Clear waypoints and reset the navigation state
        self.waypoints.clear()

        # Remove the navigation canvas if it exists
        if self.canvas is not None:
            self.canvas.get_tk_widget().destroy()
            self.canvas = None
        
        if self.navigate_button is not None:
            self.navigate_button.destroy()
            self.navigate_button = None

        # Clear the plot to remove navigation arrows/points
        self.ax.clear()
        self.ax.imshow(self.pgm_data, cmap='gray')
        self.ax.set_xlabel(f"Resolution: {self.map_metadata['resolution']} meters/pixel", color='white')
        self.ax.set_ylabel("Map height (pixels)", color='white')
        self.ax.set_title("MAP", color='white')

        # Ensure the control frame is still active and visible
        self.control_frame.tkraise()

        # Log a message confirming return to the control frame
        self.get_logger().info("Returned to the control frame.")

    '''
    Loops the Frame
    '''
    def gui(self):
        self.root.mainloop()

    def odom_callback(self, msg):
        self.current_odom_pose = msg.pose.pose

def main():

    map_path = "/home/andrew/map.pgm"
    details_path = "/home/andrew/map.yaml"

    rclpy.init()
    gui = GUI(map_path=map_path, details_path=details_path)

    executor = MultiThreadedExecutor()
    executor.add_node(gui)

    # Start ROS spinning in background
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    # Launch your GUI mainloop (blocking)
    gui.gui()

    gui.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

