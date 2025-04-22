#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <chrono>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "werdna_msgs/msg/joy_ctrl_cmds.hpp"
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono;

// Declare command message and timing variables
auto control_command = werdna_msgs::msg::JoyCtrlCmds();
steady_clock::time_point last_toggle_time_state = steady_clock::now();
const int button_toggle_delay_ms = 3000;  // Toggle delay in milliseconds
const double max_height = 0.148;         // Maximum height value
const double linear_scale = 1.0;         // Scale factor for linear velocity
const double angular_scale = 1.0;        // Scale factor for angular velocity

class PublishingSubscriber : public rclcpp::Node {
public:
    PublishingSubscriber()
        : Node("werdna_teleop_gamepad_node")
    {
        // Subscribe to joystick input
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&PublishingSubscriber::joy_callback, this, _1));

        // Publisher for control commands
        publisher_ = this->create_publisher<werdna_msgs::msg::JoyCtrlCmds>("werdna_control", 10);
    }

private:
    // Base heights that are controlled by buttons
    double base_left_height = 0.0;
    double base_right_height = 0.0;

    // Helper function to handle toggle logic
    bool toggle_state(bool current_state, bool button_pressed, steady_clock::time_point &last_toggle_time) {
        auto now = steady_clock::now();
        if (button_pressed && duration_cast<milliseconds>(now - last_toggle_time).count() > button_toggle_delay_ms) {
            last_toggle_time = now;
            return !current_state;
        }
        return current_state;
    }

    // Helper function to clamp values between min and max
    double clamp(double value, double min_val, double max_val) {
        return std::max(min_val, std::min(value, max_val));
    }

    // Process joystick input and map it to control commands
    void process_joystick_input(const sensor_msgs::msg::Joy::SharedPtr msg_joy) {
        // Toggle state using button 3
        control_command.state = toggle_state(control_command.state, msg_joy->buttons[3], last_toggle_time_state);

        // Adjust height and commands if in active state
        if (control_command.state) {
            const double height_step = 0.005;  // Adjust height by 5 mm per button press
            
            // Adjust base heights together using button presses
            if (msg_joy->buttons[2]) {
                // Increase both base heights
                base_left_height = std::min(base_left_height + height_step, max_height);
                base_right_height = std::min(base_right_height + height_step, max_height);
            }

            if (msg_joy->buttons[1]) {
                // Decrease both base heights
                base_left_height = std::max(base_left_height - height_step, 0.0);
                base_right_height = std::max(base_right_height - height_step, 0.0);
            }
            
            // Apply differential adjustment from axis 2
            double differential = msg_joy->axes[2] * max_height * 0.5; // Scale factor for smoother control
            
            // Calculate final heights by applying differential to base heights
            control_command.left_height = clamp(base_left_height - differential, 0.0, max_height);
            control_command.right_height = clamp(base_right_height + differential, 0.0, max_height);
            
            // Map joystick axes to linear.x and angular.z
            control_command.linear.x = msg_joy->axes[1] * linear_scale;  // Forward/backward motion
            control_command.angular.z = msg_joy->axes[0] * angular_scale; // Rotation left/right
        }        

        // auto logger = this->get_logger();
        // RCLCPP_INFO(
        //     logger, 
        //     "State: %s, Base Left: %.3f, Base Right: %.3f, Left Output: %.3f, Right Output: %.3f, Linear.x: %.3f, Angular.z: %.3f", 
        //     control_command.state ? "true" : "false",
        //     base_left_height,
        //     base_right_height,
        //     control_command.left_height, 
        //     control_command.right_height, 
        //     control_command.linear.x, 
        //     control_command.angular.z
        // );
    }

    // Joystick callback function
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg_joy) {
        process_joystick_input(msg_joy);
        publisher_->publish(control_command);
    }

    // ROS2 subscription and publisher
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<werdna_msgs::msg::JoyCtrlCmds>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublishingSubscriber>());
    rclcpp::shutdown();
    return 0;
}