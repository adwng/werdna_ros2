#ifndef WERDNA_ODOMETRY_BROADCASTER__WERDNA_ODOMETRY_BROADCASTER_HPP_
#define WERDNA_ODOMETRY_BROADCASTER__WERDNA_ODOMETRY_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <functional>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <werdna_odometry_broadcaster/werdna_odometry_broadcaster_parameters.hpp>

namespace werdna_odometry_broadcaster
{
class WerdnaOdometryBroadcaster : public controller_interface::ControllerInterface
{

public:
  WerdnaOdometryBroadcaster();

  ~WerdnaOdometryBroadcaster() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

protected:

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

    // map from joint/sensor names to state types to state interfaces
    std::map<std::string, std::map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>>>
        state_interfaces_map_;

    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
    
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odometry_publisher_ = nullptr;

    std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> base_link_transform_publisher_ = nullptr;

    std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> realtime_base_link_transform_publisher_ = nullptr;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> joint_state_publisher_ =
        nullptr;

    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
        realtime_joint_state_publisher_ = nullptr;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_publisher_ = nullptr;

    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>>
        realtime_imu_publisher_ = nullptr;

    rclcpp::Time last_publish_time_;
    rclcpp::Time last_sensor_publish_time_;

    double pitch_;
    double pitch_vel_;
    double x_;
    double x_vel_;
    double yaw_;
    double yaw_vel_;
    double roll_;
    double roll_vel_;
    double z_;
    double z_vel_;

    double lin_vel_x_;
    double lin_vel_y_;
    double lin_vel_z_;

    double lin_acc_x_;
    double lin_acc_y_;
    double lin_acc_z_;

    // Odometry state estimator
    double odom_x_;
    double odom_y_;
    double odom_yaw_;
    double odom_x_vel_;
    double odom_y_vel_;
    double odom_yaw_vel_;
    tf2::Quaternion odom_orientation_;

    bool isStart = true;
    double left_wheel_pos_offset = 0.0;
    double right_wheel_pos_offset = 0.0;
};
}  // namespace werdna_odometry_broadcaster
#endif  // WERDNA_ODOMETRY_BROADCASTER__WERDNA_ODOMETRY_BROADCASTER_HPP_