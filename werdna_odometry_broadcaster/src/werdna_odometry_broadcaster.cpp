#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#define EPSILON 1e-3
#define EPSILON_YAW_COMPENSATION 1e-1
#define NUM_JOINTS 6

#include "werdna_odometry_broadcaster/werdna_odometry_broadcaster.hpp"

namespace werdna_odometry_broadcaster
{

WerdnaOdometryBroadcaster::WerdnaOdometryBroadcaster() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn WerdnaOdometryBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception &e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration WerdnaOdometryBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interface_config;
  command_interface_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interface_config;

}

controller_interface::InterfaceConfiguration WerdnaOdometryBroadcaster::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::ALL
  };
}

controller_interface::return_type WerdnaOdometryBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{

  double right_hip_pos, right_hip_vel;
  double right_knee_pos, right_knee_vel;
  double left_hip_pos, left_hip_vel;
  double left_knee_pos, left_knee_vel;
  double left_wheel_pos, left_wheel_vel;
  double right_wheel_pos, right_wheel_vel;
  double orientation_w, orientation_x, orientation_y, orientation_z;
  double imu_yaw_vel;

  try
  {
    //read joint states from hardware interfaces
    left_hip_pos = state_interfaces_map_.at(params_.left_hip_name).at("position").get().get_value();
    left_hip_vel = state_interfaces_map_.at(params_.left_hip_name).at("velocity").get().get_value();
    // left_hip_force = state_interfaces_map_.at(params_.left_hip_name).at("effort").get().get_value();
    // left_knee_pos = state_interfaces_map_.at(params_.left_knee_name).at("position").get().get_value();
    left_knee_pos = left_hip_pos * 2.0;
    // left_knee_vel = state_interfaces_map_.at(params_.left_knee_name).at("velocity").get().get_value();
    left_knee_vel = left_hip_vel;
    // left_knee_force = state_interfaces_map_.at(params_.left_knee_name).at("effort").get().get_value();
    left_wheel_pos = state_interfaces_map_.at(params_.left_wheel_name).at("position").get().get_value();
    left_wheel_vel = state_interfaces_map_.at(params_.left_wheel_name).at("velocity").get().get_value();
    // left_wheel_force = state_interfaces_map_.at(params_.left_wheel_name).at("effort").get().get_value();

    right_hip_pos = state_interfaces_map_.at(params_.right_hip_name).at("position").get().get_value();
    right_hip_vel = state_interfaces_map_.at(params_.right_hip_name).at("velocity").get().get_value();    
    right_knee_pos = right_hip_pos * 2.0;
    right_knee_vel = right_hip_vel;
    // right_hip_force = state_interfaces_map_.at(params_.right_hip_name).at("effort").get().get_value();
    // right_knee_pos = state_interfaces_map_.at(params_.right_knee_name).at("position").get().get_value();
    // right_knee_vel = state_interfaces_map_.at(params_.right_knee_name).at("velocity").get().get_value();
    // right_knee_force = state_interfaces_map_.at(params_.right_knee_name).at("effort").get().get_value();
    
    right_wheel_pos = state_interfaces_map_.at(params_.right_wheel_name).at("position").get().get_value();
    right_wheel_vel = state_interfaces_map_.at(params_.right_wheel_name).at("velocity").get().get_value();
    // right_wheel_force = state_interfaces_map_.at(params_.right_wheel_name).at("effort").get().get_value();

    //IMU States
    // read IMU states from hardware interface
    pitch_vel_ = state_interfaces_map_.at(params_.imu_sensor_name).at("angular_velocity.y").get().get_value();
    roll_vel_ = state_interfaces_map_.at(params_.imu_sensor_name).at("angular_velocity.x").get().get_value();
    imu_yaw_vel = state_interfaces_map_.at(params_.imu_sensor_name).at("angular_velocity.z").get().get_value();
    lin_acc_x_ = state_interfaces_map_.at(params_.imu_sensor_name).at("linear_acceleration.x").get().get_value();
    lin_acc_y_ = state_interfaces_map_.at(params_.imu_sensor_name).at("linear_acceleration.y").get().get_value();
    lin_acc_z_ = state_interfaces_map_.at(params_.imu_sensor_name).at("linear_acceleration.z").get().get_value();
    orientation_w = state_interfaces_map_.at(params_.imu_sensor_name).at("orientation.w").get().get_value();
    orientation_x = state_interfaces_map_.at(params_.imu_sensor_name).at("orientation.x").get().get_value();
    orientation_y = state_interfaces_map_.at(params_.imu_sensor_name).at("orientation.y").get().get_value();
    orientation_z = state_interfaces_map_.at(params_.imu_sensor_name).at("orientation.z").get().get_value();
    
  }

  catch (const std::out_of_range &e)
  {
    RCLCPP_INFO(get_node()->get_logger(), "failed to read joint states from hardware interface");
    return controller_interface::return_type::ERROR;
  }

  //Read offset of the wheels positon and velocity
  if (isStart)
  {
    left_wheel_pos_offset = left_wheel_pos;
    right_wheel_pos_offset = right_wheel_pos;
    isStart = false;

  }

    // convert orientation to Euler angles
  tf2::Quaternion q(
    orientation_x,
    orientation_y,
    orientation_z,
    orientation_w
  );

  tf2::Matrix3x3 m(q);
  double imu_roll, imu_pitch, imu_yaw;
  m.getRPY(imu_roll, imu_pitch, imu_yaw);
  roll_ = imu_roll;
  pitch_ = imu_pitch;

  // calculate linear position and velocity for each wheel from rotational position
  double right_wheel_tangential_pos = (right_wheel_pos - right_wheel_pos_offset)* params_.wheel_radius;
  double right_wheel_tangential_vel = right_wheel_vel * params_.wheel_radius;
  double left_wheel_tangential_pos = (left_wheel_pos - left_wheel_pos_offset) * params_.wheel_radius;
  double left_wheel_tangential_vel = left_wheel_vel * params_.wheel_radius;
  
  double delta_x = 0.5 * (right_wheel_tangential_pos + left_wheel_tangential_pos) - x_;

  // calculate x_, yaw_, x_vel_, and y_vel_ from tangential wheel positions/velocities
  x_ = 0.5 * (right_wheel_tangential_pos + left_wheel_tangential_pos);
  x_vel_ = 0.5 * (right_wheel_tangential_vel + left_wheel_tangential_vel);

  yaw_ = imu_yaw;
  yaw_vel_ = imu_yaw_vel;

  // use IMU to correct odometry yaw drift
  odom_yaw_ = imu_yaw;

  odom_orientation_.setRPY(roll_, pitch_, odom_yaw_);
  odom_x_ +=delta_x * cos(odom_yaw_);
  odom_y_ += delta_x * sin(odom_yaw_);
  odom_x_vel_ = x_vel_ * cos(odom_yaw_);
  odom_y_vel_ = x_vel_ * sin(odom_yaw_);
  odom_yaw_vel_ = imu_yaw_vel;

  // estimate z height
  double left_leg_length = params_.leg_length * sin(left_hip_pos) + params_.leg_length * sin(left_hip_pos + left_knee_pos);
  double right_leg_length = params_.leg_length * sin(right_hip_pos) + params_.leg_length * sin(right_hip_pos + right_knee_pos);

  z_ =(0.5 * (left_leg_length + right_leg_length));

  // // Compute Linear Velocity from Linear Acceleration
  // double dt = period.seconds();
  // double alpha = 0.98;  
  // lin_vel_x_ = alpha * (lin_vel_x_ + lin_acc_x_ * dt);
  // lin_vel_y_ = alpha * (lin_vel_y_ + lin_acc_y_ * dt);
  // lin_vel_z_ = alpha * (lin_vel_z_ + lin_acc_z_ * dt);
  

  if (params_.sensor_publish_rate && (time - last_sensor_publish_time_).seconds() > 1.0 / params_.sensor_publish_rate)
  {
    last_sensor_publish_time_ = time;
    if (realtime_odometry_publisher_->trylock())
    {
      auto &odometry_message = realtime_odometry_publisher_->msg_;

      odometry_message.header.stamp = time;
      odometry_message.pose.pose.position.x = odom_x_;
      odometry_message.pose.pose.position.y = odom_y_;
      odometry_message.pose.pose.position.z = z_ + params_.wheel_radius;
      odometry_message.pose.pose.orientation.x = odom_orientation_.x();
      odometry_message.pose.pose.orientation.y = odom_orientation_.y();
      odometry_message.pose.pose.orientation.z = odom_orientation_.z();
      odometry_message.pose.pose.orientation.w = odom_orientation_.w();
      
      odometry_message.twist.twist.linear.x = x_vel_;
      
      odometry_message.twist.twist.angular.z = yaw_vel_;

      realtime_odometry_publisher_->unlockAndPublish();
    }
    if (realtime_imu_publisher_->trylock())
    {
      auto &imu_message = realtime_imu_publisher_->msg_;
      imu_message.header.stamp = time;
      imu_message.orientation.x = orientation_x;
      imu_message.orientation.y = orientation_y;
      imu_message.orientation.z = orientation_z;
      imu_message.orientation.w = orientation_w;
      imu_message.angular_velocity.x = roll_vel_;
      imu_message.angular_velocity.y = pitch_vel_;
      imu_message.angular_velocity.z = imu_yaw_vel;
      imu_message.linear_acceleration.x = lin_acc_x_;
      imu_message.linear_acceleration.y = lin_acc_y_;
      imu_message.linear_acceleration.z = lin_acc_z_;
      realtime_imu_publisher_->unlockAndPublish();
    }
  }

   if (params_.publish_rate && (time - last_publish_time_).seconds() > 1.0 / params_.publish_rate)
    {
      last_publish_time_ = time;
      // Log the state
      // RCLCPP_INFO(get_node()->get_logger(), "rate: %f, x: %f, x_vel: %f, pitch: %f, pitch_vel: %f, yaw: %f, yaw_vel: %f, z: %f, z_vel: %f, left_wheel_normal_force: %f, right_wheel_normal_force: %f",
      // 1.0 / period.seconds(), x_, x_vel_, pitch_, pitch_vel_, yaw_, yaw_vel_, z_, z_vel_, left_wheel_normal_force, right_wheel_normal_force);
      // std::cout << "z_des: " << z_des_ << "z:" << z_ << "right_hip_pos_cmd: " << right_hip_pos_cmd << "right_hip_pos: " << state_interfaces_map_.at(params_.right_hip_name).at("position").get().get_value() << "left_hip_pos_cmd: " << left_hip_pos_cmd << "left_hip_pos: " << state_interfaces_map_.at(params_.left_hip_name).at("position").get().get_value() << std::endl;

      if (realtime_base_link_transform_publisher_->trylock())
      {
        auto &transform = realtime_base_link_transform_publisher_->msg_.transforms.front();
        transform.header.stamp = time;
        transform.transform.translation.x = odom_x_;
        transform.transform.translation.y = odom_y_;
        transform.transform.translation.z = z_ + params_.wheel_radius;
        transform.transform.rotation.x = odom_orientation_.x();
        transform.transform.rotation.y = odom_orientation_.y();
        transform.transform.rotation.z = odom_orientation_.z();
        transform.transform.rotation.w = odom_orientation_.w();
        realtime_base_link_transform_publisher_->unlockAndPublish();
      }

      if (realtime_joint_state_publisher_->trylock())
      {
        auto &joint_state_message = realtime_joint_state_publisher_->msg_;
        joint_state_message.header.stamp = time;

        joint_state_message.position[0] = left_hip_pos;
        joint_state_message.velocity[0] = left_hip_vel;
        // joint_state_message.effort[0] = state_interfaces_map_.at(params_.left_hip_name).at("effort").get().get_value();

        joint_state_message.position[1] = left_knee_pos;
        joint_state_message.velocity[1] = left_knee_vel;
        // joint_state_message.effort[1] = state_interfaces_map_.at(params_.left_knee_name).at("effort").get().get_value();

        joint_state_message.position[2] = left_wheel_pos;
        joint_state_message.velocity[2] = left_wheel_vel;
        // joint_state_message.effort[2] = state_interfaces_map_.at(params_.left_wheel_name).at("effort").get().get_value();

        joint_state_message.position[3] = right_hip_pos;
        joint_state_message.velocity[3] = right_hip_vel;
        // joint_state_message.effort[3] = state_interfaces_map_.at(params_.right_hip_name).at("effort").get().get_value();

        joint_state_message.position[4] = right_knee_pos;
        joint_state_message.velocity[4] = right_knee_vel;
        // joint_state_message.effort[4] = state_interfaces_map_.at(params_.right_knee_name).at("effort").get().get_value();

        joint_state_message.position[5] = right_wheel_pos;
        joint_state_message.velocity[5] = right_wheel_vel;
        // joint_state_message.effort[5] = state_interfaces_map_.at(params_.right_wheel_name).at("effort").get().get_value();

        realtime_joint_state_publisher_->unlockAndPublish();
      }
    }

    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn WerdnaOdometryBroadcaster::on_configure(
  const rclcpp_lifecycle::State & )
{
  auto logger = get_node()->get_logger();

  // update parameters if they have changed
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  last_publish_time_ = get_node()->now();
  last_sensor_publish_time_ = get_node()->now();

  // Initialize the publishers
  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
        odometry_publisher_);

  joint_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SystemDefaultsQoS());
  realtime_joint_state_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
          joint_state_publisher_);

  imu_publisher_ = get_node()->create_publisher<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SystemDefaultsQoS());
  realtime_imu_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>>(
          imu_publisher_);

  base_link_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::SystemDefaultsQoS());
  realtime_base_link_transform_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
          base_link_transform_publisher_);

  auto &odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = "odom";
  odometry_message.child_frame_id = "base_link";
  odometry_message.pose.covariance = {
      1e-3, 0, 0, 0, 0, 0,
      0, 1e-3, 0, 0, 0, 0,
      0, 0, 1e6, 0, 0, 0,
      0, 0, 0, 1e6, 0, 0,
      0, 0, 0, 0, 1e6, 0,
      0, 0, 0, 0, 0, 1e3};
  odometry_message.twist.covariance = {
      1e-3, 0, 0, 0, 0, 0,
      0, 1e-3, 0, 0, 0, 0,
      0, 0, 1e6, 0, 0, 0,
      0, 0, 0, 1e6, 0, 0,
      0, 0, 0, 0, 1e6, 0,
      0, 0, 0, 0, 0, 1e3};

  auto & base_link_transform_publisher_message = realtime_base_link_transform_publisher_->msg_;
  base_link_transform_publisher_message.transforms.resize(1);
  base_link_transform_publisher_message.transforms[0].header.frame_id = "odom";
  base_link_transform_publisher_message.transforms[0].child_frame_id = "base_link";

  auto &joint_state_message = realtime_joint_state_publisher_->msg_;
  joint_state_message.name = {params_.left_hip_name, params_.left_knee_name, params_.left_wheel_name, params_.right_hip_name, params_.right_knee_name, params_.right_wheel_name};
  joint_state_message.position.resize(NUM_JOINTS);
  joint_state_message.velocity.resize(NUM_JOINTS);
  joint_state_message.effort.resize(NUM_JOINTS);

  auto &imu_message = realtime_imu_publisher_->msg_;
  imu_message.header.frame_id = "base_link";
  imu_message.orientation_covariance = {
      1e-3, 0, 0,
      0, 1e-3, 0,
      0, 0, 1e-3};
  imu_message.angular_velocity_covariance = {
      1e-3, 0, 0,
      0, 1e-3, 0,
      0, 0, 1e-3};
  
  RCLCPP_INFO(get_node()->get_logger(), "Configuration Successful");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WerdnaOdometryBroadcaster::on_activate(
  const rclcpp_lifecycle::State &)
{

  for (auto &state_interface : state_interfaces_)
  {
    state_interfaces_map_[state_interface.get_prefix_name()].emplace(
      state_interface.get_interface_name(),
      std::ref(state_interface));
  }
  
  RCLCPP_DEBUG(get_node()->get_logger(), "Activation Successful");
  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn WerdnaOdometryBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Deactivation Successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WerdnaOdometryBroadcaster::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WerdnaOdometryBroadcaster::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace werdna_odometry_broadcaster

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    werdna_odometry_broadcaster::WerdnaOdometryBroadcaster,
    controller_interface::ControllerInterface)