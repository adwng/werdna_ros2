#ifndef PI3HAT_HARDWARE_INTERFACE__PI3HAT_SYSTEM_HPP_
#define PI3HAT_HARDWARE_INTERFACE__PI3HAT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <optional>
#include "motues/moteus.h"
#include "motues/pi3hat_moteus_transport.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "visibility_control.h"

namespace pi3hat_hardware_interface
{
class Pi3HatControlHardware : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(Pi3HatControlHardware);

    PI3HAT_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    PI3HAT_HARDWARE_INTERFACE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    PI3HAT_HARDWARE_INTERFACE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    PI3HAT_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    PI3HAT_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;

    PI3HAT_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    PI3HAT_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    PI3HAT_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    PI3HAT_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
private:

    using QueryResult = mjbots::moteus::Query::Result;
    using Transport = mjbots::pi3hat::Pi3HatMoteusTransport; 

    std::shared_ptr<Transport> transport;
        
    // controllers
    std::vector<std::shared_ptr<mjbots::moteus::Controller>> controllers;

    //IMU
    mjbots::pi3hat::Attitude attitude;
    std::array<double, 4> hw_state_imu_orientation_;         // x, y, z, w
    std::array<double, 3> hw_state_imu_angular_velocity_;    // x, y, z
    std::array<double, 3> hw_state_imu_linear_acceleration_; // x, y, z

    //Frames
    std::vector<mjbots::moteus::CanFdFrame> send_frames;
    std::vector<mjbots::moteus::CanFdFrame> receive_frames;

    //Callbaks
    
    // Actuator CAN config
    std::vector<int> hw_actuator_can_channels_;
    std::vector<int> hw_actuator_can_ids_;

    // Actuator parameters
    std::vector<std::string> control_modes_;
    std::vector<double> hw_actuator_position_offsets_;

    // Actuator states
    std::vector<double> hw_state_positions_;
    std::vector<double> hw_state_velocities_;

    // Actuator commands
    std::vector<double> hw_command_positions_;
    std::vector<double> hw_command_efforts_;
};

} // namespace pi3hat_hardware_interface

#endif
