#include "include/pi3hat_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <unordered_set>
#include <sched.h>
#include <sys/mman.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#define DEG_TO_RAD 0.01745329251994329576923690768489

namespace pi3hat_hardware_interface
{
hardware_interface::CallbackReturn Pi3HatControlHardware::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    mjbots::pi3hat::Pi3HatMoteusFactory::Register();

    hw_state_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_state_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_state_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());


    hw_command_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_command_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_command_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        // Set params for each joint
        hw_actuator_can_channels_.push_back(std::stoi(joint.parameters.at("can_channel")));
        hw_actuator_can_ids_.push_back(std::stoi(joint.parameters.at("can_id")));
        hw_actuator_position_offsets_.push_back(std::stod(joint.parameters.at("position_offset")));
        hw_actuator_axis_directions_.push_back(std::stoi(joint.parameters.at("axis_direction")));
        control_modes_.push_back(joint.parameters.at("control_mode"));
    }

    // onInit() -> Configure realtime scheduling
    {
        int realtime_cpu1 = 0; // first core
        int realtime_cpu2 = 1; // second core
        cpu_set_t cpuset = {};
        CPU_ZERO(&cpuset);
        CPU_SET(realtime_cpu1, &cpuset); // set first core
        CPU_SET(realtime_cpu2, &cpuset); // set second core

        const int r = ::sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
        if (r < 0)
        {
            throw std::runtime_error("Error setting CPU affinity");
        }

        std::cout << "Affinity set to CPUs " << realtime_cpu1 << " and " << realtime_cpu2 << "\n";
    }
    {
        struct sched_param params = {};
        params.sched_priority = 10;
        const int r = ::sched_setscheduler(0, SCHED_RR, &params);
        if (r < 0)
        {
            throw std::runtime_error("Error setting realtime scheduler");
        }
    }
    {
        const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
        if (r < 0)
        {
            throw std::runtime_error("Error locking memory");
        }
    }

    Transport::Options toptions;
    
    // Create a map to store CAN channel -> list of CAN IDs
    std::map<int, std::vector<int>> channel_to_can_ids_map;

    // Loop through the `hw_actuator_can_channels_` and `hw_actuator_can_ids_` to group CAN IDs by their channels
    for (size_t i = 0; i < hw_actuator_can_channels_.size(); ++i)
    {
        int can_channel = hw_actuator_can_channels_[i];
        int can_id = hw_actuator_can_ids_[i];
        
        // Add the CAN ID to the corresponding channel
        channel_to_can_ids_map[can_channel].push_back(can_id);
    }

    // Now map the CAN IDs to the appropriate channel in the transport options
    for (const auto& [can_channel, can_ids] : channel_to_can_ids_map)
    {
        for (int can_id : can_ids)
        {
            toptions.servo_map[can_id] = can_channel;  // Map each CAN ID to its channel
        }
    }

    // Configure IMU Settings Here
    toptions.attitude_rate_hz = 100;
    toptions.mounting_deg.roll = std::stod(info_.hardware_parameters.at("imu_mounting_deg.roll"));
    toptions.mounting_deg.pitch = std::stod(info_.hardware_parameters.at("imu_mounting_deg.pitch"));
    toptions.mounting_deg.yaw = std::stod(info_.hardware_parameters.at("imu_mounting_deg.yaw"));
    logging = std::stoi(info_.hardware_parameters.at("logging"));

    // Create the transport with the populated options
    transport = std::make_shared<Transport>(toptions);

    for (unsigned long i = 0; i<info_.joints.size(); i++)
    {
        mjbots::moteus::Controller::Options options;
        options.transport = transport;
        options.id = hw_actuator_can_ids_[i];
        if (control_modes_[i] == "effort")
        {
            options.position_format.position = mjbots::moteus::kIgnore;
            options.position_format.velocity = mjbots::moteus::kIgnore;
            options.position_format.feedforward_torque = mjbots::moteus::kFloat;
            options.position_format.kp_scale = mjbots::moteus::kInt8;
            options.position_format.kd_scale = mjbots::moteus::kInt8;
        }
        else if (control_modes_[i] == "velocity")
        {
            options.position_format.position = mjbots::moteus::kIgnore;
            options.position_format.velocity = mjbots::moteus::kFloat;
            options.position_format.feedforward_torque = mjbots::moteus::kIgnore;
            options.position_format.kp_scale = mjbots::moteus::kInt8;
            options.position_format.kd_scale = mjbots::moteus::kInt8;
        }
        else if (control_modes_[i] == "position")
        {
            options.position_format.position = mjbots::moteus::kFloat;
            options.position_format.velocity_limit = mjbots::moteus::kFloat;
            options.position_format.accel_limit = mjbots::moteus::kFloat;
        }

        controllers.push_back(
            std::make_shared<mjbots::moteus::Controller>(options)
        );
    }

    for (auto& c:controllers){
        int fault = c->SetStop()->values.fault;

        if (fault != 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("Pi3hatControlhardware"), "Controller with id: %d reported fault: %d", c->options().id, fault);

            return hardware_interface::CallbackReturn::ERROR;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("Pi3HatControlHardware"), "Motor: %d Stopped", c->options().id);
        }
    
    }

    return hardware_interface::CallbackReturn::SUCCESS;

}

    // Clear Faults
std::vector<hardware_interface::StateInterface> Pi3HatControlHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Add joint state interfaces
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_state_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocities_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, & hw_state_efforts_[i]));
    }

    //Add IMU State Interfaces
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "orientation.x", &hw_state_imu_orientation_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "orientation.y", &hw_state_imu_orientation_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "orientation.z", &hw_state_imu_orientation_[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "orientation.w", &hw_state_imu_orientation_[3]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "angular_velocity.x", &hw_state_imu_angular_velocity_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "angular_velocity.y", &hw_state_imu_angular_velocity_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "angular_velocity.z", &hw_state_imu_angular_velocity_[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "linear_acceleration.x", &hw_state_imu_linear_acceleration_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "linear_acceleration.y", &hw_state_imu_linear_acceleration_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "imu_sensor", "linear_acceleration.z", &hw_state_imu_linear_acceleration_[2]));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Pi3HatControlHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0u; i < info_.joints.size(); i++)
    {
        // Check the control mode of each joint and allocate accordingly
        if (control_modes_[i] == "position")  // Position control
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_command_positions_[i]));
        }
        else if (control_modes_[i] == "effort")  // Effort control
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_command_efforts_[i]));
        }
        else if (control_modes_[i] == "velocity")
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_command_velocities_[i]));
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("Pi3HatControlHardware"), 
                         "Unknown control mode for joint: %s", info_.joints[i].name.c_str());
        }
    }

    return command_interfaces;
}  

hardware_interface::CallbackReturn Pi3HatControlHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("Pi3HatControlHardware"), "Configuring ...please wait...");

     // reset values always when configuring hardware
     for (uint i = 0; i < hw_state_positions_.size(); i++)
     {
        hw_state_positions_[i] = 0;
        hw_state_velocities_[i] = 0;
        if (control_modes_[i] == "position")
        {
            hw_command_positions_[i] = 0.0;
        }
        else if (control_modes_[i] == "effort")
        {
            hw_command_efforts_[i] = 0.0;
        }
        else if (control_modes_[i] == "velocity")
        {
            hw_command_velocities_[i] = 0.0;
        }
     }

    RCLCPP_INFO(rclcpp::get_logger("Pi3HatControlHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn Pi3HatControlHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("Pi3HatControlHardware"), "Successfully cleaned up!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pi3HatControlHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    
    RCLCPP_INFO(rclcpp::get_logger("Pi3HatControlHardware"), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pi3HatControlHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    // Clear Faults
    for (auto& c:controllers){ c->SetStop(); }
    
    RCLCPP_INFO(rclcpp::get_logger("Pi3HatControlHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Pi3HatControlHardware::read(
    const rclcpp::Time &, const rclcpp::Duration & period
)
{
    // Reading is done in the write() method
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type pi3hat_hardware_interface::Pi3HatControlHardware::write(
    const rclcpp::Time &, const rclcpp::Duration &period
)
{
    send_frames.clear();
    receive_frames.clear();


    // Writes Position and Effort Commands to respective controllrs, in here it update position and apply relevant offsets
    for (size_t i = 0 ; i < info_.joints.size(); ++i)
    {
        if (control_modes_[i] == "position" && std::isnan(hw_command_positions_[i]))
        {
            RCLCPP_WARN(rclcpp::get_logger("Pi3HatControlHardware"), 
                        "NaN position command for actuator %zu", i);
            continue;
        }
        else if (control_modes_[i] == "effort" && std::isnan(hw_command_efforts_[i]))
        {
            RCLCPP_WARN(rclcpp::get_logger("Pi3HatControlHardware"), 
                        "NaN effort command for actuator %zu", i);
            continue;
        }
        else if (control_modes_[i] == "velocity" && std::isnan(hw_command_velocities_[i]))
        {
            RCLCPP_WARN(rclcpp::get_logger("Pi3HatControlHardware"), 
                        "NaN velocity command for actuator %zu", i);
            continue;
        }

        double computed_cmd;

        // Sending Comamnds
        mjbots::moteus::PositionMode::Command cmd;

        if (control_modes_[i] == "position")  // Position control
        {
            computed_cmd = (((hw_command_positions_[i] / (2 * M_PI)) * 9.0) + hw_actuator_position_offsets_[i]) * hw_actuator_axis_directions_[i];
            cmd.position = computed_cmd; 
            cmd.velocity = 0.0;
            cmd.velocity_limit = 2.0;
            cmd.accel_limit = 3.0;
        }
        else if (control_modes_[i] == "effort")  // Effort control
        {
            cmd.kp_scale = 0.0f;
            cmd.kd_scale = 0.0f;
            cmd.feedforward_torque = hw_command_efforts_[i] * hw_actuator_axis_directions_[i];
        }
        else if (control_modes_[i] == "velocity") 
        {
            cmd.kp_scale = 0.0f;
            cmd.kd_scale = 1.0f;
            cmd.velocity = hw_command_velocities_[i] * hw_actuator_axis_directions_[i];
        }

        send_frames.push_back(controllers[i]->MakePosition(cmd));
    }

    // Send the frames and receive back the frames reported
    // transport->BlockingCycle(
    //     &send_frames[0], send_frames.size(),
    //     &receive_frames
    // );

    /*MUST PUT THIS HERE!!!!*/
    mjbots::moteus::BlockingCallback cbk;
    
    transport->Cycle(&send_frames[0], send_frames.size(), &receive_frames, &attitude, nullptr, nullptr, cbk.callback());

    cbk.Wait();

    // From Received Frames, it will iterate through each joint, and for each joint, it will determine if the frames matches that of the can id, and if such, parses the results
    for (size_t i = 0 ; i < info_.joints.size(); ++i)
    {
        for (auto it = receive_frames.rbegin(); it != receive_frames.rend(); ++it)
        {
            if (it->source == hw_actuator_can_ids_[i])
            {   
                double p_des;
                double v_des;
                double e_des;

                auto maybe_servo = mjbots::moteus::Query::Parse(it->data, it->size);
                
                const auto& v = maybe_servo;

                if (control_modes_[i] == "position")
                {
                    p_des = ((v.position - hw_actuator_position_offsets_[i])/9.0) * 2 * M_PI * hw_actuator_axis_directions_[i];
                    v_des = (v.velocity)/9 * hw_actuator_axis_directions_[i];
                    e_des = v.torque * hw_actuator_axis_directions_[i];
                }
                else{
                    p_des = -v.position * hw_actuator_axis_directions_[i];
                    v_des = -v.velocity * hw_actuator_axis_directions_[i];
                    e_des = -v.torque * hw_actuator_axis_directions_[i];
                }

                hw_state_positions_[i] = p_des;
                hw_state_velocities_[i] = v_des;
                hw_state_efforts_[i] = e_des;
                if (logging == 1){

                
                        RCLCPP_INFO(
                            rclcpp::get_logger("Pi3HatControlHardware"),
                            "Joint: %s, Mode: %2d, Command: %.3f, Velocity: %.3f, Torque: %.3f, Temperature: %.3f",
                            info_.joints[i].name.c_str(),
                            static_cast<int>(v.mode),
                            hw_state_positions_[i],
                            hw_state_velocities_[i],
                            hw_state_efforts_[i],
                            v.temperature
                        );
                    
                }
            }
        }
    }

    auto a = attitude;

    //Processing Attitude
    hw_state_imu_orientation_[0] = a.attitude.w;      
    hw_state_imu_orientation_[1] = -a.attitude.z;     
    hw_state_imu_orientation_[2] = a.attitude.y;     
    hw_state_imu_orientation_[3] = -a.attitude.x;      

    hw_state_imu_angular_velocity_[0] = a.rate_dps.x * DEG_TO_RAD;  
    hw_state_imu_angular_velocity_[1] = a.rate_dps.y * DEG_TO_RAD;  
    hw_state_imu_angular_velocity_[2] = a.rate_dps.z * DEG_TO_RAD;  

    hw_state_imu_linear_acceleration_[0] = a.accel_mps2.x;  
    hw_state_imu_linear_acceleration_[1] = a.accel_mps2.y;
    hw_state_imu_linear_acceleration_[2] = a.accel_mps2.z; 

    // RCLCPP_INFO(
    //     rclcpp::get_logger("Pi3HatControlHardware"),
    //     "Attitude x:%.3f, Attitude y: %.3f, Attitude z:%.3f, Attitude w:%.3f",
    //     attitude.attitude.x, attitude.attitude.y, attitude.attitude.z, attitude.attitude.w
    // );

    return hardware_interface::return_type::OK;
}

} //pi3hat_hardware_interface


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    pi3hat_hardware_interface::Pi3HatControlHardware, hardware_interface::SystemInterface
)