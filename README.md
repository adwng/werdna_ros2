# WERDNA
Wheeled Bipedal. These are the ROS2 implementations to be run on a Raspberry Pi 4 with *Ubuntu Mate* and *ROS2 Humble*. For the custom gym environments can be found [here](https://github.com/adwng/werdna_genesis).

The RPI4 is also intended to be connected to a pi3hat from mjbots which will be commanding the Moteus Drivers to actuate accordingly. 

<details>
  <summary>Dependencies</summary>

  1. `ROS2 Control`
  2. `ROS2 Controllers` 
  3. `Torch`
  4. `Moteus`
  5. `pi3hat`
   
</details>


## Package Contents:
### Main Package Contents
|_Packages_|_Functionality_|
| ------------- | ------------- |
|`werdna_bringup`|Bringup Commands|
|`werdna_description`|Description of URDF Models|
|`werdna_msgs`|custom messages for werdna teleoperation|
|`pi3hat_hardware_interface`|custom Hardware Interface from RPI->Pi3Hat|
|`werdna_teleop`|Teleoperation Commands for Joystick|
|`werdna_agent`|Node to run trained agent inference|
|`werdna_odometry_broadcaster`|Broadcaster for Odometry|

## Code Run
**Launch in Base**
|`CAPTCHAS`|`Explanation`|
|----------|-----------|
|***Super User Mode***|Ensure Super User Mode, Since the pi3hat requires root access to use the GPIO pins|
|***UDP Memory Switch***|Force FastDDS to use UDP instead of shared memory|
|***Watchdog Timer***|Increase the watchdog timer for each servo to at least 0.5s to avoid mode 11 timeout bug|

`Launch Controllers`
```
export FASTRTPS_DEFAULT_PROFILES=/home/andrew/werdna_ws/src/werdna_ros2/pi3hat_hardware_interface/fastrtps_profile_no_shmem.xml
sudo -E /home/andrew/runasroot.sh ros2 launch werdna_bringup launch_robot.py
```

"Launch this for mapping and save new map"
```
export FASTRTPS_DEFAULT_PROFILES=/home/andrew/werdna_ws/src/werdna_ros2/pi3hat_hardware_interface/fastrtps_profile_no_shmem.xml
sudo -E /home/andrew/runasroot.sh ros2 launch werdna_bringup mapping_slam.launch.py
```

For Localization and Path Planning
```
export FASTRTPS_DEFAULT_PROFILES=/home/andrew/werdna_ws/src/werdna_ros2/pi3hat_hardware_interface/fastrtps_profile_no_shmem.xml
sudo -E /home/andrew/runasroot.sh ros2 launch werdna_bringup localization.py map:=$HOME/werdna_ws/map.yaml
```


It should should launch the description, relevant controllers, rosboard and hardware interface.

> [!NOTE]
> If wish to view ROS2 topics and also enable your other programs to publish/subscribe to it. Ensure the session is enabled the same way as the script above (entering superuser mode->exporting profiles->sourcing relevant environments).
> `sudo -E /home/andrew/runasroot.sh rviz2`
> Of course, it can be launched all together with the same launch file

## Features
- [x] **BringUp Actions**: Launches the Controllers, Hardware Interface, Teleoperation Node for Joysticks, and the trained agent's inference node at once
- [x] **Description**: Contains URDF for the Werdna Robot and linked it to the hardware interface
- [x] **Hardware Interface**: Overlays the Pi3Hat to communicate with moteus controllers via ROS2 Control
- [x] **Messages**: Custom Messages for the robot
- [x] **Odometry Broadcaster**: Uses IMU and Joints to Compute Odometry, IMU, and TF.
- [x] **Teleop**: Maps Joystick Interface to the custom messages
- [x] **PID**: Run inferences

## PI3HAT HARWARE INTERFACE DETAILS
### Command Interfaces
- Position
- Torque
- Torqe

Selected based on *control mode* in the urdf file, it requires manual adjustment on the acceleration and velocity limits. 

### State Interfaces
- Position 
- Velocity
- Effort
- Quarternion Orientation
- Angular Velocities
- Linear Acceleration

**Example Xacro File for hardware interface set up**
```
<ros2_control name="pi3hat_hardware_interface" type="system">
    <hardware>
        <plugin>pi3hat_hardware_interface/Pi3HatHardwareInterface</plugin>
        <param name="imu_mounting_deg.yaw">0</param>
        <param name="imu_mounting_deg.pitch">0</param>
        <param name="imu_mounting_deg.roll">0</param>
        <param name="logging">0</param>
    </hardware>

    <joint name="joint_1">
        <param name="can_channel">1</param>
        <param name="can_id">1</param>

        <param name="position_offset">0.0</param>
        <param name="control_mode">position</param>

        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
    </joint>

    <sensor name="imu_sensor">
        <state_interface name="orientation.x"/>
        <state_interface name="orientation.y"/>
        <state_interface name="orientation.z"/>
        <state_interface name="orientation.w"/>
        <state_interface name="angular_velocity.x"/>
        <state_interface name="angular_velocity.y"/>
        <state_interface name="angular_velocity.z"/>
        <state_interface name="linear_acceleration.x"/>
        <state_interface name="linear_acceleration.y"/>
        <state_interface name="linear_acceleration.z"/>
    </sensor>
</ros2_control>
```

Ensure its at default position where all joints report `0`.




