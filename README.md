# WERDNA
Wheeled Bipedal. These are the ROS2 implementations to be run on a Raspberry Pi 4 with *Ubuntu Mate* and *ROS2 Humble*. 

The RPI4 is also intended to be connected to a pi3hat from mjbots which will be commanding the Moteus Drivers to actuate accordingly. 

<details>
  <summary>Dependencies</summary>

  1. `ROS2 Control`
  2. `ROS2 Controllers` 
  3. `Moteus`
   
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

### Additioanl Packages
|_Packages_|_Functionality_|
| ------------- | ------------- |
|`sllidar_ros2`|Driver for Lidar|
|`rosboard`|ROSBOARD Build - has custom visualizations for URDF and IMU|

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
It should should launch the description, relevant controllers, rosboard and hardware interface.

"Launch this for mapping and save new map"
```
export FASTRTPS_DEFAULT_PROFILES=/home/andrew/werdna_ws/src/werdna_ros2/pi3hat_hardware_interface/fastrtps_profile_no_shmem.xml
sudo -E /home/andrew/runasroot.sh ros2 launch werdna_bringup mapping_slam.launch.py
```

For Localization and Path Planning
```
export FASTRTPS_DEFAULT_PROFILES=/home/andrew/werdna_ws/src/werdna_ros2/pi3hat_hardware_interface/fastrtps_profile_no_shmem.xml
sudo -E /home/andrew/runasroot.sh ros2 launch werdna_bringup localization.py
```


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

## Adjusting PID Params
PID Params can be edited via the [parameter file](pid.yaml) to avoid rebuilding when testing new PID values. Nonetheless, it is assumed that users familiar with ROS2 and Linux should know when to build to implement new features.

## Bugs
Starting Either SLAM or Nav2 causes a system delay for the PID controller, meaning it will disrupt the loop frequency of the PID controller 
- Recommend to write the PID controller as a custom ROS2 Control Controller to see if the overhead from the SLAM and Nav2 stack will continue to effect it.
- Otherwise, try to find methods to ensure the PID controller is not disrupted when launching other packages.

## Improvements
1. Recommend to rewrite Pi3Hat Interface to support manipulation in KP, KD, and KP values like the one written by Gabriel Levine's Pi3Hat Interface. I suggest joining the moteus discord to discuss matters.
2. Cooling, feel free to add more fans or ventilation to the system as the motors tend to heat up easily if without sufficient cooling, if heated up too much it will lose massive amounts of torque due to heat losses.
3. Power Distribution System - I feel that the implemented power distribution system is quite janky, so feel free to modify or improve the dist system to mitigate chances of magic smoke.
4. Soldering, the soldering done on the wheel motors to drivers is not perfect, please improve the connection method, i.e. using 3 pin XT60 connectors are perfect.

## Additional Materials
It is recommended to fully familiarize themselves with the Moteus Ecosystem and code architecture such as Tview, C++ bindings, and FOC limits.
Additionally, It is also recommended to familiarize themselves with Linus Systems to better understand debugging errors. The same can be said with ROS2 as well. This helps predict future errors encountered such as race conditions, looping frequencies, differences between nodes and ROS2 controllers/hardware interfaces.

## Mechanical Structure
The mechanical design is free to be redesigned as the current design is a proof of concept in using a 4 bar linkage system to control the height of the WBR. However, there are limitations to it inherently. To approach this, feel free to design a true 3 dof leg that can move front to back and up and down. Otherwise, maintaining the original design is left to optimize the mechanical stability of the system (adding bracing for instance) to avoid constant breakage.

