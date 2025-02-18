# WERDNA
Wheeled Bipedal Trained by RL for Locomotion and Posture Control. These are the ROS2 implementations to be run on a Raspberry Pi 4 with *Ubuntu Mate* and *ROS2 Humble*. For the custom gym environments can be found [here](https://github.com/adwng/werdna_gym/tree/Advanced).

The RPI4 is also intended to be connected to a pi3hat from mjbots which will be commanding the Moteus Drivers to actuate accordingly. 

> [!NOTE] 
> It is a Final Year Thesis Project and it is still not finalized.
> 
> Hasn't been tested on real hardware.
>
> CAD will be published shortly.

> [!IMPORTANT]
> I added a Werdna Odometry broadcaster as an alternative to the werdna odometry node to determine if it works the same overall. Hopefully this broadcaster works so i can just yolo the odometry node overall.
> Will be switching the agent to be implemented using onxx to avoid issues with real time control loop when inferencing with torch
> Rewriting the hardware interface to use the c++ bindings with the moteus c++ libraries.
<details>
  <summary>Dependencies</summary>

  1. `ROS2 Control`
  2. `ROS2 Controllers` 
  3. `Stable Baselines3`
  4. `Gymnasium`
  5. Moteus C++ Library
   
</details>


## Package Contents:
### Main Package Contents
|_Packages_|_Functionality_|
| ------------- | ------------- |
|`werdna_bringup`|Bringup Commands|
|`werdna_description`|Description of URDF Models|
|`werdna_msgs`|custom messages for werdna teleoperation|
|`werdna_odometry`|Publish Odometry from wheels joint information|
|`werdna_hardware_interface`|custom Hardware Interface from RPI->Teensy|
|`werdna_teleop`|Teleoperation Commands for Joystick|
|`werdna_agent`|Node to run trained agent inference|
|`werdna_odometry_broadcaster`|Broadcaster for Odometry|

### Raspberry Pi Required Contents
|_Packages_|_Functionality_|
| -------- | ------------- |
|[MPU6050 Driver](https://github.com/kimsniper/ros2_mpu6050)|Driver Node to interface MPU6050 with RPI4|

## Code Run
**Launch in Base**
```
cd <your_ws>
source install/setup.zsh #or setup.bash (I use zsh anyways)
ros2 launch bringup launch_robot.py
```

It should should launch the description, relevant controllers and hardware interface, teleoperation (joystick), and finally the trained agent at once

## TODO
- [ ] Write pi3hat hardware interface instead of using esp32 
- [ ] Rewrite agent node to use onxxruntime
- [x] Rewrite messages and teleop nodes to accomodate for new observation spaces.
- [x] Update URDF as wheel

## Features
- [x] **BringUp Actions**: Launches the Controllers, Hardware Interface, Teleoperation Node for Joysticks, and the trained agent's inference node at once
- [x] **Description**: Contains URDF for the Werdna Robot and linked it to the hardware interface
- [x] **Hardware Interface**: Connected to a Teensy Controller and communicates via Serial Command
- [x] **Messages**: Custom Messages for the robot
- [x] **Odometry Broadcaster**: A copy from diff drive controller source code with only the odometry contents, meant to publish odometry from wheel positions
- [x] **Teleop**: Maps Joystick Interface to the custom messages
