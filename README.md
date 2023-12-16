# mae263a
Kinematics of Robotic Systems Fall 2023: Design and Control of a Robotic Manipulator

## Project Description
This repository provides an interface to the [OpenNanipulator](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/#overview) API, where we define our own custom manipulator arm **DoodleBot**. This interface allows us to simulate and operate the arm using Dynamixel [MX-28 (2.0) motors](https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/).

## Assembly
  1. Follow [Dynamixel MX-28AR Servo Motor Instruction](doc/MX-28AR%20Instruction.pdf) for instructions on setting a unique ID for each servo motor using [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/). IDs must be set to `[11,12,13,14,15]` for motors `[1,2,3,4,5]`, where motor 5 acts as a dummy motor.
  2. Within Dynamixel Wizard 2.0, ensure each motor is set to 180.0 degrees
  3. Print the parts included in `/meshes` and assemble **DoodleBot** using the assembly kit included with this project. For the end effector's [compliant mechanism](meshes/chain_link5.stl), you will need glue for assembly.

## System
Our system has been tested on Ubuntu 20.04 with ROS Noetic. ROS Noetic and accompanying build systems can be installed via [Section 4.1.2](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/#install-ros-on-pc) of the OpenNanipulator emanuel.

## Dependencies
Dependencies can be installed following [Section 4.1.3](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/#install-ros-packages) of the OpenNanipulator emanuel. Note that when installing the base open_manipulator [repository](https://github.com/ROBOTIS-GIT/open_manipulator), use our custom fork instead:
```bash
cd ~/catkin_ws/src/
git clone git@github.com:adthoms/open_manipulator.git
```
Other dependencies required can be found through Sections 5-8 of the OpenNanipulator emanual and are listed for convenience:
```bash
sudo apt-get install ros-noetic-joint-state-publisher-gui
cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/open_manipulator_controls.git
```

## Installation and Build

To install and build the **DoodleBot** interface, enter:
```bash
cd ~/catkin_ws/src/
git clone git@github.com:adthoms/mae263a.git
cd ~/catkin_ws && catkin build
```

## Running

For simulation and demo videos, we have used OpenManipulator's Control GUI modified to suite **DoodleBot**. The sequence of actions taken by **DoodleBot** follows:
  1. Click `Home Pose`
  2. Click `Init Pose`
  3. Within the `Drawing` Tab, Select `Heart` and enter the following parameters and click `Send`
     1. Radius: 0.050 m
     2. Revolution: 1.000 rev
     3. Start Angle: 0.000 rad
     4. time 10.000 s
  4. Once action 3 is complete, click `Home Pose`

### Simulation
To simulate **DoodleBot**, enter the following commands in three separate terminals:
```bash
roslaunch mae263a doodlebot_gazebo.launch # launch gazebo simulation
roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false # enable control
roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch # control gui
```
Press the `run` bottom at the bottom left corner of the gazebo window, and click `Timer Start` in the OpenManipulator control GUI. To simulate **DoodleBot** in Matlab as a proof of concept for the inverse kinematics calculated in the final report, run `main.m` in `/tooling`.

### Hardware
Prior to launching hardware control, position **DoodleBot** in a configuration similar to the one shown in [Section 5.1](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_controller_package/#launch-controller) of the OpenNanipulator emanual. Then, enter the following commands in three separate terminals:
```bash
roslaunch open_manipulator_controller open_manipulator_controller.launch baud_rate:=57600 platform:=doodlebot # enable control
roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch # control gui
roslaunch mae263a doodlebot_rviz.launch # visualize real-to-sim
```
Click `Timer Start` in the OpenManipulator control GUI.
