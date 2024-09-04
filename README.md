# Simple Robotic Arm in ROS

## Overview

This project is developed as part of the Robotics Lab course assignment, focusing on simulating a robotic arm using ROS and Gazebo.
The project involves creating the robot's description, visualizing it in Rviz, adding transmission and controllers, and implementing
a camera sensor. Additionally, a ROS publisher node is developed to control the robot's joints by sending position commands.

## Objectives

The primary objectives of this project are:
1. **Robot Description and Visualization**: Create the URDF description of a robotic arm and visualize it in Rviz.
2. **Gazebo Simulation Setup**: Spawn the robot in a Gazebo environment and add necessary controllers.
3. **Sensor Integration**: Add a camera sensor to the robot for visual perception tasks.
4. **Control Implementation**: Develop a ROS node to read joint states and send position commands to control the robot.

## Project Structure

The project is organized into several components, each corresponding to different stages of development and testing. The primary components include:

1. **Robot Description and Visualization**:
    - The robotic arm's URDF is defined, and a launch file is created to visualize it in Rviz.
    - Collision meshes are replaced with primitive shapes for better performance, and a configuration file is saved to automatically load the robot model in Rviz.
  
<p align="center">
  <img src="https://github.com/user-attachments/assets/4e1390ad-d564-4294-ad82-e94da16edb2d" alt="Pseudocode" width="400"/>
</p>


2. **Gazebo Simulation Setup**:
    - A Gazebo simulation environment is set up with the robotic arm loaded into the world.
    - Transmission and position controllers are added to the robot, allowing for joint-level control in the simulation.

<p align="center">
  <img src="https://github.com/user-attachments/assets/bb8ca3e4-4c7b-43b5-b2e7-9a4cda0541ad" alt="Pseudocode" width="400"/>
</p>

3. **Sensor Integration**:
    - A camera sensor is added to the robot using a fixed joint. The camera is configured to publish image topics, which are then visualized using `rqt_image_view`.

<p align="center">
  <img src="https://github.com/user-attachments/assets/aa24bd7c-fffd-45cc-b6df-126815c84877" alt="Pseudocode" width="400"/>
</p>

4. **Control Implementation**:
    - A ROS C++ node is developed to subscribe to the `joint_states` topic, print current joint positions, and publish position commands to control the robot's joints.


<p align="center">
  <img src="https://github.com/user-attachments/assets/a1b2d0c0-1504-4f11-a674-ae15da542567" alt="Pseudocode" width="400"/>
</p>

## Installation and Setup

To set up the project and run the simulation, follow these steps:

1. **Install ROS Noetic**:
    Ensure that ROS Noetic is installed on your system.

2. **Clone the Repository**:
    ```bash
    cd .../catkin_ws/src
    git clone https://github.com/EmanueleCuzzocrea/Homework1.git
    ```

3. **Build the Package**:
    Compile the ROS workspace:
    ```bash
    cd .../catkin_ws
    catkin_make
    source devel/setup.bash
    ```

## Key Results and Observations

- **Robot Visualization**: The robotic arm was successfully visualized in Rviz, with accurate collision models and configurations.
- **Gazebo Simulation**: The robot was loaded into the Gazebo environment, with all controllers functioning correctly. The robot's joints responded accurately to position commands.
- **Camera Integration**: The camera sensor was successfully added to the robot, with image topics correctly published and visualized in `rqt_image_view`.
- **Joint Control**: The developed ROS node effectively controlled the robot's joints by publishing position commands, demonstrating the ability to manipulate the robotic arm within the simulation.

## Conclusion

This project demonstrates the successful simulation and control of a robotic arm in a ROS and Gazebo environment.
The integration of sensors and the implementation of control nodes highlight the flexibility and capabilities of ROS for robotic simulation tasks.
