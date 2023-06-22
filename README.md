# Rover URDF File for ROS

This repository contains the URDF (Unified Robot Description Format) file for the "rover" robot, which can be used in ROS (Robot Operating System) applications. The URDF file describes the robot's physical structure and its joints.
## Robot Description

The "rover" robot consists of several components:

    base_link: This link represents the base of the robot. It has a box-shaped visual and collision geometry with specified dimensions. The link is connected to the base_footprint link through a fixed joint.

    wheel_rear_right_link, wheel_rear_left_link, wheel_front_right_link, and wheel_front_left_link: These links represent the four wheels of the rover. Each wheel link has a cylinder-shaped visual and collision geometry with specified dimensions. The links are connected to the base_link through continuous joints.

    camera_link and imu_link: These links represent a camera and an IMU (Inertial Measurement Unit) mounted on the rover, respectively. Both links have box-shaped visual and collision geometries with specified dimensions. The links are connected to the base_link through fixed joints.

    hokuyu_link: This link represents a Hokuyo LiDAR sensor mounted on the rover. It has a cylinder-shaped visual and collision geometry with specified dimensions. The link is connected to the base_link through a fixed joint.

## Joints

The rover robot has several joints that connect its links:

    base_joint: This fixed joint connects the base_footprint link to the base_link link. It has no axis of rotation and is located at the origin of the base link.

    wheel_rear_right_joint, wheel_rear_left_joint, wheel_front_right_joint, and wheel_front_left_joint: These continuous joints connect the wheel links to the base_link link. They have a rotation axis along the z-axis and are located at specific positions relative to the base link.

    camera_joint: This fixed joint connects the base_link link to the camera_link link. It has a rotation axis along the y-axis and is located at a specific position relative to the base link.

    imu_joint: This fixed joint connects the base_link link to the imu_link link. It has a rotation axis along the y-axis and is located at a specific position relative to the base link.

    hokuyu_joint: This fixed joint connects the base_link link to the hokuyu_link link. It has a rotation axis along the y-axis and is located at a specific position relative to the base link.

## Gazebo Plugins

The URDF file includes Gazebo plugins for simulating the camera and controlling the robot:

    camera_controller: This plugin simulates a camera sensor mounted on the rover. It provides image and camera information topics for perception tasks. The plugin is configured with specific parameters such as the image size, noise characteristics, and camera frame name.

    skid_steer_drive_controller: This plugin simulates a skid steer drive controller for the rover. It controls the movement of the robot's wheels based on desired velocities. The plugin is configured with parameters such as the update rate and joint names.

## Usage

To use the URDF file in your ROS project, follow these steps:

    Clone this repository to your local machine.
    Copy the rover.urdf file to your ROS package's urdf directory.
    Modify the URDF file as needed to adapt it to your specific robot configuration.
    Launch the robot simulation in Gazebo using the URDF file.
