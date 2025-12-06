# Data Models for Humanoid Robot

**Branch**: `1-humanoid-specs` | **Date**: 2025-12-03 | **Plan**: [plan.md](./plan.md)

This document defines the key data entities and their relationships, derived from the feature specification. These will primarily be represented as ROS 2 messages and services.

## 1. HumanoidRobot

Represents the overall state of the robot.

-   **Fields**:
    -   `state` (string): The current high-level state (e.g., "WALKING", "IDLE", "EMERGENCY_STOP").
    -   `joint_states` (sensor_msgs/JointState): An array containing the position, velocity, and effort of each joint.
    -   `odometry` (nav_msgs/Odometry): The estimated position and orientation of the robot in the world.
-   **Relationships**: Composed of a SensorSuite, ComputeModule, and ControlSystem.

## 2. SensorSuite

Represents the collection of all sensor data. This will be implemented as a set of ROS 2 topics, each publishing data from a specific sensor.

-   **Key Data Streams (ROS 2 Topics)**:
    -   `/camera/rgb/image_raw` (sensor_msgs/Image): Raw RGB camera feed.
    -   `/camera/depth/image_raw` (sensor_msgs/Image): Raw depth camera feed (optional).
    -   `/imu/data` (sensor_msgs/Imu): Data from the Inertial Measurement Unit.
    -   `/contact_sensors` (custom_msgs/ContactState): Data from tactile/contact sensors (optional).
    -   `/torque_feedback` (custom_msgs/TorqueArray): Torque readings from joints.

## 3. ComputeModule

Represents the state and performance of the on-board computers.

-   **Fields**:
    -   `cpu_usage` (float): Current CPU utilization percentage.
    -   `memory_usage` (float): Current memory utilization percentage.
    -   `accelerator_usage` (float): Current AI accelerator utilization.
    -   `temperature` (float): Temperature of the compute module in Celsius.

## 4. ControlSystem

Represents the commands sent to the robot's actuators.

-   **Key Action/Service Interfaces**:
    -   `/follow_trajectory` (Action): An action to make the robot follow a specified path.
    -   `/emergency_stop` (Service): A service to immediately halt all robot motion.
    -   `/set_control_mode` (Service): A service to switch between different control modes (e.g., position control, torque control).

## 5. CommunicationStack

This is not a data model itself, but the ROS 2 middleware that defines the structure for all data exchange. The contracts for this stack will be defined as `.msg`, `.srv`, and `.action` files in the `contracts/` directory.
