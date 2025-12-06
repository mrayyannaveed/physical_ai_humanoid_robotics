# ROS 2 Message and Service Definitions

This directory will contain the custom ROS 2 interface definitions (`.msg`, `.srv`, `.action` files) used for communication between the humanoid robot's software modules.

## Example (custom_msgs/ContactState.msg)

```
# Represents the state of a single contact sensor
std_msgs/Header header
string frame_id
bool is_in_contact
float64 contact_force
```

## Example (custom_srvs/GetTrajectory.srv)

```
# Request
geometry_msgs/Pose start_pose
geometry_msgs/Pose end_pose
---
# Response
trajectory_msgs/JointTrajectory trajectory
bool success
```
