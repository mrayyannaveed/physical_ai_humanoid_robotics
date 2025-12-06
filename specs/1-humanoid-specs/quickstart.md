# Quickstart Guide: Humanoid Robot Simulation

**Branch**: `1-humanoid-specs` | **Date**: 2025-12-03 | **Plan**: [plan.md](./plan.md)

This guide provides the basic steps to set up the simulation environment and run a simple test.

## 1. Prerequisites

-   NVIDIA GPU with driver version 525.60.11 or later.
-   Ubuntu 22.04
-   Docker Engine and NVIDIA Container Toolkit.
-   A local clone of this repository on the `1-humanoid-specs` branch.

## 2. Environment Setup

1.  **Install Isaac Sim**: Follow the official NVIDIA documentation to install Isaac Sim 2023.1 or later. Ensure it is installed and running correctly.
2.  **Install ROS 2 Humble**: Follow the official ROS 2 documentation to install the 'ros-humble-desktop' package.
3.  **Build the Workspace**:
    ```bash
    cd /path/to/your/workspace
    colcon build
    source install/setup.bash
    ```

## 3. Running the Simulation

1.  **Launch Isaac Sim**: Start the Isaac Sim application.
2.  **Load the Robot**: Open the humanoid robot's USD file, located in `src/simulation/assets/`.
3.  **Launch the ROS 2 Nodes**:
    ```bash
    # In a new terminal, source the workspace
    source install/setup.bash
    # Launch the simulation bridge and control nodes
    ros2 launch simulation_bringup bringup.launch.py
    ```

## 4. Verifying the Setup

-   You should see the robot model loaded in the Isaac Sim viewport.
-   Running `ros2 topic list` in a new terminal should show the various sensor and control topics (e.g., `/imu/data`, `/joint_states`).
-   You can send a simple command to test the robot's response:
    ```bash
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" -1
    ```
    The robot should exhibit a small forward motion in the simulation.

## 5. Frontend Application Setup

This section outlines the steps to set up and run the web-based frontend application.

### Prerequisites

- Node.js (LTS version)
- npm (Node Package Manager)

### Setup and Run

1.  **Navigate to the frontend directory**:
    ```bash
    cd book
    ```
2.  **Install dependencies**:
    ```bash
    npm install
    ```
3.  **Start the development server**:
    ```bash
    npm start
    ```
    The frontend application should now be accessible in your web browser, typically at `http://localhost:3000`.

## 6. Verifying Frontend Setup

-   Open your web browser and navigate to the address provided by `npm start`.
-   You should see the Docusaurus homepage.
-   Attempt to navigate to a login/signup page (if implemented) to confirm basic routing.
