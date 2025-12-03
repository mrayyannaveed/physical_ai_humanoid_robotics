---
sidebar_position: 3
---

# Actuators and Sensors

The physical capabilities of a humanoid robot are fundamentally defined by its actuators (the "muscles") and its sensors (the "senses"). Recent advancements have pushed the boundaries of both, enabling more dynamic, dexterous, and perceptive robots.

## Actuator Technologies

Actuators are responsible for generating movement. Modern humanoid robots demand actuators that are powerful, compact, energy-efficient, and capable of precise control.

### 1. Integrated Joint Modules
The trend is towards highly integrated joint modules that combine the motor, gearbox, encoder, and often the motor driver and internal sensors into a single compact unit. This design philosophy is critical for meeting the strict anthropomorphic requirements of humanoids, allowing them to operate in human-centric spaces.

**Key characteristics:**
-   **High Torque-to-Weight Ratio**: Essential for dynamic motions and lifting capabilities.
-   **Backdrivability**: The ability for the joint to be moved by external forces, which is important for compliant interaction and safety.
-   **Low Inertia**: Enables rapid and agile movements.

### 2. Types of Actuators
-   **Electric Motors**: The dominant choice for humanoids due to their high efficiency, precise control, and ability to integrate with digital systems.
    -   **Brushless DC (BLDC) Motors**: Offer high power density and efficiency.
    -   **Quasi-Direct Drive (QDD) Actuators**: Minimize or eliminate reduction gears to improve backdrivability and reduce friction, enhancing force control.
    -   **Series Elastic Actuators (SEA)**: Incorporate a series spring to measure and control output force, improving compliance and interaction safety.

-   **Hydraulic Actuators**: Historically used in highly dynamic robots (like early Atlas versions) for their immense power density. However, they are complex, noisy, and require significant plumbing, leading to a shift towards all-electric designs.

## Sensor Technologies

Sensors provide the robot with data about its own state (proprioception) and its environment (exteroception). A comprehensive sensor suite is vital for robust control, navigation, and interaction.

### 1. Proprioceptive Sensors (Internal State)
These sensors provide feedback on the robot's internal state, crucial for self-awareness, balance, and precise movement.
-   **Encoders**: Measure joint angles and velocities, typically integrated directly into the actuator.
-   **Inertial Measurement Units (IMUs)**: Combine accelerometers and gyroscopes to measure orientation, angular velocity, and linear acceleration, essential for balance and whole-body control.
-   **Force/Torque Sensors**: Located at joints or end-effectors, they measure interaction forces, enabling compliant control and dexterous manipulation. This includes the **torque feedback sensors** mentioned in our feature specification.

### 2. Exteroceptive Sensors (External Environment)
These sensors enable the robot to perceive its surroundings, mimicking human senses.
-   **Vision Systems**:
    -   **RGB Cameras**: For general visual perception, object recognition, and human-robot interaction. Multiple cameras can provide stereoscopic vision for depth perception.
    -   **Depth Cameras**: (e.g., LiDAR, Time-of-Flight, Structured Light) provide 3D spatial information, crucial for obstacle avoidance, mapping, and object manipulation.
    -   **Event Cameras**: Capture changes in pixel intensity asynchronously, offering very high temporal resolution for fast-moving scenes or objects.

-   **Tactile Systems**:
    -   **Touch/Pressure Sensors**: Arrays of pressure-sensitive materials on fingertips or body surfaces, providing information about contact, grip force, and object texture.
    -   **Proximity Sensors**: Detect nearby objects without physical contact, useful for collision avoidance in tight spaces.

-   **Audio Systems**: Microphones for sound localization and speech recognition, enhancing human-robot communication.

## Sensor Integration and Fusion

The true power of a robot's perception lies in its ability to integrate and interpret data from multiple, diverse sensors.
-   **Sensor Fusion Algorithms**: Techniques like **Kalman Filters** and **Particle Filters** combine noisy and uncertain sensor data to create a more robust and accurate estimate of the robot's state and environment.
-   **Machine Learning**: Deep learning models process raw sensor streams (e.g., camera images, LiDAR point clouds) to extract high-level semantic information (e.g., "chair", "person", "door").

The confluence of these advanced actuator and sensor technologies, coupled with sophisticated AI algorithms, is propelling humanoid robots towards unprecedented levels of autonomy and interaction capability in 2025 and beyond.

<!--
Cross-linking suggestion:
- Link to FR-004, FR-006, FR-007, FR-011 in `specs/1-humanoid-specs/spec.md` to show specific sensor requirements.
- Link to the `hardware` ROS 2 package for where the sensor interface nodes would reside.
- Link to "Chapter 3: Sensor Fusion Techniques" for a deeper dive into data integration.
-->
