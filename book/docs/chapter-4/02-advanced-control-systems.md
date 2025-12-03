---
sidebar_position: 2
---

# Advanced Control Systems

Control systems are the heart of a humanoid robot, translating high-level plans into low-level joint commands and ensuring stable, precise, and compliant execution. Given the complex, high-dimensional, and often redundant nature of humanoids, advanced control strategies are essential.

## 1. Whole-Body Control (WBC)

Whole-body control aims to coordinate the movements of all the robot's joints and its base (torso) simultaneously to achieve multiple tasks while respecting various constraints.
-   **Task Hierarchy**: WBC often formulates control as an optimization problem where tasks are prioritized. For example, maintaining balance might be a higher priority than achieving a precise end-effector pose.
-   **Quadratic Programming (QP)**: A common mathematical framework for solving WBC problems, allowing multiple tasks and inequality constraints (e.g., joint limits, contact forces) to be handled efficiently.
-   **Operational Space Control**: Controls the robot's end-effectors directly in Cartesian space, simplifying intuitive task specification. WBC extends this to manage the entire body.

## 2. Model Predictive Control (MPC)

As introduced in motion planning, MPC is also a powerful tool for low-level dynamic control of humanoids.
-   **Receding Horizon Control**: At each time step, MPC optimizes a sequence of control inputs over a future prediction horizon. Only the first control input is applied, and the process is repeated.
-   **Constraint Handling**: MPC naturally handles complex constraints, such as joint limits, velocity limits, and contact force limits.
-   **Non-linear Dynamics**: Can handle the highly non-linear dynamics of humanoids, making it suitable for aggressive and agile maneuvers.
-   **Reactive Control**: The continuous re-planning makes MPC inherently reactive to disturbances and changes in the environment.

## 3. Compliance and Impedance Control

Humanoid robots need to interact safely and robustly with unstructured environments and humans. This requires compliant behavior, where the robot can "give way" or adapt its motion in response to external forces.
-   **Compliance Control**: Directly controls the robot's stiffness and damping properties, allowing it to yield to external forces.
-   **Impedance Control**: Regulates the relationship between the robot's position/velocity and the interaction forces it exerts or experiences. This allows the robot to behave like a spring-damper system, adapting to uncertainties in its environment.

## 4. Disturbance Rejection and Balance

Maintaining balance is a paramount challenge for bipedal robots.
-   **Zero Moment Point (ZMP) Control**: A classical approach that tracks a desired ZMP trajectory to generate stable walking patterns.
-   **Centroidal Momentum Control**: A more advanced technique that uses the robot's total angular momentum about its Center of Mass (CoM) to maintain dynamic balance during highly agile motions, including jumping and running.
-   **Feedback from Proprioceptive Sensors**: IMUs and force/torque sensors are critical for detecting disturbances and enabling rapid balance recovery. (See `src/control/balance_recovery.py` in our project for a placeholder).

<!--
Cross-linking suggestion:
- Link to the `control` ROS 2 package in `src/control/` where these controllers would be implemented.
- Link to FR-009 in `specs/1-humanoid-specs/spec.md` regarding closed-loop control consistency.
- Link to "Chapter 2: Kinematics and Dynamics" for the theoretical background.
-->
