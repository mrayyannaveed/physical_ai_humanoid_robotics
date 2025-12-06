# Research & Decisions for Humanoid Robot Plan

**Branch**: `1-humanoid-specs` | **Date**: 2025-12-03 | **Plan**: [plan.md](./plan.md)

This document records the decisions made to resolve ambiguities during the planning phase.

## 1. Deep Learning Framework

-   **Decision**: PyTorch
-   **Rationale**: PyTorch is selected for its widespread adoption in the robotics research community, its imperative programming style (which simplifies debugging), and the extensive ecosystem of libraries for reinforcement learning and computer vision. It integrates well with Python, which is a primary language for high-level ROS 2 nodes.
-   **Alternatives considered**:
    -   **TensorFlow**: A viable alternative, but its declarative nature (graph-based) can be more complex to debug for robotics applications.
    -   **JAX**: Offers high-performance computation but has a smaller community and fewer robotics-specific libraries compared to PyTorch.

## 2. Physics Engine for Simulation

-   **Decision**: NVIDIA PhysX
-   **Rationale**: The plan specifies NVIDIA Isaac Sim as the training simulation reference. Isaac Sim is built on the NVIDIA Omniverse platform, which uses PhysX as its real-time physics engine. Therefore, using PhysX is the native and most compatible choice, ensuring access to the latest features and performance optimizations within the chosen simulation environment.
-   **Alternatives considered**:
    -   **MuJoCo**: A fast and popular physics simulator for robotics, but integrating it with Isaac Sim would be non-standard and complex.
    -   **Bullet**: An open-source physics engine, but it lacks the tight integration and support that PhysX has within the NVIDIA ecosystem.
