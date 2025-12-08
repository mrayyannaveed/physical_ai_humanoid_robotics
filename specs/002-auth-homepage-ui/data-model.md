# Data Model for User Background

This document outlines the data model for collecting user's software and hardware background during the signup process.

## Entity: UserProfile (Extension)

The existing User Profile entity will be extended to include a field for user background.

### Attributes:

-   **`user_id`**: (Foreign Key to existing User entity) Unique identifier for the user.
-   **`background_info`**: (Text/String) A free-form text field to capture the user's software and hardware background. This can include details about their experience with programming languages, frameworks, robotics platforms (e.g., ROS, NVIDIA Isaac Sim), and hardware (e.g., specific robot models, sensors).

### Example Data:

```json
{
  "user_id": "uuid-1234-abcd",
  "background_info": "Experienced with Python, C++, React. Familiar with ROS 2 Humble, NVIDIA Isaac Sim, and PyTorch for robotics development. Built several custom drones and robotic arms."
}
```