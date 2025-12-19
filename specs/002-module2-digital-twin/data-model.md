# Data Model: Digital Twin Module

This document defines the key data entities for the Digital Twin module. As this is a documentation and simulation-focused feature, the "data model" refers to the conceptual entities and their representations in configuration files and simulation, rather than a database schema.

## 1. Humanoid Robot Model

-   **Representation**: URDF (Unified Robot Description Format) file.
-   **Key Attributes**:
    -   `links`: The rigid bodies of the robot.
    -   `joints`: The connections between links, defining their motion.
    -   `visual`: The 3D mesh and material for rendering the link in viewers and simulators.
    -   `collision`: The geometry used for physics calculations.
    -   `inertial`: The mass and inertia properties of each link.
-   **Relationships**:
    -   Contains one or more `Sensor` entities attached to its links.

## 2. Sensor

-   **Representation**: Defined within the robot's URDF as a Gazebo plugin.
-   **Key Attributes**:
    -   `type`: The kind of sensor (e.g., `camera`, `ray` for LiDAR, `imu`).
    -   `update_rate`: How often the sensor publishes data.
    -   `topic`: The ROS 2 topic where data is published.
    -   Sensor-specific parameters (e.g., resolution for a camera, range for LiDAR).
-   **Data Published**:
    -   **LiDAR**: `sensor_msgs/LaserScan` or `sensor_msgs/PointCloud2`
    -   **Depth Camera**: `sensor_msgs/Image`
    -   **IMU**: `sensor_msgs/Imu`

## 3. Simulation World

-   **Representation**: SDF (Simulation Description Format) `.world` file.
-   **Key Attributes**:
    -   `scene`: Defines ambient light and background color.
    -   `physics`: Sets global physics properties like gravity.
    -   `models`: Includes static objects in the environment (e.g., ground plane, walls, obstacles).
-   **Relationships**:
    -   Contains the `Humanoid Robot Model`.

## 4. Unity Scene

-   **Representation**: A Unity `.unity` scene file.
-   **Key Attributes**:
    -   ROS Connection settings (IP address, port).
    -   A 3D representation of the `Humanoid Robot Model`.
    -   Scripts that subscribe to ROS 2 topics (e.g., `/joint_states`) to animate the robot model.
    -   UI elements for interaction (e.g., buttons to publish messages).
-   **Relationships**:
    -   Connects to the ROS 2 network, which is also connected to the `Simulation World` in Gazebo.
