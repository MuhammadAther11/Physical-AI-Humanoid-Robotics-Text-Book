# Quickstart: Digital Twin Simulation

This guide provides the steps to get the digital twin simulation running.

### Prerequisites

1.  **ROS 2 Humble**: Ensure you have a working installation.
2.  **Gazebo**: The default simulator shipped with ROS 2 Humble.
3.  **Unity Hub** and **Unity 2022.3 (LTS)** or later.
4.  The example ROS 2 package for this module.

### Steps

1.  **Build the ROS 2 Package**

    ```bash
    cd your_ros2_ws
    colcon build --packages-select digital_twin_examples
    source install/setup.bash
    ```

2.  **Launch the Gazebo Simulation**

    Open a new terminal and run the following command:

    ```bash
    ros2 launch digital_twin_examples simulation.launch.py
    ```

    This will open Gazebo, and you should see a humanoid robot load and settle on the ground plane.

3.  **Configure Unity**

    -   Open the example Unity project.
    -   Go to the `ROS-TCP-Connector` settings in the toolbar.
    -   Ensure the `ROS IP Address` is set to `127.0.0.1` or the IP of your machine running ROS 2.
    -   Open the `DigitalTwin` scene.

4.  **Run the Unity Scene**

    -   Press the "Play" button in the Unity Editor.
    -   The scene will connect to the ROS 2 network.
    -   The robot in Unity should now mirror the pose of the robot in Gazebo. Move the robot in Gazebo (e.g., by applying a force) and see the change reflected in Unity.

5.  **Visualize Sensor Data**

    Open a new terminal and run RViz2:

    ```bash
    rviz2
    ```

    -   Add a `PointCloud2` display and subscribe to the `/lidar/points` topic to see the simulated LiDAR data.
    -   Add an `Image` display and subscribe to the `/depth_camera/image_raw` topic to see the simulated camera feed.
