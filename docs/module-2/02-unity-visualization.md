---
sidebar_position: 2
---

# Chapter 5: Visualizing the Robot in Unity

This tutorial explains how to set up a Unity scene to act as a high-fidelity, interactive visualizer for our Gazebo simulation.

## 1. Install ROS-TCP-Connector

First, in your new Unity project, you need to install the `ROS-TCP-Connector` package. You can do this via the Unity Package Manager by adding a package from a git URL.

1.  Go to `Window -> Package Manager`.
2.  Click the `+` icon and select `Add package from git URL...`.
3.  Enter `https://github.com/Unity-Technologies/ROS-TCP-Connector.git` and click `Add`.

*(Screenshot of Unity Package Manager would go here)*

## 2. Import the Robot URDF

Next, we need a package to import our robot's URDF model. We'll use the `URDF-Importer` for this.

1.  Go to `Window -> Package Manager`.
2.  Click the `+` icon and select `Add package from git URL...`.
3.  Enter `https://github.com/Unity-Technologies/URDF-Importer.git` and click `Add`.
4.  Once installed, you can go to `Assets -> Import Robot from URDF` and select your `robot.urdf` file.

*(Screenshot of the imported robot model in the Unity scene would go here)*

## 3. Subscribe to Joint States

To make the robot move, we need to subscribe to the `/joint_states` topic published by `robot_state_publisher` and update the model in Unity.

Create a new C# script called `JointStateSubscriber` and add the following code:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStateSubscriber : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>("/joint_states", UpdateJoints);
    }

    void UpdateJoints(JointStateMsg jointState)
    {
        // This is a simplified example. You would need to map the joint names
        // from the message to the corresponding ArticulationBody components
        // on your imported robot model and set their joint positions.
        Debug.Log("Received joint states");
    }
}
```

Attach this script to an empty GameObject in your scene.

## 4. Configure and Run

Finally, configure the connection and run the scene.

1.  In the Unity menu, go to `Robotics -> ROS Settings`.
2.  Set the `ROS IP Address` to `127.0.0.1`.
3.  Press the "Play" button.

You should see the "Received joint states" message in the console, confirming that your Unity scene is connected to the ROS 2 network.

