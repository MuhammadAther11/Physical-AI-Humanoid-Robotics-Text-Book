# Data Model: Module 1 - ROS 2 Nervous System

**Purpose**: To define the key conceptual entities that will be explained in this educational module. These are not software data models but are the core concepts being taught.

## Key Entities

### 1. ROS 2 Node
- **Represents**: An independent, executable process in a ROS 2 system. It is the primary unit of computation.
- **Key Attributes/Concepts**:
    - **Name**: A unique identifier within the ROS graph.
    - **Publishers**: Interfaces for sending out messages on a topic.
    - **Subscribers**: Interfaces for receiving messages from a topic.
    - **Services**: Interfaces for request/reply communication.
    - **Actions**: Interfaces for long-running, feedback-driven tasks.
- **Relationships**: A node is a participant in the larger ROS 2 graph, communicating with other nodes via topics, services, and actions.

### 2. ROS 2 Topic
- **Represents**: A named bus for messages. It is the primary mechanism for many-to-many, asynchronous communication.
- **Key Attributes/Concepts**:
    - **Name**: A unique identifier for the bus (e.g., `/cmd_vel`).
    - **Message Type**: The defined data structure of messages that can be sent on the topic (e.g., `geometry_msgs/Twist`).
- **Relationships**: Topics are published to by nodes and subscribed to by nodes. A single topic can have many publishers and many subscribers.

### 3. URDF (Unified Robot Description Format)
- **Represents**: An XML format for describing the physical properties of a robot model.
- **Key Attributes/Concepts**:
    - **`<link>`**: Describes a rigid body part of the robot, including its visual, collision, and inertial properties.
    - **`<joint>`**: Describes the kinematics and dynamics of the connection between two links. Defines the type of motion (e.g., revolute, prismatic).
    - **`<sensor>`**: Describes a sensor attached to a link.
- **Relationships**: Links are connected by joints to form a kinematic chain, which represents the robot's structure as a tree.
