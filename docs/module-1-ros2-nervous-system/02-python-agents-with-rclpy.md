---
sidebar_position: 2
---

# Chapter 2: Python Agents with rclpy

Now that we understand the fundamental concepts of ROS 2, let's put them into practice. This chapter will show you how to write your own ROS 2 nodes using Python, the language of choice for many AI and robotics applications. We will use the `rclpy` library, which is the official ROS 2 client library for Python.

## T007: Introduction to rclpy
`rclpy` provides the tools to create nodes, publishers, subscribers, and more, directly within a Python script. This is how we bridge the high-level logic of an AI agent with the low-level control of a robot's hardware.

## T008: A Simple Publisher Node
Let's create a node that publishes a simple string message to a topic named `chatter`. This could represent an AI agent sending a command.

```python
# T008: publisher_example.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ChatterPublisher(Node):
    def __init__(self):
        super().__init__('chatter_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    chatter_publisher = ChatterPublisher()
    rclpy.spin(chatter_publisher)
    chatter_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## T009: A Simple Subscriber Node
Now, let's create a node that listens to the `chatter` topic and prints the messages it receives. This could represent a robot's controller receiving a command from an AI agent.

```python
# T009: subscriber_example.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ChatterSubscriber(Node):
    def __init__(self):
        super().__init__('chatter_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    chatter_subscriber = ChatterSubscriber()
    rclpy.spin(chatter_subscriber)
    chatter_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## T010: Bridging AI Agents to Robot Controllers
The publisher/subscriber model shown above is the core pattern for connecting an AI agent to a robot.
-   **The AI Agent**: The agent's logic would be inside a node like `ChatterPublisher`. Instead of publishing "Hello World", it would publish meaningful commands, like `{'action': 'wave', 'speed': 0.8}`.
-   **The Robot Controller**: A motor control node on the robot would be a subscriber, like `ChatterSubscriber`. It would listen for these command messages and translate them into the low-level electrical signals needed to move the robot's arm.

## T011: The Control Loop
The `timer_callback` in our publisher is a simple form of a **control loop**. In a real robot, this loop is where the "thinking" happens. A typical control loop for an AI agent would look like this:

1.  **Read Sensors**: Subscribe to topics providing data from cameras, lidars, joint encoders, etc.
2.  **Process Data**: The AI algorithm analyzes the sensor data to understand the current state of the world and the robot.
3.  **Make a Decision**: Based on its goals and the current state, the agent decides what to do next.
4.  **Publish Commands**: Publish messages to actuator topics (e.g., `/arm_controller/command`, `/cmd_vel`) to execute the decision.
5.  **Repeat**: This loop runs continuously, often at a high frequency (e.g., 10-100 times per second), allowing the robot to react dynamically to its environment.
