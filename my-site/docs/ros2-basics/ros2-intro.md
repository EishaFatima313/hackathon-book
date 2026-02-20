---
sidebar_position: 1
---

# ROS 2 Basics

## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

Unlike traditional software frameworks, ROS 2 is designed to support distributed computation. That means that you can split your system into many different processes that can be run on different machines, and ROS 2 will help you coordinate between them.

## Key Concepts in ROS 2

### Nodes

A node is a process that performs computation. ROS 2 is designed to be modular at the level of a node — a robot control system usually consists of many nodes. For example, one node controls a laser range finder, one node controls the wheels, and one node might listen to the laser data and compute safe velocities to send to the wheel controller.

Nodes are connected to each other via a peer-to-peer network. Each node can directly communicate with any other node. The ROS 2 runtime provides services to help nodes find each other. Once a node finds another node, they can communicate directly.

### Topics

Topics are named buses over which nodes exchange messages. Each topic has a type associated with it. The type defines the structure of the data that will be passed over that bus. Multiple nodes can subscribe to a topic, and multiple nodes can publish to a topic.

Topics enable asynchronous message passing between nodes. Publishers send messages to a topic without knowing who (if anyone) is subscribed. Subscribers receive messages from a topic without knowing who (if anyone) is publishing.

### Services

Services provide a request/response mechanism for communication between nodes. A service has a name, and a definition that specifies the structure of the request and response. When a node wants to use a service, it sends a request to that service and waits for a response.

Services enable synchronous message passing between nodes. The client node sends a request and waits for the server node to respond.

### Messages

Messages are the data structures that are passed between nodes. Every message has a type that defines its fields. ROS 2 comes with many predefined message types, and you can define your own message types as well.

Messages are the fundamental unit of communication in ROS 2. They can be passed over topics or used in services.

## ROS 2 Architecture Overview

ROS 2 uses a DDS (Data Distribution Service) implementation as its middleware. This provides:

- **Discovery**: Nodes automatically discover each other on the network
- **Communication**: Reliable message passing between nodes
- **Quality of Service (QoS)**: Configurable policies for message delivery

The architecture supports both centralized and decentralized systems, making it suitable for a wide range of robotic applications.

## Simple rclpy Example

Here's a simple example of a ROS 2 publisher node using Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
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
    minimal_publisher = MinimalPublisher()
    
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

And here's a corresponding subscriber node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    
    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Example

To run these examples:

1. Save the publisher code as `publisher.py` and the subscriber code as `subscriber.py`
2. Make sure you have ROS 2 installed and sourced
3. Run the publisher in one terminal: `python publisher.py`
4. Run the subscriber in another terminal: `python subscriber.py`

You should see the publisher sending messages and the subscriber receiving them.

## Summary

In this chapter, we covered the fundamental concepts of ROS 2:
- Nodes: computational processes that perform robot functions
- Topics: named buses for asynchronous message passing
- Services: request/response communication mechanism
- Messages: data structures passed between nodes

These concepts form the foundation of ROS 2 programming and will be essential as we move forward to connecting AI agents with robot control.