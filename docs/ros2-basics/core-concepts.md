---
sidebar_position: 2
---

# ROS 2 Core Concepts

In this section, we'll dive deep into the fundamental building blocks of ROS 2.

---

## 1. Nodes – The Basic Unit

A **node** is a program that performs a specific task.

### Key Points

- Each node should do **one thing well** (like a Unix tool)
- Nodes can run on the **same computer** or **different computers**
- Nodes can be written in **Python**, **C++**, or other languages
- Nodes can be **started** and **stopped** independently

### Example: A Simple Node

```python
# my_node.py
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Hello from ROS 2!')

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running a Node

```bash
# Run the node
python3 my_node.py

# Or with ROS 2 CLI
ros2 run my_package my_node
```

---

## 2. Topics – Publish/Subscribe Communication

**Topics** are how nodes broadcast data to other nodes.

### How Topics Work

```
              Topic: /sensor_data
              ┌─────────────────┐
              │                 │
    ┌─────┐   │   ┌─────┐       │   ┌─────┐
    │ Pub │───┴──→│ Sub │       │──→│ Sub │
    │     │       │     │       │   │     │
    └─────┘       └─────┘       │   └─────┘
                                │
                                │   ┌─────┐
                                └──→│ Sub │
                                    └─────┘
```

### Key Characteristics

| Property | Description |
|----------|-------------|
| **Many-to-many** | Multiple publishers and subscribers |
| **Anonymous** | Publishers don't know who subscribes |
| **Asynchronous** | No waiting for responses |
| **Streaming** | Continuous flow of data |

### Common Topic Types

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/camera/image` | `sensor_msgs/Image` | Camera images |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR data |
| `/odom` | `nav_msgs/Odometry` | Robot position |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/joint_states` | `sensor_msgs/JointState` | Joint positions |

---

## 3. Messages – Data Structures

A **message** is the data sent between nodes.

### Built-in Message Types

ROS 2 comes with many standard message types:

```python
# geometry_msgs/Twist - for velocity commands
linear:
  x: 1.0    # forward speed (m/s)
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5    # rotation speed (rad/s)
```

```python
# sensor_msgs/Image - for camera images
header:
  stamp: time
  frame_id: "camera_frame"
height: 480
width: 640
encoding: "rgb8"
data: [raw pixel data]
```

```python
# geometry_msgs/PoseStamped - for position + orientation
header:
  stamp: time
  frame_id: "map"
pose:
  position:
    x: 1.0
    y: 2.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.707  # quaternion
    w: 0.707
```

### Creating Custom Messages

Create a `.msg` file in your package:

```
# msg/Person.msg
string name
int32 age
string email
```

Then update your `package.xml` and `CMakeLists.txt` to build it.

---

## 4. Services – Request/Response Communication

**Services** are for synchronous request/response communication.

### How Services Work

```
┌──────────┐      Request       ┌──────────┐
│  Client  │ ──────────────────→│  Server  │
│          │                    │          │
│          │ ←──────────────────│          │
│          │      Response      │          │
└──────────┘                    └──────────┘
```

### Common Service Types

| Service | Description |
|---------|-------------|
| `/spawn` | Spawn a new turtle (in turtlesim) |
| `/reset` | Reset the simulation |
| `/get_parameters` | Get robot parameters |
| `/set_mode` | Change robot operating mode |

### Example Service Definition

```
# srv/AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

---

## 5. Actions – Long-Running Tasks

**Actions** are for tasks that take time and need progress updates.

### How Actions Work

```
┌──────────┐       Goal         ┌──────────┐
│  Client  │ ──────────────────→│  Server  │
│          │ ←──────────────────│          │
│          │      Feedback      │          │
│          │ ←──────────────────│          │
│          │      Feedback      │          │
│          │ ←──────────────────│          │
│          │       Result       │          │
└──────────┘                    └──────────┘
```

### When to Use Actions

| Use Actions When... | Example |
|---------------------|---------|
| Task takes time | Navigate to a location |
| Need progress updates | "50% complete" |
| May need to cancel | Stop navigation mid-way |

### Example Action Definition

```
# action/Navigate.action
goal:
  geometry_msgs/PoseStamped target_pose
---
result:
  bool success
  float32 distance_traveled
---
feedback:
  geometry_msgs/PoseStamped current_pose
  float32 remaining_distance
```

---

## 6. Parameters – Configuration at Runtime

**Parameters** are like settings for nodes.

### Setting Parameters

```bash
# Set a parameter
ros2 param set /my_node max_speed 1.5

# Get a parameter
ros2 param get /my_node max_speed

# List all parameters
ros2 param list
```

### Using Parameters in Code

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Declare and get parameter
        self.declare_parameter('max_speed', 1.0)
        self.max_speed = self.get_parameter('max_speed').value
        
        self.get_logger().info(f'Max speed: {self.max_speed}')
```

---

## 7. Launch Files – Start Multiple Nodes

**Launch files** start multiple nodes at once with configuration.

### Python Launch File Example

```python
# launch/my_robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='sensor_node',
            name='camera',
            parameters=[{'frame_rate': 30}]
        ),
        Node(
            package='my_package',
            executable='processor_node',
            name='vision',
            remappings=[('/input', '/camera/image')]
        ),
        Node(
            package='my_package',
            executable='controller_node',
            name='motor_control'
        )
    ])
```

### Running a Launch File

```bash
ros2 launch my_package my_robot.launch.py
```

---

## 8. ROS 2 CLI Tools

The `ros2` command-line tool is essential for debugging.

### Essential Commands

| Command | Description |
|---------|-------------|
| `ros2 node list` | List all running nodes |
| `ros2 topic list` | List all topics |
| `ros2 topic echo /topic_name` | View messages on a topic |
| `ros2 service list` | List all services |
| `ros2 param list` | List parameters |
| `ros2 launch pkg file.launch.py` | Run a launch file |
| `ros2 run pkg executable` | Run a single node |

### Example Workflow

```bash
# See what's running
ros2 node list

# Check topics
ros2 topic list

# View camera images topic
ros2 topic echo /camera/image

# Check node parameters
ros2 param list

# Run a node
ros2 run turtlesim turtlesim_node
```

---

## Architecture Summary

Here's how all the pieces fit together:

```
┌─────────────────────────────────────────────────────────────────┐
│                     ROS 2 System Architecture                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   ┌─────────────┐                                               │
│   │   Node 1    │  [Publisher]                                  │
│   │  (Sensor)   │  → Topics: /sensor_data                      │
│   │             │  Parameters: frame_rate, resolution          │
│   └─────────────┘                                               │
│                                                                  │
│   ┌─────────────┐                                               │
│   │   Node 2    │  [Subscriber + Service Server]               │
│   │ (Processor) │  ← Topics: /sensor_data                      │
│   │             │  → Topics: /processed_data                   │
│   │             │  Services: /reset, /configure                │
│   └─────────────┘                                               │
│                                                                  │
│   ┌─────────────┐                                               │
│   │   Node 3    │  [Action Client]                             │
│   │ (Controller)│  Actions: /navigate                          │
│   │             │  ← Topics: /processed_data                   │
│   │             │  → Topics: /cmd_vel                          │
│   └─────────────┘                                               │
│                                                                  │
│   ┌─────────────┐                                               │
│   │   Node 4    │  [Action Server]                             │
│   │ (Navigator) │  Actions: /navigate                          │
│   │             │  Services: /get_position                     │
│   └─────────────┘                                               │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Summary Table

| Concept | Communication Type | Use Case |
|---------|-------------------|----------|
| **Topic** | Publish/Subscribe | Streaming sensor data |
| **Service** | Request/Response | Quick queries, settings |
| **Action** | Goal/Feedback/Result | Long tasks (navigation) |
| **Parameter** | Configuration | Runtime settings |
| **Launch File** | Startup | Start multiple nodes |

---

## What's Next?

Now that you understand the core concepts, let's set up your ROS 2 environment!

Continue to [Setup Guide](./setup.md).
