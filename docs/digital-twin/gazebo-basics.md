---
sidebar_position: 1
---

# Chapter 1: Gazebo Basics & Robot Simulation

## Overview
Welcome to the world of digital twins! In this chapter, we'll learn about Gazebo, a powerful physics simulator that helps us test robots in a virtual world before building them in real life. Think of Gazebo as a video game where you can build and test robots!

## What is Gazebo?
Gazebo is a 3D simulation environment that creates realistic worlds for robots. It simulates:
- Physics (gravity, friction, collisions)
- Sensors (cameras, LiDAR, IMU)
- Lighting and weather conditions
- Different terrains (grass, sand, concrete)

### Why Use Gazebo?
- **Safe Testing**: Test your robot without breaking anything
- **Cost Effective**: No need to buy expensive hardware
- **Repeatable Experiments**: Same conditions every time
- **Fast Development**: Try many ideas quickly

## Installing Gazebo
For this course, Gazebo should already be installed with ROS2. If not, install it with:

```bash
sudo apt install gazebo libgazebo-dev
```

## Basic Gazebo Concepts

### Worlds
Worlds are environments where your robots live. Gazebo comes with sample worlds like:
- Empty world (just a flat ground)
- Small room
- Maze
- Outdoor environments

### Models
Models are objects in the world:
- Robots (humanoids, wheeled robots, drones)
- Furniture (tables, chairs)
- Obstacles (boxes, walls)
- Sensors (cameras, LiDAR units)

### Plugins
Plugins connect Gazebo to ROS2, allowing:
- Control of robot joints
- Reading sensor data
- Moving objects in the world

## Creating Your First Robot Simulation

### Step 1: Launch Gazebo
Open a terminal and run:
```bash
gazebo
```

### Step 2: Add a Robot
1. Click on the "Insert" tab
2. Choose a simple robot model (like a differential drive robot)
3. Place it in the world

### Step 3: Explore the Interface
- **Left Panel**: Model database
- **Top Bar**: Play/Pause simulation
- **Right Panel**: Scene properties
- **Main Window**: 3D view of the world

## Understanding Physics Simulation

### Gravity
Gravity pulls objects downward. In Gazebo:
- Default gravity: 9.8 m/s² (Earth's gravity)
- Can be changed for moon/mars simulation
- Affects robot movement and falling objects

### Collision Detection
Gazebo calculates when objects touch:
- Prevents objects from passing through each other
- Calculates forces during impacts
- Important for realistic robot behavior

### Friction
Friction affects how objects slide:
- High friction: Objects grip well (rubber tires)
- Low friction: Objects slide easily (ice)
- Affects robot mobility and stability

## Practice Task 1: Basic Exploration
1. Launch Gazebo
2. Insert a simple robot model
3. Try moving the robot around using the GUI
4. Pause and resume the simulation
5. Take a screenshot of your robot in the world

## Working with URDF in Gazebo

URDF (Unified Robot Description Format) describes your robot:
- Physical dimensions
- Joint connections
- Mass properties
- Visual appearance

### Example URDF for a Simple Robot:
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- Simple wheel joint -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0.0 -0.3 0.0" rpy="0 0 0"/>
  </joint>
  
  <!-- Wheel link -->
  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Connecting URDF to Gazebo

To use your URDF in Gazebo, you need Gazebo-specific tags:

```xml
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
</gazebo>
```

## Practice Task 2: URDF Simulation
1. Create a simple URDF file (similar to the example above)
2. Launch Gazebo with your URDF
3. Observe how the robot behaves with physics
4. Modify the URDF and see changes in simulation

## Controlling Robots in Gazebo

### Using ROS2 Topics
Gazebo connects to ROS2 topics:
- `/cmd_vel` for velocity commands
- `/joint_states` for joint positions
- Sensor topics for camera, LiDAR, etc.

### Example Python Controller
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0  # Move forward
        msg.angular.z = 0.5  # Turn slightly
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = SimpleController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary
In this chapter, you learned:
- What Gazebo is and why it's useful
- Basic Gazebo interface and concepts
- How to create simple robot simulations
- How URDF connects to Gazebo
- How to control robots using ROS2

## Next Steps
In the next chapter, we'll explore how to create a digital twin in Unity that mirrors what happens in Gazebo!