---
sidebar_position: 3
---

# Module 3: The AI-Robot Brain (NVIDIA Isaac)

## Introduction

Welcome to Module 3! In this module, you will learn about the **AI-Robot Brain**.

Just like humans have a brain that helps them think and move, robots need a brain too.

**NVIDIA Isaac** is a powerful AI brain for robots. It helps robots:

- **See** and understand the world
- **Think** about what to do
- **Move** safely and smoothly
- **Learn** from practice

By the end of this module, you will understand how to use NVIDIA Isaac tools to build smart robots.

---

## 1. What is the "AI-Robot Brain"?

### The Robot Brain Idea

Think about how your brain works:

```
[Eyes see] → [Brain thinks] → [Body moves]
```

A robot works the same way:

```
[Cameras see] → [AI Brain thinks] → [Motors move]
```

### What Does the AI-Robot Brain Do?

The AI-Robot Brain does three main jobs:

| Job | What It Does | Example |
|-----|--------------|---------|
| **Perception** | Sees and understands the world | "There is a cup on the table" |
| **Planning** | Decides what to do | "I need to pick up the cup" |
| **Control** | Moves the robot body | "Move arm down, close gripper" |

### Why Use NVIDIA Isaac?

NVIDIA Isaac is special because it:

- Uses **GPU acceleration** (very fast computing)
- Works with **real robots** and **simulated robots**
- Has **pre-built AI models** ready to use
- Supports **ROS 2** (the robot software framework)

### The Isaac Family

NVIDIA Isaac has three main parts:

```
┌─────────────────────────────────────────────────────────┐
│                  NVIDIA Isaac Platform                   │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  Isaac Sim   │  │  Isaac ROS   │  │  Isaac Lab   │  │
│  │              │  │              │  │              │  │
│  │  Simulation  │  │  Real-time   │  │  Robot       │  │
│  │  & Training  │  │  AI Skills   │  │  Learning    │  │
│  │              │  │              │  │              │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

1. **Isaac Sim** – Practice in a virtual world
2. **Isaac ROS** – Fast AI for real robots
3. **Isaac Lab** – Train robots with AI

In this module, we focus on **Isaac Sim** and **Isaac ROS**.

---

## 2. NVIDIA Isaac Sim – Photorealistic Simulation

### What is Isaac Sim?

**Isaac Sim** is a robot simulator. It creates a **virtual world** where robots can practice.

Think of it like a video game for robots!

```
┌─────────────────────────────────────────┐
│          Isaac Sim World                │
│                                         │
│    ┌─────┐                              │
│    │Robot│     ┌────┐                   │
│    └─────┘     │Table│                  │
│                └────┘                   │
│      ┌──┐                              │
│      │Cup│     ┌────────┐              │
│      └──┘     │  Chair │              │
│               └────────┘              │
│                                         │
│  Realistic lighting, shadows, physics!  │
└─────────────────────────────────────────┘
```

### Why Use Simulation?

Testing robots in the real world is:

- **Expensive** – Robots can break
- **Slow** – Setting up takes time
- **Dangerous** – Robots might hurt people

Simulation is:

- **Cheap** – No broken robots
- **Fast** – Practice 24/7
- **Safe** – No one gets hurt

### Photorealistic Graphics

Isaac Sim uses **ray tracing** for realistic images.

**Ray tracing** simulates how light works in real life:

- Shadows look real
- Reflections look real
- Materials look real

This helps robots learn in a world that looks **real**.

### Synthetic Data Generation

**Synthetic data** means fake data created by the computer.

Isaac Sim can create thousands of training images:

```
[Robot Camera] → [Isaac Sim] → [10,000 labeled images]
                                      ↓
                              "cup at x=100, y=200"
                              "table at x=300, y=150"
                              "chair at x=500, y=400"
```

This data trains AI models to recognize objects.

### Isaac Sim Example

Here is a simple Python script to start Isaac Sim:

```python
# isaac_sim_hello.py
# A simple Isaac Sim script

from omni.isaac.kit import SimulationApp

# Start the simulation
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core.utils.prims import define_prim

# Create a new world
world = World(stage_units_in_meters=1.0)

# Add a ground plane
world.scene.add_default_ground_plane()

# Add a robot (simplified)
robot_prim = define_prim("/World/MyRobot", "Xform")

# Run the simulation
for i in range(1000):
    world.step(render=True)
    print(f"Step {i}")

# Close simulation
simulation_app.close()
```

### Running Isaac Sim

1. **Install Isaac Sim** from NVIDIA website
2. **Open Isaac Sim** application
3. **Load a scene** or create your own
4. **Add robots** and objects
5. **Run the simulation**

### Isaac Sim with ROS 2

Isaac Sim can talk to ROS 2! This means:

- ROS 2 nodes can control simulated robots
- Simulated sensors publish to ROS 2 topics
- Real robot code works in simulation

```python
# isaac_ros_bridge.py
# Connect Isaac Sim to ROS 2

from omni.isaac.core import World
from omni.isaac.ros2_bridge import SimulationNode

import rclpy
from geometry_msgs.msg import Twist

class IsaacSimROS2:
    def __init__(self):
        # Start simulation
        self.world = World()
        
        # Start ROS 2
        rclpy.init()
        self.node = rclpy.create_node('isaac_sim_bridge')
        
        # Subscribe to robot commands
        self.cmd_sub = self.node.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )
    
    def cmd_callback(self, msg):
        """Receive velocity commands"""
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Apply to simulated robot
        self.apply_robot_velocity(linear, angular)
    
    def apply_robot_velocity(self, linear, angular):
        """Move the simulated robot"""
        # Get robot prim
        robot = self.world.scene.get_object("my_robot")
        
        # Set velocity
        robot.set_linear_velocity([linear, 0, 0])
        robot.set_angular_velocity([0, 0, angular])
    
    def run(self):
        """Main loop"""
        while rclpy.ok():
            # Step simulation
            self.world.step(render=True)
            
            # Process ROS 2 messages
            rclpy.spin_once(self.node, timeout_sec=0)

# Run the bridge
if __name__ == "__main__":
    bridge = IsaacSimROS2()
    bridge.run()
```

### Isaac Sim Features

| Feature | What It Does |
|---------|--------------|
| **URDF Import** | Load robot models from URDF files |
| **Physics Engine** | Realistic collisions and gravity |
| **Sensors** | Cameras, LiDAR, IMU, GPS |
| **ROS 2 Bridge** | Connect to real robot software |
| **Python API** | Control simulation with code |
| **GUI** | Visual editor for building worlds |

---

## 3. Isaac ROS – Hardware-Accelerated AI

### What is Isaac ROS?

**Isaac ROS** is a set of fast AI packages for ROS 2.

It uses **NVIDIA GPUs** to make AI run very fast.

```
┌─────────────────────────────────────────┐
│           Isaac ROS Packages            │
├─────────────────────────────────────────┤
│                                         │
│  ┌─────────────┐  ┌─────────────┐      │
│  │  Isaac ROS  │  │  Isaac ROS  │      │
│  │  VSLAM      │  │  Nav        │      │
│  │             │  │             │      │
│  │  Visual     │  │  Navigation │      │
│  │  SLAM       │  │  with AI    │      │
│  └─────────────┘  └─────────────┘      │
│                                         │
│  ┌─────────────┐  ┌─────────────┐      │
│  │  Isaac ROS  │  │  Isaac ROS  │      │
│  │  Perception │  │  Manipulation│     │
│  │             │  │             │      │
│  │  Object     │  │  Arm Control │      │
│  │  Detection  │  │  & Grasping  │      │
│  └─────────────┘  └─────────────┘      │
│                                         │
└─────────────────────────────────────────┘
```

### Why Hardware Acceleration?

Normal CPU processing is **slow** for AI:

```
CPU: Process image → 100 ms (10 frames per second)
GPU: Process image → 10 ms (100 frames per second)
```

Isaac ROS uses **GPU acceleration** to make AI run **10x faster**!

### Isaac ROS VSLAM

**VSLAM** stands for **Visual Simultaneous Localization and Mapping**.

It helps robots:

1. **Know where they are** (Localization)
2. **Build a map** of the room (Mapping)

```
[Camera Images] → [VSLAM] → [Robot Position + Map]
                               ↓
                    "I am at x=2.5, y=1.0"
                    "Wall at x=5.0, y=0.0"
                    "Door at x=3.0, y=4.0"
```

### VSLAM Example

Here is how to use Isaac ROS VSLAM:

```python
# vslam_example.py
# Using Isaac ROS VSLAM for robot localization

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class VSLAMNode(Node):
    def __init__(self):
        super().__init__('vslam_node')
        
        # Subscribe to camera images
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Subscribe to VSLAM output (position)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/vslam/odometry',
            self.odom_callback,
            10
        )
        
        # Subscribe to VSLAM map
        self.path_sub = self.create_subscription(
            Path,
            '/vslam/path',
            self.path_callback,
            10
        )
        
        # Current position
        self.current_position = None
        self.map_path = []
    
    def image_callback(self, msg):
        """Receive camera image"""
        # Image is processed by VSLAM automatically
        pass
    
    def odom_callback(self, msg):
        """Receive robot position from VSLAM"""
        pos = msg.pose.pose.position
        self.current_position = [pos.x, pos.y, pos.z]
        
        self.get_logger().info(
            f"Robot position: x={pos.x:.2f}, y={pos.y:.2f}"
        )
    
    def path_callback(self, msg):
        """Receive map path from VSLAM"""
        self.map_path = msg.poses
        
        self.get_logger().info(
            f"Map has {len(self.map_path)} points"
        )

def main():
    rclpy.init()
    vslam_node = VSLAMNode()
    rclpy.spin(vslam_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

### Installing Isaac ROS

```bash
# Install Isaac ROS packages
sudo apt install ros-${ROS_DISTRO}-isaac-ros-*

# Or build from source
cd ~/ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_vslam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nav.git

# Build
cd ~/ws
colcon build
source install/setup.bash
```

### Isaac ROS Nodes

Isaac ROS provides pre-built nodes:

| Node | What It Does |
|------|--------------|
| `isaac_ros_vslam` | Visual SLAM for positioning |
| `isaac_ros_depth_image_proc` | Process depth camera images |
| `isaac_ros_tensor_rt` | Fast AI inference with TensorRT |
| `isaac_ros_yolov8` | Object detection with YOLOv8 |
| `isaac_ros_nitros` | Fast message passing |

### Running Isaac ROS VSLAM

```bash
# Launch VSLAM node
ros2 launch isaac_ros_vslam isaac_ros_vslam.launch.py

# Topics available:
# /vslam/odometry - Robot position
# /vslam/path - Map path
# /vslam/pointcloud - 3D point cloud
```

---

## 4. Nav2 – Path Planning for Humanoid Robots

### What is Nav2?

**Nav2** is the **Navigation 2** package for ROS 2.

It helps robots move from point A to point B safely.

```
┌─────────────────────────────────────────┐
│              Nav2 Stack                 │
├─────────────────────────────────────────┤
│                                         │
│  [Goal: "Go to kitchen"]                │
│           ↓                             │
│  ┌─────────────────┐                   │
│  │  Global Planner │  ← Plans full path│
│  └─────────────────┘                   │
│           ↓                             │
│  ┌─────────────────┐                   │
│  │  Local Planner  │  ← Avoids obstacles│
│  └─────────────────┘                   │
│           ↓                             │
│  ┌─────────────────┐                   │
│  │  Controller     │  ← Sends commands │
│  └─────────────────┘                   │
│           ↓                             │
│  [Robot moves safely to kitchen]        │
│                                         │
└─────────────────────────────────────────┘
```

### Nav2 for Bipedal Humanoids

**Bipedal** means walking on two legs (like humans).

Nav2 works with humanoid robots! But there are special considerations:

| Challenge | Solution |
|-----------|----------|
| **Balance** | Use whole-body control |
| **Step planning** | Plan foot placements |
| **Terrain** | Detect stairs and slopes |
| **Speed** | Move slower than wheeled robots |

### Nav2 Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Nav2 Components                       │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  ┌──────────────┐                                       │
│  │  Behavior    │  ← High-level decisions               │
│  │  Tree        │                                       │
│  └──────────────┘                                       │
│         ↓                                               │
│  ┌──────────────┐  ┌──────────────┐                    │
│  │  Global      │  │  Local       │                    │
│  │  Planner     │  │  Planner     │                    │
│  │  (A*, Dijkstra)│ │  (DWB, TEB) │                    │
│  └──────────────┘  └──────────────┘                    │
│         ↓                ↓                              │
│  ┌──────────────────────────────┐                      │
│  │      Costmap 2D              │  ← Obstacle map      │
│  │  ┌────┐ ┌────┐ ┌────┐       │                      │
│  │  │Wall│ │Robot│ │Chair│      │                      │
│  │  └────┘ └────┘ └────┘       │                      │
│  └──────────────────────────────┘                      │
│         ↓                                               │
│  ┌──────────────┐                                       │
│  │  Controller  │  ← Velocity commands                  │
│  └──────────────┘                                       │
│         ↓                                               │
│  ┌──────────────┐                                       │
│  │  Robot Base  │                                       │
│  └──────────────┘                                       │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

### Nav2 Configuration

Create a Nav2 config file (`nav2_params.yaml`):

```yaml
# nav2_params.yaml
# Nav2 configuration for humanoid robot

bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    
    # Behavior tree for navigation
    default_nav_through_poses_bt_xml: "navigate_through_poses.xml"
    default_nav_to_pose_bt_xml: "navigate_to_pose.xml"

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: true

controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller/RregulatedPurePursuitController"
      desired_linear_vel: 0.5  # Slower for humanoid
      max_linear_accel: 0.2    # Gentle acceleration
      max_angular_accel: 0.3

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      
      plugins: ["obstacle_layer", "inflation_layer"]
      
      obstacle_layer:
        plugin: "nav2_costmap_2d/ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
      
      inflation_layer:
        plugin: "nav2_costmap_2d/InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.5

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      static_map: true
      resolution: 0.05
      
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

### Nav2 Example: Send a Goal

```python
# nav2_goal_example.py
# Send a navigation goal to Nav2

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class Nav2GoalClient(Node):
    def __init__(self):
        super().__init__('nav2_goal_client')
        
        # Create action client
        self.action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
    
    def send_goal(self, x, y, theta=0.0):
        """Send goal to Nav2"""
        
        # Wait for Nav2 server
        self.get_logger().info("Waiting for Nav2 server...")
        self.action_client.wait_for_server()
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        
        # Set target pose
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Set orientation (quaternion)
        goal_msg.pose.pose.orientation.z = theta
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f"Sending goal: ({x}, {y})")
        
        # Send goal
        future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal acceptance"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected!")
            return
        
        self.get_logger().info("Goal accepted!")
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Receive progress updates"""
        distance = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f"Distance remaining: {distance:.2f}m")
    
    def result_callback(self, future):
        """Handle final result"""
        result = future.result().result
        self.get_logger().info(f"Navigation result: {result}")

def main():
    rclpy.init()
    client = Nav2GoalClient()
    
    # Send goal: go to position (5.0, 3.0)
    client.send_goal(5.0, 3.0)
    
    rclpy.spin(client)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

### Running Nav2

```bash
# Launch Nav2 with your robot
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  params_file:=nav2_params.yaml

# Or with simulation
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true
```

### Nav2 for Humanoid-Specific Tasks

For bipedal robots, you may need custom planners:

```python
# humanoid_navigator.py
# Custom navigation for bipedal humanoid

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point

class HumanoidNavigator(Node):
    def __init__(self):
        super().__init__('humanoid_navigator')
        
        # Publisher for footstep plan
        self.footstep_pub = self.create_publisher(
            Path, '/footstep_plan', 10
        )
        
        # Humanoid parameters
        self.step_length = 0.3  # meters
        self.step_width = 0.15  # meters
    
    def plan_footsteps(self, start, goal):
        """Plan footsteps from start to goal"""
        
        footsteps = []
        current = start
        step_count = 0
        
        while self.distance(current, goal) > self.step_length:
            # Calculate next step
            direction = self.normalize(
                [goal[0]-current[0], goal[1]-current[1]]
            )
            
            # Alternate left and right foot
            offset = self.step_width if step_count % 2 == 0 else -self.step_width
            
            next_step = [
                current[0] + direction[0] * self.step_length,
                current[1] + direction[1] * self.step_length + offset,
                0.0
            ]
            
            footsteps.append(next_step)
            current = next_step
            step_count += 1
        
        # Add final step to goal
        footsteps.append(goal)
        
        return footsteps
    
    def publish_footsteps(self, footsteps):
        """Publish footstep plan"""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        
        for step in footsteps:
            pose = PoseStamped()
            pose.pose.position.x = step[0]
            pose.pose.position.y = step[1]
            pose.pose.position.z = step[2]
            path_msg.poses.append(pose)
        
        self.footstep_pub.publish(path_msg)
        self.get_logger().info(f"Published {len(footsteps)} footsteps")
    
    def distance(self, p1, p2):
        """Calculate distance between two points"""
        return ((p1[0]-p2[0])**2 + **(p1[1]-p2[1])2)**0.5
    
    def normalize(self, vec):
        """Normalize a vector"""
        length = self.distance([0,0], vec)
        if length == 0:
            return [1, 0]
        return [vec[0]/length, vec[1]/length]

def main():
    rclpy.init()
    navigator = HumanoidNavigator()
    
    # Plan footsteps from (0, 0) to (2, 1)
    start = [0.0, 0.0, 0.0]
    goal = [2.0, 1.0, 0.0]
    
    footsteps = navigator.plan_footsteps(start, goal)
    navigator.publish_footsteps(footsteps)
    
    rclpy.spin(navigator)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

---

## 5. How These Tools Work Together in ROS 2

### The Complete System

Here is how all the pieces fit together:

```
┌─────────────────────────────────────────────────────────────────┐
│                    Complete AI-Robot Brain                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                   Isaac Sim (Simulation)                  │  │
│  │  ┌─────────┐  ┌─────────┐  ┌─────────┐                  │  │
│  │  │  Robot  │  │ Sensors │  │  World  │                  │  │
│  │  │  Model  │  │ (Cam,   │  │ (Tables,│                  │  │
│  │  │         │  │  LiDAR) │  │  Walls) │                  │  │
│  │  └─────────┘  └─────────┘  └─────────┘                  │  │
│  └──────────────────────────────────────────────────────────┘  │
│                            ↓ ROS 2 Topics                      │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                   Isaac ROS (AI Processing)               │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐      │  │
│  │  │   VSLAM     │  │  Perception │  │  Tensor RT  │      │  │
│  │  │  (Position) │  │  (Objects)  │  │  (Fast AI)  │      │  │
│  │  └─────────────┘  └─────────────┘  └─────────────┘      │  │
│  └──────────────────────────────────────────────────────────┘  │
│                            ↓ ROS 2 Actions                     │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                   Nav2 (Navigation)                       │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐      │  │
│  │  │   Global    │  │   Local     │  │  Costmap    │      │  │
│  │  │  Planner    │  │  Planner    │  │  (Obstacles)│      │  │
│  │  └─────────────┘  └─────────────┘  └─────────────┘      │  │
│  └──────────────────────────────────────────────────────────┘  │
│                            ↓ /cmd_vel                          │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                   Robot Hardware                          │  │
│  │  ┌─────────┐  ┌─────────┐  ┌─────────┐                  │  │
│  │  │  Motors │  │  Arms   │  │ Gripper │                  │  │
│  │  └─────────┘  └─────────┘  └─────────┘                  │  │
│  └──────────────────────────────────────────────────────────┘  │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Data Flow Example

Let's trace how a command flows through the system:

**Command**: "Go to the kitchen and pick up the cup"

```
Step 1: Voice Command
┌──────────────┐
│ "Go to the   │
│  kitchen"    │
└──────────────┘
       ↓

Step 2: LLM Planning
┌──────────────┐
│ 1. Navigate  │
│    to kitchen│
│ 2. Find cup  │
│ 3. Pick cup  │
└──────────────┘
       ↓

Step 3: Nav2 Navigation
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│ Global       │ →   │ Local        │ →   │ Robot        │
│ Planner      │     │ Planner      │     │ Moves        │
│ (Full path)  │     │ (Avoid       │     │              │
│              │     │  obstacles)  │     │              │
└──────────────┘     └──────────────┘     └──────────────┘
       ↓                      ↓                    ↓
  Path planned         Adjusts for          Motors turn
  to kitchen           moving people          wheels/legs

Step 4: Isaac ROS Vision
┌──────────────┐     ┌──────────────┐
│ Camera       │ →   │ Object       │
│ Image        │     │ Detection    │
└──────────────┘     └──────────────┘
                            ↓
                    "Cup found at (1.5, 0.3)"

Step 5: Arm Control
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│ Move arm     │ →   │ Close        │ →   │ Lift cup     │
│ to cup       │     │ gripper      │     │              │
└──────────────┘     └──────────────┘     └──────────────┘

Step 6: Task Complete!
┌──────────────┐
│ "Cup picked  │
│  up!"        │
└──────────────┘
```

### Complete Integration Example

```python
# complete_ai_robot_brain.py
# Full integration of Isaac Sim, Isaac ROS, and Nav2

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, LaserScan
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import cv2

class AIBrainCoordinator(Node):
    """Main coordinator for the AI-Robot Brain"""
    
    def __init__(self):
        super().__init__('ai_brain_coordinator')
        
        # === Isaac Sim Bridge ===
        # Subscribe to simulated sensors
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10
        )
        
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10
        )
        
        # === Isaac ROS VSLAM ===
        # Subscribe to robot position
        self.odom_sub = self.create_subscription(
            Twist, '/vslam/odometry', self.odom_callback, 10
        )
        
        # === Nav2 ===
        # Action client for navigation
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        
        # Publisher for robot commands
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        
        # === State ===
        self.current_position = None
        self.detected_objects = []
        self.navigation_active = False
    
    def camera_callback(self, msg):
        """Process camera image from Isaac Sim"""
        # Convert image
        image = self.ros_to_cv2(msg)
        
        # Run object detection (Isaac ROS Perception)
        objects = self.detect_objects(image)
        self.detected_objects = objects
        
        if objects:
            self.get_logger().info(f"Detected: {objects}")
    
    def lidar_callback(self, msg):
        """Process LiDAR scan for obstacle avoidance"""
        # Check for close obstacles
        min_distance = min(msg.ranges)
        
        if min_distance < 0.5:
            self.get_logger().warn("Obstacle too close!")
            self.stop_robot()
    
    def odom_callback(self, msg):
        """Receive position from VSLAM"""
        self.current_position = msg
        self.get_logger().debug(
            f"Position: {msg.linear.x:.2f}, {msg.angular.z:.2f}"
        )
    
    def navigate_to(self, x, y):
        """Send navigation goal to Nav2"""
        self.nav_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        self.get_logger().info(f"Navigating to ({x}, {y})")
        self.nav_client.send_goal_async(goal_msg)
    
    def detect_objects(self, image):
        """Detect objects in image (simplified)"""
        # In real system, use Isaac ROS TensorRT
        # This is a placeholder
        return ["cup", "table"]
    
    def ros_to_cv2(self, ros_image):
        """Convert ROS Image to OpenCV"""
        import numpy as np
        np_arr = np.frombuffer(ros_image.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    def stop_robot(self):
        """Stop robot movement"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
    
    def execute_task(self, task):
        """Execute a high-level task"""
        self.get_logger().info(f"Executing task: {task}")
        
        if task == "go_to_kitchen":
            self.navigate_to(5.0, 3.0)
        elif task == "find_cup":
            # Look for cup in detected objects
            if "cup" in self.detected_objects:
                self.get_logger().info("Cup found!")
            else:
                self.get_logger().info("Searching for cup...")
        elif task == "pick_object":
            self.get_logger().info("Picking object...")
            # Send arm command (not shown)

class IsaacSimBridge(Node):
    """Bridge between Isaac Sim and ROS 2"""
    
    def __init__(self):
        super().__init__('isaac_sim_bridge')
        
        # Publish robot state to Isaac Sim
        self.state_pub = self.create_publisher(
            PoseStamped, '/robot_state', 10
        )
        
        # Timer to send state updates
        self.timer = self.create_timer(0.1, self.publish_state)
    
    def publish_state(self):
        """Send robot state to Isaac Sim"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # In real system, get actual robot state
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        
        self.state_pub.publish(msg)


def main():
    """Main entry point"""
    rclpy.init()
    
    # Create nodes
    coordinator = AIBrainCoordinator()
    sim_bridge = IsaacSimBridge()
    
    # Run all nodes together
    executor = MultiThreadedExecutor()
    executor.add_node(coordinator)
    executor.add_node(sim_bridge)
    
    try:
        # Execute a sample task
        coordinator.execute_task("go_to_kitchen")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### Launch File for Complete System

Create a launch file (`ai_robot_brain.launch.py`):

```python
# ai_robot_brain.launch.py
# Launch the complete AI-Robot Brain system

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Isaac ROS VSLAM
        Node(
            package='isaac_ros_vslam',
            executable='isaac_ros_vslam_node',
            name='vslam_node',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Nav2
        Node(
            package='nav2_bringup',
            executable='navigation_launch.py',
            name='nav2_node',
            parameters=[{'use_sim_time': True}]
        ),
        
        # AI Brain Coordinator
        Node(
            package='ai_robot_brain',
            executable='ai_brain_coordinator',
            name='coordinator',
            parameters=[{'use_sim_time': True}]
        ),
        
        # Isaac Sim Bridge
        Node(
            package='ai_robot_brain',
            executable='isaac_sim_bridge',
            name='sim_bridge',
            parameters=[{'use_sim_time': True}]
        ),
    ])
```

Run the complete system:

```bash
ros2 launch ai_robot_brain ai_robot_brain.launch.py
```

---

## Summary

In this module, you learned about the **AI-Robot Brain** using NVIDIA Isaac:

| Topic | What You Learned |
|-------|------------------|
| **AI-Robot Brain** | How robots think and make decisions |
| **Isaac Sim** | Photorealistic simulation and synthetic data |
| **Isaac ROS** | Hardware-accelerated VSLAM and perception |
| **Nav2** | Path planning for humanoid robots |
| **Integration** | How all tools work together in ROS 2 |

### Key Takeaways

1. **Isaac Sim** lets robots practice in safe, realistic virtual worlds
2. **Isaac ROS** makes AI run fast using GPU acceleration
3. **VSLAM** helps robots know where they are and build maps
4. **Nav2** plans safe paths and avoids obstacles
5. **Integration** of all tools creates a complete AI-Robot Brain

### What's Next?

Try these exercises:

- Install Isaac Sim and run a sample scene
- Set up Isaac ROS VSLAM with a camera
- Configure Nav2 for a simple robot
- Build a complete system that navigates and detects objects

Congratulations on completing Module 3!
