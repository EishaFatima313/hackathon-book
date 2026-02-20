---
sidebar_position: 7
---

# Custom Messages Tutorial

In this tutorial, you'll learn to create custom message types for your ROS 2 applications.

---

## Why Custom Messages?

Built-in message types are great, but sometimes you need custom data structures:

```
# Built-in: Single value
std_msgs/Float32 → data: 25.5

# Custom: Multiple values
SensorReading → {
    temperature: 25.5,
    humidity: 60.0,
    pressure: 1013.25,
    timestamp: 1234567890
}
```

---

## Step 1: Create Your Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake custom_messages
```

---

## Step 2: Create .msg Files

Create the `msg` directory:

```bash
cd custom_messages
mkdir -p msg
```

### Message 1: Sensor Reading

Create `custom_messages/msg/SensorReading.msg`:

```
# SensorReading.msg
# Multi-sensor environment reading

Header header
float32 temperature
float32 humidity
float32 pressure
float32 co2
```

### Message 2: Robot Status

Create `custom_messages/msg/RobotStatus.msg`:

```
# RobotStatus.msg
# Robot operational status

string robot_name
bool is_active
float32 battery_level
string current_task
string[] active_sensors
int32 error_code
```

### Message 3: Navigation Goal

Create `custom_messages/msg/NavigationGoal.msg`:

```
# NavigationGoal.msg
# Navigation target for mobile robot

geometry_msgs/PoseStamped target_pose
float32 max_velocity
float32 max_acceleration
bool avoid_obstacles
string[] waypoints
```

---

## Step 3: Update package.xml

Add message generation dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001"?>
<package format="3">
  <name>custom_messages</name>
  <version>0.0.0</version>
  <description>Custom message types tutorial</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## Step 4: Update CMakeLists.txt

Edit `custom_messages/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(custom_messages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorReading.msg"
  "msg/RobotStatus.msg"
  "msg/NavigationGoal.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

---

## Step 5: Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select custom_messages
source install/setup.bash
```

---

## Step 6: Using Custom Messages in Python

### Publisher Example

Create `custom_messages/scripts/sensor_publisher.py`:

```python
#!/usr/bin/env python3
# sensor_publisher.py
# Publish custom SensorReading messages

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from custom_messages.msg import SensorReading
import random
import time

class SensorPublisher(Node):
    """Publishes custom sensor readings."""
    
    def __init__(self):
        super().__init__('sensor_publisher')
        
        # Create publisher for custom message
        self.publisher_ = self.create_publisher(
            SensorReading,
            '/environment/sensor_data',
            10
        )
        
        # Timer to publish every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_reading)
        
        self.get_logger().info('Sensor Publisher Started')
    
    def publish_reading(self):
        """Create and publish a sensor reading."""
        
        # Create message
        msg = SensorReading()
        
        # Set header (includes timestamp and frame_id)
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sensor_frame'
        
        # Set sensor values with random variation
        msg.temperature = 25.0 + random.uniform(-2, 2)
        msg.humidity = 60.0 + random.uniform(-10, 10)
        msg.pressure = 1013.25 + random.uniform(-5, 5)
        msg.co2 = 400 + random.uniform(-50, 50)
        
        # Publish
        self.publisher_.publish(msg)
        
        self.get_logger().info(
            f'Published: T={msg.temperature:.1f}°C, '
            f'H={msg.humidity:.1f}%, P={msg.pressure:.1f}hPa'
        )

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Example

Create `custom_messages/scripts/sensor_subscriber.py`:

```python
#!/usr/bin/env python3
# sensor_subscriber.py
# Subscribe to custom SensorReading messages

import rclpy
from rclpy.node import Node
from custom_messages.msg import SensorReading

class SensorSubscriber(Node):
    """Subscribes to custom sensor readings."""
    
    def __init__(self):
        super().__init__('sensor_subscriber')
        
        # Create subscriber for custom message
        self.subscription = self.create_subscription(
            SensorReading,
            '/environment/sensor_data',
            self.sensor_callback,
            10
        )
        
        self.get_logger().info('Sensor Subscriber Started')
    
    def sensor_callback(self, msg):
        """Handle incoming sensor reading."""
        
        # Access custom message fields
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        
        self.get_logger().info(
            f'[{timestamp:.0f}] Sensor Reading:\n'
            f'  Temperature: {msg.temperature:.2f}°C\n'
            f'  Humidity:    {msg.humidity:.2f}%\n'
            f'  Pressure:    {msg.pressure:.2f} hPa\n'
            f'  CO2:         {msg.co2:.2f} ppm'
        )
        
        # Check for alerts
        if msg.temperature > 30:
            self.get_logger().warning('High temperature detected!')
        if msg.co2 > 500:
            self.get_logger().warning('Elevated CO2 levels!')

def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Step 7: Make Scripts Executable

```bash
chmod +x custom_messages/scripts/*.py
```

Update `setup.py` or `CMakeLists.txt` to install scripts:

```cmake
# Add to CMakeLists.txt
install(PROGRAMS
  scripts/sensor_publisher.py
  scripts/sensor_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)
```

Rebuild:
```bash
colcon build --packages-select custom_messages
source install/setup.bash
```

---

## Step 8: Run and Test

### Terminal 1: Start Publisher

```bash
ros2 run custom_messages sensor_publisher
```

### Terminal 2: Start Subscriber

```bash
ros2 run custom_messages sensor_subscriber
```

### Terminal 3: View with CLI

```bash
# List topics
ros2 topic list

# Get topic info
ros2 topic info /environment/sensor_data

# Echo messages
ros2 topic echo /environment/sensor_data
```

Expected output:
```
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: sensor_frame
temperature: 26.5
humidity: 58.3
pressure: 1015.2
co2: 420.5
```

---

## Using Custom Messages in Launch Files

Create `custom_messages/launch/sensor_system.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='custom_messages',
            executable='sensor_publisher',
            name='environment_sensor',
            output='screen'
        ),
        Node(
            package='custom_messages',
            executable='sensor_subscriber',
            name='environment_monitor',
            output='screen'
        )
    ])
```

Run with:
```bash
ros2 launch custom_messages sensor_system.launch.py
```

---

## Message Definition Reference

### Field Types

| Type | Description | Example |
|------|-------------|---------|
| `bool` | True/False | `bool is_active` |
| `int8`, `int16`, `int32`, `int64` | Signed integers | `int32 error_code` |
| `uint8`, `uint16`, `uint32`, `uint64` | Unsigned integers | `uint32 count` |
| `float32`, `float64` | Floating point | `float32 temperature` |
| `string` | Text | `string robot_name` |
| `time`, `duration` | Time values | `time timestamp` |
| `Header` | ROS header | `Header header` |
| `type[]` | Arrays | `string[] names` |
| `package/Type` | Other messages | `geometry_msgs/Pose pose` |

### Constants

```
# Constants in messages
float32 PI=3.14159
string VERSION="1.0"
int32 MAX_VALUE=100
```

---

## Best Practices

### 1. Use Descriptive Names

```
# Good
float32 battery_voltage
string robot_id

# Avoid
float32 bv
string id
```

### 2. Include Header for Time-Sensitive Data

```
Header header  # Always include for sensor data
float32 value
```

### 3. Use Standard Types When Possible

```
# Use built-in types
geometry_msgs/Pose pose
sensor_msgs/Image image

# Create custom only when needed
```

### 4. Document Your Messages

```
# MotorCommand.msg
# Command for controlling robot motor

float32 velocity    # Desired velocity in m/s
float32 acceleration # Max acceleration in m/s²
int32 motor_id      # Motor identifier (0-7)
```

---

## Debugging Custom Messages

```bash
# Show message definition
ros2 interface show custom_messages/msg/SensorReading

# List all message types in package
ros2 interface list | grep custom_messages

# Check if message was built correctly
ros2 run custom_messages sensor_publisher
```

---

## What's Next?

You now know how to create custom messages! 

Continue to [Exercises](../exercises.md) to practice everything you've learned.
