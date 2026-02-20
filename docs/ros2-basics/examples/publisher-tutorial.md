---
sidebar_position: 4
---

# Publisher Tutorial

In this tutorial, you'll create a ROS 2 publisher node that sends messages to a topic.

---

## What You'll Build

A publisher node that simulates a temperature sensor, sending readings every second.

```
┌──────────────────┐         ┌──────────────────┐
│  Temperature     │         │   Any Node       │
│  Publisher       │ ──────→ │   Listening to   │
│                  │  /temp  │   /temp Topic    │
└──────────────────┘         └──────────────────┘
```

---

## Step 1: Create Your Package

```bash
# Navigate to your workspace src directory
cd ~/ros2_ws/src

# Create a Python package
ros2 pkg create --build-type ament_python temp_publisher
```

---

## Step 2: Write the Publisher Code

Open `temp_publisher/temp_publisher/temperature_sensor.py`:

```python
# temperature_sensor.py
# A ROS 2 node that publishes temperature sensor data

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TemperatureSensor(Node):
    """Publishes simulated temperature sensor data."""
    
    def __init__(self):
        super().__init__('temperature_sensor')
        
        # Create publisher
        # Parameters: message type, topic name, queue size
        self.publisher_ = self.create_publisher(
            Float32, 
            '/temperature', 
            10
        )
        
        # Create timer to publish every second
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(
            timer_period, 
            self.publish_temperature
        )
        
        # Counter for simulation
        self.base_temp = 25.0  # Base temperature in Celsius
        
        self.get_logger().info('Temperature Sensor Node Started')
        self.get_logger().info('Publishing to: /temperature')
        
    def publish_temperature(self):
        """Generate and publish a temperature reading."""
        
        # Create message
        msg = Float32()
        
        # Simulate temperature with small random variation
        variation = random.uniform(-0.5, 0.5)
        msg.data = self.base_temp + variation
        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log the published value
        self.get_logger().info(f'Published: {msg.data:.2f}°C')

def main(args=None):
    """Entry point for the node."""
    rclpy.init(args=args)
    
    # Create the node
    temperature_sensor = TemperatureSensor()
    
    # Keep the node running
    rclpy.spin(temperature_sensor)
    
    # Cleanup
    temperature_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Step 3: Update setup.py

Edit `temp_publisher/setup.py` to register the executable:

```python
from setuptools import setup

package_name = 'temp_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    description='Temperature sensor publisher tutorial',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'temperature_sensor = temp_publisher.temperature_sensor:main',
        ],
    },
)
```

---

## Step 4: Build the Package

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build
colcon build --packages-select temp_publisher

# Source the workspace
source install/setup.bash
```

---

## Step 5: Run the Publisher

```bash
# Run the node
ros2 run temp_publisher temperature_sensor
```

You should see output like:

```
[INFO] [temperature_sensor]: Temperature Sensor Node Started
[INFO] [temperature_sensor]: Publishing to: /temperature
[INFO] [temperature_sensor]: Published: 25.23°C
[INFO] [temperature_sensor]: Published: 24.87°C
[INFO] [temperature_sensor]: Published: 25.41°C
```

---

## Step 6: Verify with ROS 2 CLI

Open a **new terminal** and check the topic:

```bash
# List all topics
ros2 topic list

# You should see /temperature

# Get topic info
ros2 topic info /temperature

# View the messages in real-time
ros2 topic echo /temperature
```

Expected output:

```
data: 25.234567
---
data: 24.876543
---
data: 25.412345
```

---

## Understanding the Code

### Key Components

```python
# 1. Create the publisher
self.publisher_ = self.create_publisher(
    Float32,      # Message type
    '/temperature', # Topic name
    10            # Queue size (buffer)
)
```

```python
# 2. Create a timer (calls function every X seconds)
self.timer = self.create_timer(
    1.0,                    # Period in seconds
    self.publish_temperature # Function to call
)
```

```python
# 3. Publish a message
msg = Float32()
msg.data = 25.5
self.publisher_.publish(msg)
```

---

## Common Message Types

| Type | Package | Use Case |
|------|---------|----------|
| `std_msgs/msg/String` | Text messages |
| `std_msgs/msg/Float32` | Single number |
| `std_msgs/msg/Int32` | Integer |
| `std_msgs/msg/Bool` | True/False |
| `geometry_msgs/msg/Twist` | Velocity commands |
| `sensor_msgs/msg/Image` | Camera images |

---

## Try It: Modify the Publisher

### Challenge 1: Change Publish Rate

Modify the timer to publish 10 times per second:

<details>
<summary>Solution</summary>

```python
timer_period = 0.1  # 0.1 seconds = 10 Hz
self.timer = self.create_timer(timer_period, self.publish_temperature)
```

</details>

### Challenge 2: Add a Counter

Add a message counter to track total publications:

<details>
<summary>Solution</summary>

```python
def __init__(self):
    super().__init__('temperature_sensor')
    self.publisher_ = self.create_publisher(Float32, '/temperature', 10)
    self.timer = self.create_timer(1.0, self.publish_temperature)
    self.base_temp = 25.0
    self.counter = 0  # Add counter
    
def publish_temperature(self):
    msg = Float32()
    self.counter += 1
    variation = random.uniform(-0.5, 0.5)
    msg.data = self.base_temp + variation
    self.publisher_.publish(msg)
    self.get_logger().info(f'Message #{self.counter}: {msg.data:.2f}°C')
```

</details>

### Challenge 3: Simulate Temperature Change

Make the temperature gradually increase over time:

<details>
<summary>Solution</summary>

```python
def publish_temperature(self):
    msg = Float32()
    self.counter += 1
    # Temperature increases by 0.1°C every message
    trend = self.counter * 0.1
    variation = random.uniform(-0.5, 0.5)
    msg.data = self.base_temp + trend + variation
    self.publisher_.publish(msg)
    self.get_logger().info(f'{msg.data:.2f}°C')
```

</details>

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| No output | Check if node is running: `ros2 node list` |
| Wrong topic | List topics: `ros2 topic list` |
| Import errors | Source workspace: `source install/setup.bash` |
| Permission denied | Make script executable: `chmod +x temperature_sensor.py` |

---

## What's Next?

Now that you have a publisher, let's create a [subscriber](./subscriber-tutorial.md) to receive the data!
