---
sidebar_position: 5
---

# Subscriber Tutorial

In this tutorial, you'll create a ROS 2 subscriber node that receives messages from a topic.

---

## What You'll Build

A subscriber node that listens to temperature data and displays statistics.

```
┌──────────────────┐         ┌──────────────────┐
│  Temperature     │         │   Statistics     │
│  Publisher       │ ──────→ │   Subscriber     │
│                  │  /temp  │                  │
└──────────────────┘         └──────────────────┘
```

---

## Prerequisites

Complete the [Publisher Tutorial](./publisher-tutorial.md) first, or have another node publishing to `/temperature`.

---

## Step 1: Create Your Package

```bash
# Navigate to your workspace src directory
cd ~/ros2_ws/src

# Create a Python package
ros2 pkg create --build-type ament_python temp_subscriber
```

---

## Step 2: Write the Subscriber Code

Create `temp_subscriber/temp_subscriber/temperature_monitor.py`:

```python
# temperature_monitor.py
# A ROS 2 node that subscribes to temperature data and shows statistics

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TemperatureMonitor(Node):
    """Subscribes to temperature data and displays statistics."""
    
    def __init__(self):
        super().__init__('temperature_monitor')
        
        # Create subscriber
        # Parameters: message type, topic name, callback function, queue size
        self.subscription = self.create_subscription(
            Float32,
            '/temperature',
            self.temperature_callback,
            10
        )
        
        # Statistics tracking
        self.readings = []
        self.max_readings = 100  # Keep last 100 readings
        
        # Alert thresholds
        self.high_temp = 30.0  # Alert if above 30°C
        self.low_temp = 20.0   # Alert if below 20°C
        
        self.get_logger().info('Temperature Monitor Started')
        self.get_logger().info('Subscribed to: /temperature')
        
    def temperature_callback(self, msg):
        """Called whenever a temperature message is received."""
        
        # Store the reading
        self.readings.append(msg.data)
        
        # Keep only recent readings
        if len(self.readings) > self.max_readings:
            self.readings.pop(0)
        
        # Display the reading
        self.get_logger().info(f'Received: {msg.data:.2f}°C')
        
        # Check for alerts
        self.check_alerts(msg.data)
        
        # Show statistics every 10 readings
        if len(self.readings) % 10 == 0:
            self.show_statistics()
    
    def check_alerts(self, temp):
        """Check if temperature is outside normal range."""
        
        if temp > self.high_temp:
            self.get_logger().warning(
                f'⚠️  HIGH TEMPERATURE ALERT: {temp:.2f}°C'
            )
        elif temp < self.low_temp:
            self.get_logger().warning(
                f'⚠️  LOW TEMPERATURE ALERT: {temp:.2f}°C'
            )
    
    def show_statistics(self):
        """Display temperature statistics."""
        
        if len(self.readings) == 0:
            return
        
        avg_temp = sum(self.readings) / len(self.readings)
        min_temp = min(self.readings)
        max_temp = max(self.readings)
        
        self.get_logger().info(
            f'Statistics | Avg: {avg_temp:.2f}°C | '
            f'Min: {min_temp:.2f}°C | Max: {max_temp:.2f}°C'
        )

def main(args=None):
    """Entry point for the node."""
    rclpy.init(args=args)
    
    # Create the node
    monitor = TemperatureMonitor()
    
    # Keep the node running
    rclpy.spin(monitor)
    
    # Cleanup
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Step 3: Update setup.py

Edit `temp_subscriber/setup.py`:

```python
from setuptools import setup

package_name = 'temp_subscriber'

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
    description='Temperature subscriber tutorial',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'temperature_monitor = temp_subscriber.temperature_monitor:main',
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
colcon build --packages-select temp_subscriber

# Source the workspace
source install/setup.bash
```

---

## Step 5: Run the Subscriber

```bash
# Run the node
ros2 run temp_subscriber temperature_monitor
```

You should see:

```
[INFO] [temperature_monitor]: Temperature Monitor Started
[INFO] [temperature_monitor]: Subscribed to: /temperature
```

---

## Step 6: Run Publisher and Subscriber Together

### Option A: Two Terminals

**Terminal 1:**
```bash
ros2 run temp_publisher temperature_sensor
```

**Terminal 2:**
```bash
ros2 run temp_subscriber temperature_monitor
```

### Option B: Using a Launch File

Create `temp_subscriber/launch/temp_system.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='temp_publisher',
            executable='temperature_sensor',
            name='sensor'
        ),
        Node(
            package='temp_subscriber',
            executable='temperature_monitor',
            name='monitor'
        )
    ])
```

Run with:
```bash
ros2 launch temp_subscriber temp_system.launch.py
```

---

## Understanding the Code

### Key Components

```python
# 1. Create the subscriber
self.subscription = self.create_subscription(
    Float32,           # Message type
    '/temperature',    # Topic name
    self.temperature_callback,  # Callback function
    10                 # Queue size
)
```

```python
# 2. Callback function (automatically called when message arrives)
def temperature_callback(self, msg):
    temp = msg.data
    self.get_logger().info(f'Received: {temp:.2f}°C')
```

---

## Visualizing the Data Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                    Temperature System                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   ┌──────────────┐                                              │
│   │  Publisher   │  [Publishes every 1 second]                 │
│   │  (Sensor)    │  msg.data = 25.3                            │
│   └──────┬───────┘                                              │
│          │                                                       │
│          │ Topic: /temperature                                  │
│          │ Type: Float32                                        │
│          ↓                                                       │
│   ┌──────────────┐                                              │
│   │  Subscriber  │  [Receives and processes]                   │
│   │  (Monitor)   │  - Store reading                            │
│   │              │  - Check alerts                             │
│   │              │  - Show statistics                          │
│   └──────────────┘                                              │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Try It: Modify the Subscriber

### Challenge 1: Change Alert Thresholds

Make the alerts trigger at different temperatures:

<details>
<summary>Solution</summary>

```python
def __init__(self):
    super().__init__('temperature_monitor')
    # ... other code ...
    
    # New thresholds
    self.high_temp = 35.0  # Alert if above 35°C
    self.low_temp = 15.0   # Alert if below 15°C
```

</details>

### Challenge 2: Save Data to File

Add functionality to save readings to a file:

<details>
<summary>Solution</summary>

```python
import csv
from datetime import datetime

def __init__(self):
    super().__init__('temperature_monitor')
    # ... other code ...
    self.log_file = open('temperature_log.csv', 'w', newline='')
    self.csv_writer = csv.writer(self.log_file)
    self.csv_writer.writerow(['timestamp', 'temperature'])

def temperature_callback(self, msg):
    # ... existing code ...
    
    # Log to file
    timestamp = datetime.now().isoformat()
    self.csv_writer.writerow([timestamp, msg.data])
    self.log_file.flush()

def destroy_node(self):
    self.log_file.close()
    super().destroy_node()
```

</details>

### Challenge 3: Subscribe to Multiple Topics

Subscribe to both temperature and humidity:

<details>
<summary>Solution</summary>

```python
def __init__(self):
    super().__init__('environment_monitor')
    
    # Subscribe to temperature
    self.temp_sub = self.create_subscription(
        Float32,
        '/temperature',
        self.temp_callback,
        10
    )
    
    # Subscribe to humidity
    self.humidity_sub = self.create_subscription(
        Float32,
        '/humidity',
        self.humidity_callback,
        10
    )
    
    self.current_temp = None
    self.current_humidity = None

def temp_callback(self, msg):
    self.current_temp = msg.data
    self.get_logger().info(f'Temp: {msg.data:.2f}°C')

def humidity_callback(self, msg):
    self.current_humidity = msg.data
    self.get_logger().info(f'Humidity: {msg.data:.2f}%')
```

</details>

---

## Debug Commands

```bash
# See all nodes
ros2 node list

# See info about your subscriber
ros2 node info /temperature_monitor

# See topic info (who's publishing/subscribing)
ros2 topic info /temperature --verbose

# View messages in real-time
ros2 topic echo /temperature

# Get topic statistics
ros2 topic hz /temperature
```

---

## Common Issues

| Problem | Solution |
|---------|----------|
| No messages received | Check topic name matches publisher |
| Old messages | Queue size might be too large |
| Callback not called | Verify message type matches |
| Memory issues | Limit stored readings (use `pop(0)`) |

---

## What's Next?

You've mastered publishers and subscribers! 

Next, learn about [Services](./service-tutorial.md) for request/response communication.
