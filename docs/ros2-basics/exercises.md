---
sidebar_position: 6
---

# Exercises & Practice

Test your ROS 2 knowledge with these hands-on exercises!

---

## Exercise 1: Hello ROS 2 Node

**Goal:** Create your first ROS 2 node.

### Task

Create a node that prints "Hello, ROS 2!" to the console.

### Starter Code

```python
# my_first_node.py
import rclpy
from rclpy.node import Node

class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        # TODO: Add your code here
        pass

def main():
    rclpy.init()
    node = HelloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Solution

<details>
<summary>Click to reveal solution</summary>

```python
import rclpy
from rclpy.node import Node

class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.get_logger().info('Hello, ROS 2!')

def main():
    rclpy.init()
    node = HelloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

</details>

---

## Exercise 2: Simple Publisher

**Goal:** Create a node that publishes messages to a topic.

### Task

Create a publisher that sends a greeting message every second.

### Requirements

- Topic name: `/greeting`
- Message type: `std_msgs/msg/String`
- Publish rate: 1 Hz (once per second)

### Starter Code

```python
# greeting_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GreetingPublisher(Node):
    def __init__(self):
        super().__init__('greeting_publisher')
        # TODO: Create publisher
        # TODO: Create timer
        
    def publish_greeting(self):
        # TODO: Publish message
        pass

def main():
    rclpy.init()
    node = GreetingPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Solution

<details>
<summary>Click to reveal solution</summary>

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GreetingPublisher(Node):
    def __init__(self):
        super().__init__('greeting_publisher')
        self.publisher_ = self.create_publisher(String, '/greeting', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_greeting)
        self.count = 0
        
    def publish_greeting(self):
        msg = String()
        self.count += 1
        msg.data = f'Hello! Count: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main():
    rclpy.init()
    node = GreetingPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

</details>

---

## Exercise 3: Simple Subscriber

**Goal:** Create a node that subscribes to a topic.

### Task

Create a subscriber that listens to the `/greeting` topic and prints received messages.

### Requirements

- Subscribe to: `/greeting`
- Message type: `std_msgs/msg/String`
- Print each received message

### Starter Code

```python
# greeting_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GreetingSubscriber(Node):
    def __init__(self):
        super().__init__('greeting_subscriber')
        # TODO: Create subscription
        
    def greeting_callback(self, msg):
        # TODO: Print the message
        pass

def main():
    rclpy.init()
    node = GreetingSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Solution

<details>
<summary>Click to reveal solution</summary>

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GreetingSubscriber(Node):
    def __init__(self):
        super().__init__('greeting_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/greeting',
            self.greeting_callback,
            10
        )
        
    def greeting_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main():
    rclpy.init()
    node = GreetingSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

</details>

---

## Exercise 4: Publisher + Subscriber Test

**Goal:** Run both nodes together and observe communication.

### Task

1. Open **Terminal 1** and run the subscriber
2. Open **Terminal 2** and run the publisher
3. Observe the messages in both terminals

### Commands

```bash
# Terminal 1
ros2 run my_package greeting_subscriber

# Terminal 2
ros2 run my_package greeting_publisher
```

### Questions

1. What happens when you stop the publisher?
2. What happens when you stop the subscriber?
3. How many subscribers can listen to the same topic?

---

## Exercise 5: Custom Message - Sensor Data

**Goal:** Create and use a custom message type.

### Task

Create a custom message for sensor data with temperature and humidity.

### Steps

1. Create a `.msg` file:
```
# msg/SensorData.msg
float32 temperature
float32 humidity
int64 timestamp
```

2. Update `package.xml` to include message generation dependencies
3. Update `setup.py` to include the `.msg` file
4. Create a publisher that sends random sensor data
5. Create a subscriber that displays the data

---

## Exercise 6: Service Server

**Goal:** Create a service that adds two numbers.

### Task

Create a service that takes two integers and returns their sum.

### Requirements

- Service type: `example_interfaces/srv/AddTwoInts`
- Service name: `/add_two_ints`

### Starter Code

```python
# add_server.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        # TODO: Create service
        
    def add_callback(self, request, response):
        # TODO: Add the numbers
        return response

def main():
    rclpy.init()
    node = AddTwoIntsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Solution

<details>
<summary>Click to reveal solution</summary>

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts, 
            '/add_two_ints', 
            self.add_callback
        )
        
    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Adding {request.a} + {request.b} = {response.sum}')
        return response

def main():
    rclpy.init()
    node = AddTwoIntsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

</details>

---

## Exercise 7: Service Client

**Goal:** Create a client that calls the add service.

### Task

Create a client that sends two numbers to the `/add_two_ints` service.

### Starter Code

```python
# add_client.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        # TODO: Create client
        
    def send_request(self, a, b):
        # TODO: Send request and wait for response
        pass

def main():
    rclpy.init()
    node = AddTwoIntsClient()
    node.send_request(5, 3)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Exercise 8: Launch File

**Goal:** Create a launch file that starts multiple nodes.

### Task

Create a launch file that starts both the publisher and subscriber.

### Starter Code

```python
# launch/greeting_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # TODO: Add publisher node
        # TODO: Add subscriber node
    ])
```

### Solution

<details>
<summary>Click to reveal solution</summary>

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='greeting_publisher',
            name='publisher'
        ),
        Node(
            package='my_package',
            executable='greeting_subscriber',
            name='subscriber'
        )
    ])
```

</details>

---

## Quiz - Test Your Knowledge

### Question 1

What is a ROS 2 node?

A) A type of message
B) A program that performs a specific task
C) A communication channel
D) A configuration file

<details>
<summary>Answer</summary>
**B) A program that performs a specific task**
</details>

---

### Question 2

Which communication pattern is best for streaming sensor data?

A) Service
B) Action
C) Topic (Publish/Subscribe)
D) Parameter

<details>
<summary>Answer</summary>
**C) Topic (Publish/Subscribe)**
</details>

---

### Question 3

What command builds a ROS 2 workspace?

A) `catkin_make`
B) `ros2 build`
C) `colcon build`
D) `make`

<details>
<summary>Answer</summary>
**C) `colcon build`**
</details>

---

### Question 4

Which message type would you use for velocity commands?

A) `std_msgs/String`
B) `geometry_msgs/Twist`
C) `sensor_msgs/Image`
D) `nav_msgs/Odometry`

<details>
<summary>Answer</summary>
**B) `geometry_msgs/Twist`**
</details>

---

### Question 5

What is the purpose of a launch file?

A) To compile code
B) To start multiple nodes at once
C) To create new packages
D) To view topics

<details>
<summary>Answer</summary>
**B) To start multiple nodes at once**
</details>

---

## Challenge Project: Mini Robot System

**Goal:** Build a complete mini system that simulates a robot.

### Requirements

Create 3 nodes:

1. **Sensor Node** - Publishes random "distance to obstacle" values (0-100 cm)
2. **Decision Node** - Subscribes to distance, publishes commands:
   - If distance < 20: publish "STOP"
   - If distance < 50: publish "SLOW"
   - If distance >= 50: publish "GO"
3. **Motor Node** - Subscribes to commands and prints the action

### Bonus

- Add a launch file to start all nodes
- Add parameters for distance thresholds
- Add a service to reset the system

---

## Troubleshooting Guide

### Common Errors

| Error | Cause | Solution |
|-------|-------|----------|
| `ModuleNotFoundError: No module named 'rclpy'` | ROS 2 not sourced | Run `source /opt/ros/humble/setup.bash` |
| `Executable not found` | Package not built | Run `colcon build` and source install |
| `No messages received` | Topics don't match | Check topic names with `ros2 topic list` |
| `Service not available` | Server not running | Start the service server first |

### Debug Commands

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# View messages on a topic
ros2 topic echo /topic_name

# List all services
ros2 service list

# Check node info
ros2 node info /node_name

# View topic connections
ros2 topic info /topic_name --verbose
```

---

## Next Steps

Congratulations on completing Module 1! 

Ready to move on? Check out:
- [Module 2: AI → Robot Control](../ai-robot-control)
- [Module 3: Digital Twin](../digital-twin)
