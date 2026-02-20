---
sidebar_position: 2
---

# AI Agents → Robot Control

## Connecting AI Agents to ROS Controllers

In this chapter, we'll explore how to bridge Python-based AI agents with ROS 2 controllers to enable intelligent robot behavior. This connection forms the "brain" of our robotic system, allowing AI algorithms to control physical or simulated robots.

## Understanding the Bridge Architecture

The connection between AI agents and robot controllers typically follows this pattern:

```
AI Agent (Python) → ROS 2 Publisher → Robot Controller → Physical/Simulated Robot
```

The AI agent makes decisions based on sensor inputs and sends commands to the robot through ROS 2 topics. The robot controller receives these commands and executes them on the actual hardware or simulation.

## Using rclpy to Publish and Subscribe

### Publishing Commands from AI Agents

Here's an example of how an AI agent can publish movement commands to a robot:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class AINavigationAgent(Node):
    def __init__(self):
        super().__init__('ai_navigation_agent')
        
        # Publisher for robot velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for laser scan data (sensor input)
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Timer for AI decision-making loop
        self.timer = self.create_timer(0.1, self.ai_decision_loop)
        
        # Internal state
        self.laser_data = None
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def scan_callback(self, msg):
        """Process incoming laser scan data"""
        self.laser_data = msg.ranges
    
    def ai_decision_loop(self):
        """Main AI decision-making function"""
        if self.laser_data is None:
            return
            
        # Simple obstacle avoidance algorithm
        cmd_vel = self.simple_avoidance_algorithm()
        
        # Publish the command
        self.cmd_vel_publisher.publish(cmd_vel)
        
    def simple_avoidance_algorithm(self):
        """Simple AI algorithm for obstacle avoidance"""
        msg = Twist()
        
        if self.laser_data:
            # Get distances in front, left, and right
            front_distances = self.laser_data[330:390]  # Front 60 degrees
            left_distances = self.laser_data[60:120]    # Left 60 degrees  
            right_distances = self.laser_data[240:300]  # Right 60 degrees
            
            # Calculate minimum distances
            min_front = min(front_distances) if front_distances else float('inf')
            min_left = min(left_distances) if left_distances else float('inf')
            min_right = min(right_distances) if right_distances else float('inf')
            
            # Simple navigation logic
            if min_front > 1.0:  # Clear path ahead
                msg.linear.x = 0.5  # Move forward
                msg.angular.z = 0.0
            elif min_left > min_right:  # Turn left is safer
                msg.linear.x = 0.2
                msg.angular.z = 0.5
            else:  # Turn right is safer
                msg.linear.x = 0.2
                msg.angular.z = -0.5
                
        return msg

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AINavigationAgent()
    
    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Receiving Robot State Information

AI agents often need to receive state information from the robot. Here's an example of subscribing to robot odometry:

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
import math

class AILocalizationAgent(Node):
    def __init__(self):
        super().__init__('ai_localization_agent')
        
        # Subscriber for robot pose
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publisher for navigation goals
        self.goal_publisher = self.create_publisher(Pose, '/goal_pose', 10)
        
        # Internal state
        self.current_pose = None
        self.current_twist = None
        
    def odom_callback(self, msg):
        """Process incoming odometry data"""
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist
        
        # Process the data with AI algorithms
        self.process_robot_state()
        
    def process_robot_state(self):
        """AI processing of robot state"""
        if self.current_pose:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            
            # Example: Check if robot is near a boundary
            if abs(x) > 5.0 or abs(y) > 5.0:
                # Send a goal back toward center
                goal_msg = Pose()
                goal_msg.position.x = 0.0
                goal_msg.position.y = 0.0
                self.goal_publisher.publish(goal_msg)
                
    def distance_to_goal(self, goal_x, goal_y):
        """Calculate Euclidean distance to goal"""
        if self.current_pose:
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            return math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
        return float('inf')

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AILocalizationAgent()
    
    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced AI Integration Patterns

### Reinforcement Learning Agent

Here's an example of how a reinforcement learning agent could interface with ROS 2:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class RLNavigationAgent(Node):
    def __init__(self):
        super().__init__('rl_navigation_agent')
        
        # Publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Timer for decision making
        self.timer = self.create_timer(0.2, self.rl_decision_loop)
        
        # RL agent parameters
        self.state = None
        self.previous_action = None
        self.reward = 0.0
        
        # Initialize simple Q-table (for demonstration)
        self.q_table = np.zeros((10, 5))  # 10 states, 5 actions
        self.learning_rate = 0.1
        self.discount_factor = 0.9
        self.exploration_rate = 0.1

    def scan_callback(self, msg):
        """Convert laser scan to discrete state representation"""
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=10.0)  # Replace NaN with max range
        
        # Discretize the state based on distance readings
        front_avg = np.mean(ranges[330:390])
        left_avg = np.mean(ranges[60:120])
        right_avg = np.mean(ranges[240:300])
        
        # Create discrete state (simplified)
        state_idx = 0
        if front_avg < 0.5:
            state_idx = 1  # Very close obstacle
        elif front_avg < 1.0:
            state_idx = 2  # Close obstacle
        elif front_avg < 2.0:
            state_idx = 3  # Medium distance
        else:
            state_idx = 4  # Clear path
            
        if left_avg < 0.5:
            state_idx += 5  # Left too close
            
        self.state = state_idx

    def rl_decision_loop(self):
        """RL decision making process"""
        if self.state is None:
            return
            
        # Choose action using epsilon-greedy policy
        if np.random.random() < self.exploration_rate:
            action = np.random.choice(5)  # Explore
        else:
            action = np.argmax(self.q_table[self.state])  # Exploit
            
        # Convert action to robot command
        cmd_vel = self.action_to_cmd_vel(action)
        
        # Publish command
        self.cmd_vel_publisher.publish(cmd_vel)
        
        # Store action for next iteration
        self.previous_action = action

    def action_to_cmd_vel(self, action):
        """Map discrete action to Twist command"""
        msg = Twist()
        
        if action == 0:  # Move forward
            msg.linear.x = 0.5
            msg.angular.z = 0.0
        elif action == 1:  # Turn slightly left
            msg.linear.x = 0.3
            msg.angular.z = 0.3
        elif action == 2:  # Turn slightly right
            msg.linear.x = 0.3
            msg.angular.z = -0.3
        elif action == 3:  # Turn left
            msg.linear.x = 0.1
            msg.angular.z = 0.8
        elif action == 4:  # Turn right
            msg.linear.x = 0.1
            msg.angular.z = -0.8
            
        return msg

def main(args=None):
    rclpy.init(args=args)
    rl_agent = RLNavigationAgent()
    
    try:
        rclpy.spin(rl_agent)
    except KeyboardInterrupt:
        pass
    finally:
        rl_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Example Command → Robot Movement

Here's a complete example showing how an AI agent can interpret high-level commands and convert them to robot movements:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json

class CommandInterpreter(Node):
    def __init__(self):
        super().__init__('command_interpreter')
        
        # Publisher for robot movement
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for high-level commands
        self.command_subscriber = self.create_subscription(
            String,
            '/ai_commands',
            self.command_callback,
            10
        )
        
        self.get_logger().info('Command interpreter initialized')

    def command_callback(self, msg):
        """Process high-level commands from AI"""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command', '')
            
            if command == 'move_forward':
                self.move_forward(command_data.get('distance', 1.0))
            elif command == 'turn_left':
                self.turn_left(command_data.get('angle', 90))
            elif command == 'turn_right':
                self.turn_right(command_data.get('angle', 90))
            elif command == 'stop':
                self.stop_robot()
            else:
                self.get_logger().warning(f'Unknown command: {command}')
                
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON command: {msg.data}')

    def move_forward(self, distance):
        """Move robot forward by specified distance"""
        msg = Twist()
        msg.linear.x = 0.5  # Speed
        # In a real implementation, you'd integrate over time to achieve distance
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f'Moving forward {distance} meters')

    def turn_left(self, angle):
        """Turn robot left by specified angle"""
        msg = Twist()
        msg.angular.z = 0.5  # Angular speed
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f'Turning left {angle} degrees')

    def turn_right(self, angle):
        """Turn robot right by specified angle"""
        msg = Twist()
        msg.angular.z = -0.5  # Angular speed
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f'Turning right {angle} degrees')

    def stop_robot(self):
        """Stop robot movement"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info('Robot stopped')

def main(args=None):
    rclpy.init(args=args)
    interpreter = CommandInterpreter()
    
    try:
        rclpy.spin(interpreter)
    except KeyboardInterrupt:
        pass
    finally:
        interpreter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for AI-Agent Integration

1. **State Management**: Keep track of the robot's state and environment to make informed decisions
2. **Safety Checks**: Always implement safety mechanisms to prevent dangerous robot behavior
3. **Modular Design**: Separate perception, decision-making, and action components
4. **Error Handling**: Handle sensor failures and communication issues gracefully
5. **Performance**: Optimize AI algorithms for real-time execution

## Summary

In this chapter, we explored how to connect AI agents with ROS 2 robot controllers:
- Using publishers to send commands from AI agents
- Using subscribers to receive sensor data for AI decision-making
- Implementing different AI approaches (rule-based, reinforcement learning)
- Converting high-level commands to robot movements

These techniques enable sophisticated robot behaviors driven by artificial intelligence, forming the core of autonomous robotic systems.