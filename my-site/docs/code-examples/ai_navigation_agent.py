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