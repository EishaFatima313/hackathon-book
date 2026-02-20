---
sidebar_position: 4
---

# Module 4: Vision-Language-Action (VLA)

## Introduction

Welcome to Module 4! In this module, you will learn how robots can:

- **See** the world using cameras
- **Understand** human speech and commands
- **Act** to complete tasks

This combination is called **Vision-Language-Action** or **VLA** for short.

By the end of this module, you will build a capstone project: a humanoid robot that listens to your voice, plans its actions, navigates a room, finds an object, and picks it up!

---

## 1. What is VLA?

### The Three Parts of VLA

**VLA** stands for **Vision-Language-Action**. It is how smart robots understand and interact with the world.

Think of VLA like a human:

| Part | What It Does | Like a Human |
|------|--------------|--------------|
| **Vision** | Sees objects, people, and places | Eyes |
| **Language** | Understands words and commands | Ears + Brain |
| **Action** | Moves and does tasks | Hands + Legs |

### How VLA Works Together

```
[Human speaks] → [Robot hears] → [Robot understands] → [Robot sees] → [Robot acts]
   "Pick up      [Speech-to-     [LLM plans          [Camera        [Motors
    the cup"]      text]           steps]              detects]       move]
```

### Why VLA Matters

Before VLA, robots could only do pre-programmed tasks. They could not understand new commands or adapt to new situations.

With VLA, robots can:

- Understand new commands they never heard before
- Work in rooms they have never seen
- Handle objects in different positions
- Learn from mistakes

### Simple Example

Imagine you say: **"Bring me the red ball"**

A VLA robot:

1. **Hears** your words (Language)
2. **Understands** it needs to find a red ball and bring it (Language + Planning)
3. **Looks** around for something red and round (Vision)
4. **Moves** to the ball, grabs it, and brings it to you (Action)

---

## 2. Voice-to-Action: Speech-to-Text

### How Robots Hear

Robots do not have ears like humans. Instead, they use:

- A **microphone** to capture sound
- A **speech-to-text** system to convert sound to words

### Speech-to-Text Basics

**Speech-to-text** (also called **STT**) converts spoken words into text that a computer can understand.

```
[You speak] → [Microphone] → [Audio signal] → [STT System] → [Text]
"Move forward"                    (sound waves)              "move forward"
```

### Using Speech Recognition in Python

Python has easy-to-use speech recognition libraries. Here is a simple example:

```python
import speech_recognition as sr

def listen_to_command():
    """Listen for voice command and return text"""
    recognizer = sr.Recognizer()
    
    with sr.Microphone() as source:
        print("Listening...")
        # Adjust for ambient noise
        recognizer.adjust_for_ambient_noise(source)
        
        # Listen for audio
        audio = recognizer.listen(source)
        
        try:
            # Convert speech to text using Google's free API
            command = recognizer.recognize_google(audio)
            print(f"You said: {command}")
            return command.lower()
        except sr.UnknownValueError:
            print("Sorry, I did not understand")
            return None
        except sr.RequestError:
            print("Could not connect to speech service")
            return None

# Test the function
if __name__ == "__main__":
    command = listen_to_command()
    if command:
        print(f"Received command: {command}")
```

### Installing Speech Recognition

To use speech recognition, install the required packages:

```bash
pip install SpeechRecognition
pip install pyaudio
```

### Common Voice Commands for Robots

Here are some simple commands your robot might understand:

| Command | What Robot Does |
|---------|-----------------|
| "Move forward" | Drives forward |
| "Turn left" | Rotates left |
| "Stop" | Halts all motion |
| "Find the cup" | Looks for a cup |
| "Pick it up" | Grabs the object |
| "Bring it to me" | Returns with object |

### From Text to Action

Once the robot has the text, it needs to understand what to do. This is where **Large Language Models (LLMs)** come in.

---

## 3. LLM: Converting Commands to Robot Steps

### What is an LLM?

**LLM** stands for **Large Language Model**. It is an AI that understands and generates human language.

Examples of LLMs:
- GPT-4
- Claude
- Llama
- Gemini

### How LLMs Help Robots

When you say **"Clean the room"**, the LLM breaks it down into steps:

```
[Input] "Clean the room"
   ↓
[LLM thinks about what cleaning means]
   ↓
[Output] Steps:
1. Find trash on the floor
2. Pick up each piece of trash
3. Put trash in the bin
4. Find dirty dishes
5. Pick up dishes
6. Take dishes to kitchen
```

### Using an LLM with Your Robot

Here is how to use an LLM to convert commands into robot actions:

```python
import openai

def command_to_steps(command):
    """Use LLM to convert command into robot steps"""
    
    # Create a prompt for the LLM
    prompt = f"""
    Convert this robot command into simple steps.
    Each step should be one action the robot can do.
    
    Command: "{command}"
    
    Steps (use format: 1. [action]):
    """
    
    # Call the LLM (using OpenAI as example)
    response = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=[
            {"role": "system", "content": "You are a robot planning assistant."},
            {"role": "user", "content": prompt}
        ]
    )
    
    # Get the steps from the response
    steps_text = response.choices[0].message.content
    return steps_text

# Example usage
command = "Clean the room"
steps = command_to_steps(command)
print(steps)
```

### Example Output

For the command **"Clean the room"**, the LLM might output:

```
1. Look around the room for objects on the floor
2. Move to the nearest object
3. Identify if the object is trash
4. If trash, pick it up
5. Move to the trash bin
6. Drop the trash in the bin
7. Repeat until no more trash is found
```

### Mapping Steps to ROS 2 Actions

Each step needs to become a ROS 2 command:

| LLM Step | ROS 2 Action |
|----------|--------------|
| "Look around" | Rotate camera, publish to `/camera` topic |
| "Move to object" | Publish velocity to `/cmd_vel` topic |
| "Pick it up" | Call `/arm/grasp` service |
| "Drop in bin" | Call `/arm/release` service |

### Complete Voice-to-Steps Example

```python
import speech_recognition as sr
import openai
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VoiceCommandRobot(Node):
    def __init__(self):
        super().__init__('voice_command_robot')
        
        # Publisher for robot movement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Speech recognizer
        self.recognizer = sr.Recognizer()
        
    def listen_and_act(self):
        """Main loop: listen, understand, act"""
        
        # Step 1: Listen to voice command
        with sr.Microphone() as source:
            self.get_logger().info("Listening for command...")
            audio = self.recognizer.listen(source)
            
        try:
            # Step 2: Convert speech to text
            command = self.recognizer.recognize_google(audio)
            self.get_logger().info(f"Command: {command}")
            
            # Step 3: Use LLM to get steps
            steps = self.get_steps_from_llm(command)
            self.get_logger().info(f"Steps: {steps}")
            
            # Step 4: Execute each step
            self.execute_steps(steps)
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
    
    def get_steps_from_llm(self, command):
        """Get action steps from LLM"""
        # Simplified - in real use, call actual LLM API
        simple_commands = {
            "move forward": ["move_forward"],
            "turn left": ["turn_left"],
            "stop": ["stop"],
        }
        return simple_commands.get(command.lower(), ["unknown"])
    
    def execute_steps(self, steps):
        """Execute each step"""
        for step in steps:
            if step == "move_forward":
                self.move_forward()
            elif step == "turn_left":
                self.turn_left()
            elif step == "stop":
                self.stop()
    
    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.5
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info("Moving forward")
    
    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.5
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info("Turning left")
    
    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info("Stopped")

def main():
    rclpy.init()
    robot = VoiceCommandRobot()
    robot.listen_and_act()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

---

## 4. Computer Vision: Detecting Objects

### How Robots See

Robots use **cameras** to see the world. But seeing is not enough - the robot must **understand** what it sees.

This is called **Computer Vision**.

### Object Detection

**Object Detection** means finding and naming objects in an image.

```
[Camera Image] → [Object Detection AI] → [Objects Found]
                                          ↓
                                    "cup at x=100, y=200"
                                    "chair at x=300, y=150"
                                    "person at x=500, y=400"
```

### Using YOLO for Object Detection

**YOLO** (You Only Look Once) is a popular, fast object detection system.

```python
import cv2
from ultralytics import YOLO

# Load a pre-trained YOLO model
model = YOLO('yolov8n.pt')  # 'n' = nano (smallest, fastest)

def detect_objects(image_path):
    """Detect objects in an image"""
    
    # Load the image
    image = cv2.imread(image_path)
    
    # Run detection
    results = model(image)
    
    # Get detections
    detections = results[0].boxes
    
    # Print results
    for detection in detections:
        class_id = int(detection.cls[0])
        class_name = model.names[class_id]
        confidence = float(detection.conf[0])
        bbox = detection.xyxy[0].tolist()  # [x1, y1, x2, y2]
        
        print(f"Found: {class_name} ({confidence:.2f}) at {bbox}")
    
    return detections

# Example usage
detections = detect_objects('room_photo.jpg')
```

### Installing YOLO

```bash
pip install ultralytics
pip install opencv-python
```

### Common Objects YOLO Can Detect

YOLO can detect 80 common objects:

| Category | Objects |
|----------|---------|
| People | person |
| Vehicles | car, bicycle, motorcycle, bus, truck |
| Animals | bird, cat, dog, horse |
| Furniture | chair, couch, bed, dining table |
| Items | cup, bottle, book, cell phone, laptop |

### Getting Object Positions

For a robot to interact with objects, it needs to know **where** they are in 3D space.

```python
import numpy as np

def pixel_to_robot_coordinates(pixel_x, pixel_y, depth_value, camera_info):
    """Convert pixel position to robot coordinates"""
    
    # Camera intrinsics (from camera calibration)
    fx = camera_info['fx']  # Focal length x
    fy = camera_info['fy']  # Focal length y
    cx = camera_info['cx']  # Principal point x
    cy = camera_info['cy']  # Principal point y
    
    # Convert to 3D coordinates
    z = depth_value  # Distance from camera
    x = (pixel_x - cx) * z / fx
    y = (pixel_y - cy) * z / fy
    
    return [x, y, z]

# Example
camera_info = {'fx': 500, 'fy': 500, 'cx': 320, 'cy': 240}
robot_pos = pixel_to_robot_coordinates(400, 300, 2.0, camera_info)
print(f"Object is at: {robot_pos} meters from robot")
```

### Vision Pipeline for Robot

Here is the complete vision pipeline:

```python
import cv2
from ultralytics import YOLO
import numpy as np

class RobotVision:
    def __init__(self):
        # Load object detection model
        self.model = YOLO('yolov8n.pt')
        
        # Camera info (from calibration)
        self.camera_info = {
            'fx': 500, 'fy': 500,
            'cx': 320, 'cy': 240
        }
    
    def find_object(self, target_class, image, depth_image):
        """Find a specific object and return its 3D position"""
        
        # Detect objects
        results = self.model(image)
        detections = results[0].boxes
        
        # Search for target object
        for detection in detections:
            class_id = int(detection.cls[0])
            class_name = self.model.names[class_id]
            
            if class_name == target_class:
                # Get bounding box center
                bbox = detection.xyxy[0].tolist()
                center_x = int((bbox[0] + bbox[2]) / 2)
                center_y = int((bbox[1] + bbox[3]) / 2)
                
                # Get depth at center
                depth = depth_image[center_y, center_x]
                
                # Convert to 3D coordinates
                position = self.pixel_to_3d(
                    center_x, center_y, depth
                )
                
                return {
                    'found': True,
                    'name': class_name,
                    'position': position,
                    'bbox': bbox
                }
        
        return {'found': False}
    
    def pixel_to_3d(self, px, py, depth):
        """Convert pixel to 3D robot coordinates"""
        fx = self.camera_info['fx']
        fy = self.camera_info['fy']
        cx = self.camera_info['cx']
        cy = self.camera_info['cy']
        
        z = depth
        x = (px - cx) * z / fx
        y = (py - cy) * z / fy
        
        return [x, y, z]

# Example usage
vision = RobotVision()
image = cv2.imread('room.jpg')
depth = np.load('depth.npy')

result = vision.find_object('cup', image, depth)
if result['found']:
    print(f"Cup found at: {result['position']}")
else:
    print("Cup not found")
```

---

## 5. ROS 2 Nodes and Actions

### ROS 2 Nodes Recap

A **Node** is a program that does one job. Robots use many nodes working together.

```
[Camera Node] ──────→ [Vision Node] ────→ [Action Node]
     ↓                      ↓                    ↓
  publishes              processes            controls
  images                 objects              motors
```

### ROS 2 Actions

**Actions** are like services but for long-running tasks. They have:

- **Goal**: What to do
- **Feedback**: Progress updates
- **Result**: Final outcome

### Action Example: Navigate to Position

```python
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time

class NavigationAction(Node):
    def __init__(self):
        super().__init__('navigation_action')
        
        # Create action server
        self.action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.navigate_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
    
    def goal_callback(self, goal_request):
        """Accept or reject goal"""
        self.get_logger().info("Received navigation goal")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Handle cancel request"""
        self.get_logger().info("Canceling navigation")
        return CancelResponse.ACCEPT
    
    async def navigate_callback(self, goal_handle):
        """Execute navigation"""
        self.get_logger().info("Executing navigation")
        
        # Get target position
        target = goal_handle.request.pose.pose.position
        
        feedback_msg = NavigateToPose.Feedback()
        
        # Simple navigation loop
        while True:
            # Check if canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.stop_robot()
                return NavigateToPose.Result()
            
            # Get current position (simplified)
            current_x = self.get_current_position()
            
            # Check if arrived
            distance = abs(target.x - current_x)
            if distance < 0.1:  # Within 10 cm
                break
            
            # Move toward target
            self.move_toward_target(target)
            
            # Send feedback
            feedback_msg.distance_remaining = distance
            goal_handle.publish_feedback(feedback_msg)
            
            # Wait a bit
            await asyncio.sleep(0.1)
        
        # Success
        goal_handle.succeed()
        result = NavigateToPose.Result()
        result.success = True
        return result
    
    def get_current_position(self):
        """Get robot's current position"""
        # In real robot, get from odometry
        return type('obj', (object,), {'x': 0})()
    
    def move_toward_target(self, target):
        """Move robot toward target"""
        msg = Twist()
        msg.linear.x = 0.5
        self.cmd_vel_pub.publish(msg)
    
    def stop_robot(self):
        """Stop robot"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

def main():
    rclpy.init()
    nav_action = NavigationAction()
    rclpy.spin(nav_action)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

### Creating an Action Client

The action client sends goals to the action server:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        
        # Create action client
        self.action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
    
    def send_goal(self, x, y, z):
        """Send navigation goal"""
        
        # Wait for server
        self.action_client.wait_for_server()
        
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        
        # Send goal
        self.get_logger().info(f"Sending goal: ({x}, {y}, {z})")
        future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
        
        self.get_logger().info("Goal accepted")
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Handle feedback"""
        distance = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f"Distance remaining: {distance:.2f}m")
    
    def result_callback(self, future):
        """Handle final result"""
        result = future.result().result
        self.get_logger().info(f"Navigation complete: {result.success}")

def main():
    rclpy.init()
    client = NavigationClient()
    
    # Send a goal
    client.send_goal(1.0, 0.0, 0.0)
    
    rclpy.spin(client)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

### Putting It All Together: VLA Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    VLA System Architecture                   │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐              │
│  │  Voice   │───→│   LLM    │───→│  Vision  │              │
│  │  Input   │    │  Planner │    │  System  │              │
│  └──────────┘    └──────────┘    └──────────┘              │
│       ↓                ↓                ↓                   │
│  Speech-to-       Command          Object                  │
│  Text (STT)       Steps            Detection               │
│                                      ↓                     │
│                              ┌──────────────┐              │
│                              │  ROS 2       │              │
│                              │  Actions     │              │
│                              └──────────────┘              │
│                                      ↓                     │
│                              ┌──────────────┐              │
│                              │   Robot      │              │
│                              │   Hardware   │              │
│                              └──────────────┘              │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

---

## 6. Capstone Project: Humanoid Robot Assistant

### Project Overview

In this capstone, you will create a **humanoid robot** that:

1. **Listens** to voice commands
2. **Plans** actions using an LLM
3. **Navigates** to objects
4. **Detects** objects with vision
5. **Picks up** the object

### System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│              Humanoid Robot Assistant                        │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  1. Voice Command                                            │
│     "Pick up the cup"                                        │
│           ↓                                                  │
│  2. Speech-to-Text                                           │
│     Text: "pick up the cup"                                  │
│           ↓                                                  │
│  3. LLM Planning                                             │
│     Steps: [find cup, navigate to cup, pick cup]             │
│           ↓                                                  │
│  4. Vision System                                            │
│     Detects: cup at position (1.5, 0.3, 0.8)                 │
│           ↓                                                  │
│  5. Navigation                                               │
│     Move to position (1.5, 0.3)                              │
│           ↓                                                  │
│  6. Arm Control                                              │
│     Reach, grasp, lift                                       │
│           ↓                                                  │
│  7. Task Complete!                                           │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### Complete Code

Here is the complete VLA humanoid robot code:

```python
"""
VLA Humanoid Robot Assistant
Capstone Project for Module 4

This robot can:
1. Listen to voice commands
2. Understand commands using LLM
3. Find objects with computer vision
4. Navigate to objects
5. Pick up objects with its arm
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import speech_recognition as sr
from ultralytics import YOLO
import cv2
import numpy as np
import asyncio


class SpeechListener(Node):
    """Node 1: Listen to voice commands"""
    
    def __init__(self):
        super().__init__('speech_listener')
        
        # Speech recognizer
        self.recognizer = sr.Recognizer()
        
        # Publisher for recognized commands
        self.command_pub = self.create_publisher(
            String, '/voice_command', 10
        )
        
        # Timer for listening (can be triggered by button in real robot)
        self.timer = self.create_timer(5.0, self.listen_loop)
        
        self.listening = True
    
    def listen_loop(self):
        """Continuously listen for commands"""
        if not self.listening:
            return
        
        try:
            with sr.Microphone() as source:
                self.get_logger().info("Listening...")
                self.recognizer.adjust_for_ambient_noise(source)
                audio = self.recognizer.listen(source, timeout=3)
                
                command = self.recognizer.recognize_google(audio)
                self.get_logger().info(f"Heard: {command}")
                
                # Publish command
                msg = String()
                msg.data = command.lower()
                self.command_pub.publish(msg)
                
        except sr.WaitTimeoutError:
            pass  # No speech detected
        except sr.UnknownValueError:
            self.get_logger().info("Could not understand speech")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


class LLMPlanner(Node):
    """Node 2: Convert commands to action steps using LLM"""
    
    def __init__(self):
        super().__init__('llm_planner')
        
        # Subscriber for voice commands
        self.command_sub = self.create_subscription(
            String, '/voice_command', self.command_callback, 10
        )
        
        # Publisher for planned steps
        self.steps_pub = self.create_publisher(
            String, '/planned_steps', 10
        )
        
        # Simple command mapping (replace with real LLM API)
        self.command_map = {
            "pick up the cup": "1.find_cup 2.navigate_to_cup 3.pick_object",
            "bring me the bottle": "1.find_bottle 2.navigate_to_bottle 3.pick_object 4.return_to_user",
            "clean the room": "1.find_trash 2.navigate_to_trash 3.pick_object 4.go_to_bin 5.release_object",
            "move forward": "1.move_forward",
            "turn left": "1.turn_left",
            "turn right": "1.turn_right",
            "stop": "1.stop",
        }
    
    def command_callback(self, msg):
        """Process voice command and generate steps"""
        command = msg.data
        self.get_logger().info(f"Planning for: {command}")
        
        # Get steps from LLM (simplified - use real LLM API here)
        steps = self.command_map.get(command, "1.unknown_command")
        
        self.get_logger().info(f"Steps: {steps}")
        
        # Publish steps
        steps_msg = String()
        steps_msg.data = steps
        self.steps_pub.publish(steps_msg)


class VisionSystem(Node):
    """Node 3: Detect objects using computer vision"""
    
    def __init__(self):
        super().__init__('vision_system')
        
        # Load YOLO model
        self.model = YOLO('yolov8n.pt')
        
        # Subscriber for camera images
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        
        # Publisher for detected objects
        self.object_pub = self.create_publisher(
            String, '/detected_objects', 10
        )
        
        # Camera info (from calibration)
        self.camera_info = {
            'fx': 500.0, 'fy': 500.0,
            'cx': 320.0, 'cy': 240.0
        }
        
        # Current target object
        self.target_object = None
        self.object_position = None
    
    def set_target(self, target_name):
        """Set which object to look for"""
        self.target_object = target_name
        self.get_logger().info(f"Looking for: {target_name}")
    
    def image_callback(self, msg):
        """Process camera image"""
        if self.target_object is None:
            return
        
        # Convert ROS image to OpenCV format
        image = self.ros_to_cv2(msg)
        
        # Detect objects
        results = self.model(image)
        detections = results[0].boxes
        
        # Find target object
        for detection in detections:
            class_id = int(detection.cls[0])
            class_name = self.model.names[class_id]
            
            if class_name == self.target_object:
                # Get position
                bbox = detection.xyxy[0].tolist()
                center_x = int((bbox[0] + bbox[2]) / 2)
                center_y = int((bbox[1] + bbox[3]) / 2)
                
                # Estimate depth (simplified - use depth camera in real robot)
                depth = 2.0  # Assume 2 meters for demo
                
                # Convert to 3D
                position = self.pixel_to_3d(center_x, center_y, depth)
                
                self.object_position = position
                
                # Publish detection
                obj_msg = String()
                obj_msg.data = f"{class_name} at {position}"
                self.object_pub.publish(obj_msg)
                
                self.get_logger().info(f"Found {class_name} at {position}")
                return
        
        self.get_logger().info(f"{self.target_object} not found")
    
    def ros_to_cv2(self, ros_image):
        """Convert ROS Image message to OpenCV format"""
        np_arr = np.frombuffer(ros_image.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return image
    
    def pixel_to_3d(self, px, py, depth):
        """Convert pixel coordinates to 3D robot coordinates"""
        fx = self.camera_info['fx']
        fy = self.camera_info['fy']
        cx = self.camera_info['cx']
        cy = self.camera_info['cy']
        
        x = (px - cx) * depth / fx
        y = (py - cy) * depth / fy
        z = depth
        
        return [x, y, z]


class NavigationController(Node):
    """Node 4: Navigate robot to positions"""
    
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Action client for navigation
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        
        # Publisher for simple velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        
        # Subscriber for object positions
        self.object_sub = self.create_subscription(
            String, '/detected_objects', self.object_callback, 10
        )
        
        self.target_position = None
    
    def object_callback(self, msg):
        """Handle detected object position"""
        # Parse position from message
        # Format: "cup at [x, y, z]"
        try:
            parts = msg.data.split(' at ')
            position_str = parts[1].strip('[]')
            position = [float(x) for x in position_str.split(',')]
            self.target_position = position
            self.get_logger().info(f"Target position: {position}")
        except Exception as e:
            self.get_logger().error(f"Could not parse position: {e}")
    
    def navigate_to_object(self):
        """Navigate to the detected object"""
        if self.target_position is None:
            self.get_logger().warn("No target position set")
            return
        
        self.nav_client.wait_for_server()
        
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.pose.position.x = self.target_position[0]
        goal_msg.pose.pose.position.y = self.target_position[1]
        goal_msg.pose.pose.position.z = 0.0
        
        self.get_logger().info("Sending navigation goal")
        self.nav_client.send_goal_async(goal_msg)


class ArmController(Node):
    """Node 5: Control robot arm for picking objects"""
    
    def __init__(self):
        super().__init__('arm_controller')
        
        # Publisher for arm joint commands
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )
        
        # Gripper control
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10
        )
        
        # Arm state
        self.arm_ready = True
    
    def pick_object(self):
        """Execute pick motion"""
        if not self.arm_ready:
            return
        
        self.get_logger().info("Starting pick sequence")
        
        # Step 1: Move arm to pre-pick position
        self.move_to_pre_pick()
        
        # Step 2: Lower arm to object
        self.lower_arm()
        
        # Step 3: Close gripper
        self.close_gripper()
        
        # Step 4: Lift arm
        self.lift_arm()
        
        self.get_logger().info("Pick complete")
    
    def move_to_pre_pick(self):
        """Move arm to position above object"""
        traj = JointTrajectory()
        traj.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3']
        
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.5, -0.5]  # Example joint angles
        point.time_from_start = rclpy.duration.Duration(seconds=2)
        
        traj.points.append(point)
        self.arm_pub.publish(traj)
    
    def lower_arm(self):
        """Lower arm to grasp object"""
        traj = JointTrajectory()
        traj.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3']
        
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.2, -0.3]
        point.time_from_start = rclpy.duration.Duration(seconds=2)
        
        traj.points.append(point)
        self.arm_pub.publish(traj)
    
    def close_gripper(self):
        """Close gripper to grasp object"""
        traj = JointTrajectory()
        traj.joint_names = ['gripper_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [0.0]  # Closed position
        point.time_from_start = rclpy.duration.Duration(seconds=1)
        
        traj.points.append(point)
        self.gripper_pub.publish(traj)
    
    def lift_arm(self):
        """Lift arm with object"""
        traj = JointTrajectory()
        traj.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3']
        
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.8, -0.8]
        point.time_from_start = rclpy.duration.Duration(seconds=2)
        
        traj.points.append(point)
        self.arm_pub.publish(traj)


class VLACoordinator(Node):
    """Main coordinator node that ties all VLA components together"""
    
    def __init__(self):
        super().__init__('vla_coordinator')
        
        # Subscriber for planned steps
        self.steps_sub = self.create_subscription(
            String, '/planned_steps', self.steps_callback, 10
        )
        
        # Create component nodes
        self.vision = VisionSystem()
        self.navigation = NavigationController()
        self.arm = ArmController()
        
        # Current step
        self.current_step = 0
        self.steps = []
    
    def steps_callback(self, msg):
        """Process planned steps and execute them"""
        steps_str = msg.data
        self.steps = steps_str.split()
        self.current_step = 0
        
        self.get_logger().info(f"Executing {len(self.steps)} steps")
        
        # Execute first step
        self.execute_next_step()
    
    def execute_next_step(self):
        """Execute the next step in the plan"""
        if self.current_step >= len(self.steps):
            self.get_logger().info("All steps complete!")
            return
        
        step = self.steps[self.current_step]
        self.get_logger().info(f"Executing: {step}")
        
        # Parse and execute step
        if 'find_' in step:
            object_name = step.replace('find_', '')
            self.vision.set_target(object_name)
            
        elif 'navigate_' in step:
            self.navigation.navigate_to_object()
            
        elif 'pick' in step:
            self.arm.pick_object()
            
        elif 'release' in step:
            self.arm.open_gripper()
            
        elif 'move_forward' in step:
            self.send_velocity(0.5, 0.0)
            
        elif 'turn_left' in step:
            self.send_velocity(0.0, 0.5)
            
        elif 'turn_right' in step:
            self.send_velocity(0.0, -0.5)
            
        elif 'stop' in step:
            self.send_velocity(0.0, 0.0)
        
        # Move to next step after delay (in real robot, wait for completion)
        self.create_timer(2.0, self.increment_step)
    
    def increment_step(self):
        """Move to next step"""
        self.current_step += 1
        self.execute_next_step()
    
    def send_velocity(self, linear, angular):
        """Send velocity command"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.navigation.cmd_vel_pub.publish(msg)


def main():
    """Main entry point for VLA Humanoid Robot"""
    rclpy.init()
    
    # Create all nodes
    speech_listener = SpeechListener()
    llm_planner = LLMPlanner()
    vla_coordinator = VLACoordinator()
    
    # Use multi-threaded executor to run all nodes
    executor = MultiThreadedExecutor()
    executor.add_node(speech_listener)
    executor.add_node(llm_planner)
    executor.add_node(vla_coordinator)
    
    # Spin all nodes
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running the Capstone Project

1. **Install dependencies**:
```bash
pip install SpeechRecognition pyaudio ultralytics opencv-python
pip install rclpy nav2_msgs trajectory_msgs
```

2. **Download YOLO model** (first time only):
```python
from ultralytics import YOLO
model = YOLO('yolov8n.pt')  # Auto-downloads on first run
```

3. **Launch the robot**:
```bash
python vla_humanoid_robot.py
```

4. **Give a voice command**:
- Say: "Pick up the cup"
- The robot will:
  - Hear your command
  - Plan the steps
  - Find the cup with its camera
  - Navigate to the cup
  - Pick it up

### Testing Without Hardware

You can test the system with simulated data:

```python
# Test speech recognition with audio file
audio = sr.AudioFile('test_command.wav')
with audio as source:
    audio_data = recognizer.record(source)
    command = recognizer.recognize_google(audio_data)

# Test vision with image file
image = cv2.imread('test_room.jpg')
results = model(image)

# Test navigation with mock positions
navigation.target_position = [1.5, 0.3, 0.0]
navigation.navigate_to_object()
```

---

## Summary

In this module, you learned about **Vision-Language-Action (VLA)** systems:

| Topic | What You Learned |
|-------|------------------|
| **VLA Basics** | How robots combine vision, language, and action |
| **Speech-to-Text** | Converting voice commands to text |
| **LLM Planning** | Breaking commands into robot steps |
| **Computer Vision** | Detecting objects with YOLO |
| **ROS 2 Actions** | Using actions for long-running tasks |
| **Capstone Project** | A complete humanoid robot assistant |

### Key Takeaways

1. **VLA** combines three abilities: seeing, understanding, and acting
2. **Speech recognition** lets robots hear human commands
3. **LLMs** convert vague commands into specific steps
4. **Computer vision** helps robots find and identify objects
5. **ROS 2 actions** control complex robot behaviors
6. **Integration** of all components creates intelligent robots

### What's Next?

You now have the foundation to build smart, autonomous robots! Try:

- Adding more voice commands
- Training a custom object detector
- Implementing better navigation with SLAM
- Adding obstacle avoidance
- Creating multi-step task planning

Congratulations on completing Module 4!
