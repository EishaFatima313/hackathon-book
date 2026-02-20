---
sidebar_position: 2
---

# Chapter 2: Digital Twin in Unity

## Overview
Welcome to Chapter 2! In this chapter, we'll explore Unity, a powerful game engine that we'll use to create stunning visualizations of our robot simulations. Unity will serve as the "digital twin" - a visual representation that mirrors what's happening in Gazebo. Think of Unity as the "pretty face" that shows people what your robot is doing!

## What is Unity?
Unity is a cross-platform game engine that's perfect for creating:
- Realistic 3D visualizations
- Interactive environments
- VR/AR experiences
- Digital twins of physical systems

### Why Use Unity for Digital Twins?
- **High-Quality Graphics**: Beautiful, realistic visuals
- **Real-Time Rendering**: Instant visual feedback
- **Flexible Environment**: Create any world you imagine
- **Easy Integration**: Connect with external systems (like ROS2)

## Setting Up Unity for Robotics

### Prerequisites
Before starting, make sure you have:
- Unity Hub installed
- Unity Editor (version 2021.3 LTS or newer)
- ROS# (ROS bridge for Unity) package
- ROS2 running on your system

### Installing ROS# Package
ROS# is a Unity package that enables communication between Unity and ROS2:
1. Download the ROS# Unity package
2. Import it into your Unity project
3. Configure the connection settings

## Creating Your First Unity Robot Visualization

### Step 1: Create a New Unity Project
1. Open Unity Hub
2. Click "New Project"
3. Select "3D (Built-in Render Pipeline)"
4. Name your project "RobotDigitalTwin"

### Step 2: Import Robot Assets
1. Create a folder called "Robots" in your Assets
2. Import your robot model (FBX, OBJ, or similar format)
3. Position the model at the origin (0,0,0)

### Step 3: Set Up the Scene
1. Create a plane for the floor
2. Add lighting (Directional Light)
3. Position the Main Camera to view your robot

## Understanding Digital Twin Architecture

### The Bridge Concept
A digital twin connects the physical/virtual world to a visual representation:
- **Physical World**: Real robot or Gazebo simulation
- **Data Layer**: ROS2 topics carrying sensor/joint data
- **Visualization**: Unity showing the robot state

### Data Flow
```
Gazebo Robot → ROS2 Topics → Unity ROS# → Unity Robot Model
     ↑                                    ↓
     └─────────────────────────────────────┘
           Synchronized Movement
```

## Connecting Unity to ROS2

### ROS# Setup
1. Add ROSConnectionManager to your scene
2. Configure IP address and port (usually 10000)
3. Create publishers/subscribers for robot data

### Example Unity Script for Joint Control
```csharp
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosSharp.Messages.Sensor;

public class JointController : MonoBehaviour
{
    public string jointName;
    private float currentAngle = 0f;
    private Transform jointTransform;

    void Start()
    {
        jointTransform = transform;
        // Subscribe to joint state topic
        RosConnector.Instance.Subscribe<JointState>("/joint_states", OnJointStateReceived);
    }

    void OnJointStateReceived(JointState jointState)
    {
        int index = System.Array.IndexOf(jointState.name, jointName);
        if (index >= 0 && index < jointState.position.Length)
        {
            currentAngle = jointState.position[index];
            UpdateJointPosition();
        }
    }

    void UpdateJointPosition()
    {
        // Rotate the joint based on received angle
        jointTransform.localRotation = Quaternion.Euler(0, currentAngle * Mathf.Rad2Deg, 0);
    }
}
```

## Creating a Robot Prefab

### What is a Prefab?
A prefab is a reusable robot model that can be instantiated multiple times:
- Contains all robot parts and joints
- Includes scripts for ROS communication
- Can be customized for different robots

### Creating Your Robot Prefab
1. Create an empty GameObject named "Robot"
2. Add child objects for each robot part
3. Attach joint controllers to movable parts
4. Save as prefab by dragging to Assets folder

## Synchronizing Robot States

### Joint Positions
To keep Unity synchronized with Gazebo:
1. Subscribe to `/joint_states` topic
2. Update Unity robot joints based on received values
3. Handle coordinate system differences (ROS uses right-handed, Unity uses left-handed)

### Coordinate System Conversion
```csharp
// Convert from ROS to Unity coordinates
Vector3 RosToUnity(Vector3 rosPos)
{
    return new Vector3(rosPos.x, rosPos.z, rosPos.y);
}

Quaternion RosToUnity(Quaternion rosRot)
{
    return new Quaternion(rosRot.x, rosRot.z, rosRot.y, -rosRot.w);
}
```

## Practice Task 1: Basic Unity Robot
1. Create a simple robot model in Unity (cube body, cylinder wheels)
2. Add ROS# connection to your scene
3. Write a script to move the robot based on ROS messages
4. Test the connection with a simple publisher

## Advanced Visualization Features

### Camera Systems
Unity offers multiple camera options:
- **Fixed Cameras**: Static views of the robot
- **Follow Cameras**: Track robot movement
- **Orbit Cameras**: Rotate around the robot
- **Sensor Cameras**: Show what robot sensors see

### Lighting Effects
Enhance your digital twin with:
- Dynamic shadows
- Environmental lighting
- Special effects for sensors
- Day/night cycles

### Particle Systems
Visualize robot activities:
- Dust clouds when moving
- Laser beams for LiDAR
- Smoke effects for thrusters
- Trail effects for movement

## Practice Task 2: Enhanced Visualization
1. Add multiple cameras to view your robot from different angles
2. Implement a follow camera that tracks robot movement
3. Add particle effects to visualize robot actions
4. Create a UI panel showing robot status

## Unity-ROS2 Communication Patterns

### Publisher Pattern
Unity sends data to ROS2:
- Robot pose updates
- Simulation status
- Custom events

### Subscriber Pattern
Unity receives data from ROS2:
- Joint positions
- Sensor readings
- Control commands

### Service Calls
Unity requests specific actions:
- Reset simulation
- Change world
- Load new robot

## Best Practices for Digital Twins

### Performance Optimization
- Use object pooling for frequently created objects
- Optimize meshes and textures
- Limit the number of active cameras
- Use Level of Detail (LOD) for distant objects

### Accuracy
- Maintain scale accuracy between Gazebo and Unity
- Synchronize timing between systems
- Validate sensor data representations
- Test with real robot data when possible

### User Experience
- Provide intuitive controls
- Clear visualization of robot state
- Responsive interface
- Helpful annotations and labels

## Troubleshooting Common Issues

### Connection Problems
- Check IP addresses and ports
- Verify firewall settings
- Ensure ROS2 network setup is correct
- Test with simple ping messages first

### Synchronization Issues
- Check time synchronization between systems
- Verify coordinate system conversions
- Monitor for dropped messages
- Adjust update rates if needed

## Summary
In this chapter, you learned:
- How to set up Unity for robotics applications
- How to create robot visualizations in Unity
- How to connect Unity to ROS2 using ROS#
- How to synchronize robot states between systems
- Best practices for digital twin development

## Next Steps
In the next chapter, we'll focus on sensor simulation and testing, connecting the sensors in Gazebo to their visualizations in Unity!