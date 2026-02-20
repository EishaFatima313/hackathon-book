---
sidebar_position: 3
---

# Humanoid Structure with URDF

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its links, joints, inertial properties, and sensors. URDF is essential for simulating robots in environments like Gazebo and for visualizing them in RViz.

## Basic URDF Concepts

### Links
Links represent rigid bodies in a robot. Each link has:
- Visual properties (shape, color, mesh)
- Collision properties (collision geometry)
- Inertial properties (mass, center of mass, inertia tensor)

### Joints
Joints connect links and define how they can move relative to each other. Common joint types include:
- **Fixed**: No movement between links
- **Revolute**: Rotational movement around an axis
- **Continuous**: Unlimited rotational movement
- **Prismatic**: Linear sliding movement
- **Floating**: 6 degrees of freedom
- **Planar**: Movement in a plane

### Sensors
URDF can also define sensors attached to the robot, specifying their type, position, and properties.

## Simple Humanoid URDF Example

Here's a basic URDF model for a humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.3 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.4"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.04"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.15"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Arm -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.04"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.15"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.06"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.4"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <link name="left_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.35"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Right Leg -->
  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.06"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.4"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <link name="right_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.35"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Joint Definitions -->
  <!-- Torso connected to base -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
  </joint>

  <!-- Head connected to torso -->
  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <!-- Left Arm Joints -->
  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Right Arm Joints -->
  <joint name="torso_to_right_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <joint name="right_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Left Leg Joints -->
  <joint name="torso_to_left_hip" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.07 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <joint name="left_knee" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Right Leg Joints -->
  <joint name="torso_to_right_hip" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.07 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <joint name="right_knee" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Visualizing Robot Model

### Using RViz

RViz is ROS's 3D visualization tool that can display your URDF model:

1. Launch RViz:
```bash
rviz2
```

2. Load your URDF file using the RobotModel plugin

3. Set the Fixed Frame to your robot's base frame (e.g., `base_link`)

### Using Gazebo Simulation

To simulate your humanoid robot in Gazebo:

1. Create a launch file to spawn the robot:
```xml
<launch>
  <param name="robot_description" command="$(find xacro)/xacro $(find your_package)/urdf/humanoid.urdf.xacro" />
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_entity.py" args="-entity humanoid -file $(find your_package)/urdf/humanoid.urdf" />
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>
</launch>
```

## Advanced URDF Features

### Transmission Elements

For controlling joints with actuators, add transmission elements:

```xml
<transmission name="left_shoulder_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="torso_to_left_shoulder">
    <hardwareInterface>PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_shoulder_motor">
    <hardwareInterface>PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Gazebo-Specific Elements

Add Gazebo-specific properties:

```xml
<gazebo reference="left_upper_arm">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>
```

### Xacro for Complex Models

Xacro (XML Macros) helps create more maintainable URDF files:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="arm_length" value="0.2" />
  <xacro:property name="leg_length" value="0.3" />

  <!-- Macro for arm -->
  <xacro:macro name="arm" params="prefix side">
    <link name="${prefix}_upper_arm">
      <visual>
        <geometry>
          <cylinder length="${arm_length}" radius="0.05"/>
        </geometry>
        <material name="green">
          <color rgba="0 1 0 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder length="${arm_length}" radius="0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.2"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>

    <link name="${prefix}_lower_arm">
      <visual>
        <geometry>
          <cylinder length="${arm_length}" radius="0.04"/>
        </geometry>
        <material name="green">
          <color rgba="0 1 0 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder length="${arm_length}" radius="0.04"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.15"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>

    <joint name="torso_to_${side}_shoulder" type="revolute">
      <parent link="torso"/>
      <child link="${prefix}_upper_arm"/>
      <origin xyz="${prefix == 'left' and '0.15' or '-0.15'} 0 0.1" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    </joint>

    <joint name="${side}_elbow" type="revolute">
      <parent link="${prefix}_upper_arm"/>
      <child link="${prefix}_lower_arm"/>
      <origin xyz="0 0 -${arm_length}" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro to create arms -->
  <xacro:arm prefix="left" side="left"/>
  <xacro:arm prefix="right" side="right"/>

</robot>
```

## Working with URDF in ROS 2

### Loading URDF Parameters

In ROS 2, you typically load the URDF as a parameter:

```python
import rclpy
from rclpy.node import Node
import xacro
from ament_index_python.packages import get_package_share_path

class URDFLoader(Node):
    def __init__(self):
        super().__init__('urdf_loader')
        
        # Load URDF from file
        urdf_path = get_package_share_path('your_package') / 'urdf' / 'humanoid.urdf'
        self.robot_description = xacro.process_file(str(urdf_path)).toprettyxml(indent='  ')
        
        # Set as parameter
        self.declare_parameter('robot_description', self.robot_description)
        
        self.get_logger().info('URDF loaded successfully')

def main(args=None):
    rclpy.init(args=args)
    loader = URDFLoader()
    
    try:
        rclpy.spin(loader)
    except KeyboardInterrupt:
        pass
    finally:
        loader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Robot State Publisher

Use robot_state_publisher to publish transforms based on joint states:

```xml
<node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
  <param name="robot_description" value="$(var robot_description)"/>
</node>
```

## Best Practices for URDF Design

1. **Realistic Inertial Properties**: Accurate mass and inertia values are crucial for realistic simulation
2. **Collision vs Visual**: Use simpler geometries for collision detection than for visual representation
3. **Consistent Naming**: Use consistent naming conventions for links and joints
4. **Joint Limits**: Always specify appropriate joint limits to prevent damage in simulation
5. **Xacro for Complex Models**: Use Xacro macros to avoid repetition in complex models
6. **Testing**: Test your URDF in RViz and Gazebo to ensure it displays correctly

## Summary

In this chapter, we covered:
- The fundamentals of URDF (Unified Robot Description Format)
- How to define links and joints for a humanoid robot
- Creating a complete humanoid model with arms, legs, and torso
- Visualization techniques using RViz and Gazebo
- Advanced features like transmissions and Xacro macros
- Best practices for URDF design

URDF is essential for representing robots in ROS, enabling simulation, visualization, and control of complex robotic systems like humanoids.