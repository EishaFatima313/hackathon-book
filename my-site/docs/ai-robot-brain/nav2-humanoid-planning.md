---
sidebar_position: 3
---

# Chapter 3: Nav2 for Humanoid Path Planning

## Overview
Welcome to Chapter 3! In this chapter, we'll explore the Navigation2 (Nav2) framework and how to adapt it for humanoid robots. Navigation2 is ROS2's official navigation framework that enables autonomous navigation for mobile robots. We'll focus specifically on how to configure and customize Nav2 for bipedal humanoid robots, which have unique challenges compared to wheeled robots. Think of this as teaching your humanoid robot how to walk intelligently through complex environments!

## Understanding Navigation2 (Nav2)

### What is Nav2?
Navigation2 is ROS2's state-of-the-art navigation framework that provides:
- **Global Path Planning**: Finding optimal routes from start to goal
- **Local Path Planning**: Avoiding obstacles in real-time
- **Recovery Behaviors**: Handling navigation failures gracefully
- **Behavior Trees**: Flexible task execution framework
- **Map Management**: Handling static and dynamic maps

### Key Components of Nav2
- **Planners**: Global and local path planners
- **Controllers**: Robot motion controllers
- **Sensors**: Integration with various sensors
- **Behaviors**: Recovery and action behaviors
- **Transforms**: Coordinate system management

## Challenges with Humanoid Navigation

### Unique Characteristics of Humanoid Robots
Humanoid robots present unique navigation challenges:
- **Bipedal Locomotion**: Walking motion differs significantly from wheeled robots
- **Balance Constraints**: Must maintain balance during movement
- **Foot Placement**: Precise footstep planning required
- **Center of Mass**: Dynamic center of mass affects stability
- **Step Height**: Limited ability to step over obstacles
- **Turning Radius**: Different turning mechanics than wheeled robots

### Differences from Wheeled Navigation
- **Motion Model**: Humanoids follow different kinematic constraints
- **Collision Checking**: 3D collision detection vs 2D footprint
- **Path Following**: Requires footstep planning, not just trajectory following
- **Speed Control**: Discrete stepping vs continuous motion
- **Terrain Adaptation**: Ability to step over small obstacles

## Nav2 Architecture for Humanoids

### Core Navigation Stack
```
Goal Request → Global Planner → Local Planner → Controller → Robot
                    ↓              ↓            ↓
               Map Server    Costmap Server  Footstep Planner
                    ↓              ↓
               Path Smoother   Obstacle Avoidance
```

### Humanoid-Specific Modifications
- **Custom Motion Model**: Kinematic constraints for bipedal locomotion
- **Footstep Planner**: Generates safe footholds for walking
- **Balance Controller**: Maintains stability during navigation
- **Terrain Analyzer**: Evaluates terrain traversability for humanoids

## Setting Up Nav2 for Humanoid Robots

### Prerequisites
Before configuring Nav2 for humanoid robots, ensure you have:
- ROS2 Humble Hawksbill or later
- Navigation2 packages installed
- Robot state publisher running
- TF tree properly configured
- Sensor data (LiDAR, camera, IMU) available
- Humanoid robot model with proper URDF

### Installation
```bash
# Install Navigation2 packages
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-nav2-rviz-plugins
```

### Basic Configuration
Create a configuration file for your humanoid robot:

```yaml
# humanoid_nav2_config.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    likelihood_max_dist: 2.0
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    scan_topic: "scan"
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "/odom"
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Specify the custom behavior tree for humanoid navigation
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_goal_updated_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node
    - nav2_remove_passed_goals_action_bt_node
    - nav2_planner_selector_bt_node
    - nav2_controller_selector_bt_node
    - nav2_goal_checker_selector_bt_node

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    # Use a custom controller for humanoid robots
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 50
      model_dt: 0.05
      batch_size: 1000
      vx_std: 0.2
      vy_std: 0.0
      wz_std: 0.3
      vx_max: 0.5
      vx_min: -0.2
      vy_max: 0.3
      wz_max: 0.8
      iteration_count: 1
      temperature: 0.3
      gamma: 0.015
      motion_model: "DiffDrive"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      state_reset_tol: 0.5
      publish_cost_grid_pc: false
      transform_tolerance: 0.3
      speed_limit_scale: 0.7

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: "odom"
      robot_base_frame: "base_link"
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      # Humanoid-specific footprint
      footprint: "[[-0.3, -0.2], [-0.3, 0.2], [0.3, 0.2], [0.3, -0.2]]"
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: False
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 8
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: "/scan"
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      always_send_full_costmap: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 0.5
      global_frame: "map"
      robot_base_frame: "base_link"
      use_sim_time: True
      # Humanoid-specific footprint
      footprint: "[[-0.3, -0.2], [-0.3, 0.2], [0.3, 0.2], [0.3, -0.2]]"
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: "/scan"
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      # Use a custom planner suitable for humanoid robots
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

## Customizing Nav2 for Humanoid Robots

### Humanoid-Specific Parameters
- **Footprint**: Larger and more complex than wheeled robots
- **Inflation Radius**: Account for robot's 3D presence
- **Velocity Limits**: Lower speeds for stability
- **Clearing Range**: Adjust for sensor placement on humanoid

### Custom Controllers
For humanoid robots, you may need custom controllers:
- **Footstep Planner**: Plans safe footholds
- **Balance Controller**: Maintains stability during walking
- **ZMP Controller**: Zero Moment Point for balance

## Launching Nav2 for Humanoid Robots

### Launch File Example
```xml
<!-- humanoid_nav2.launch.py -->
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there are a few models that
    # are not correctly placed in namespaces, the multi-robot example of nav2
    # has this workaround for that issue.
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'params_file',
            default_value='path/to/humanoid_nav2_config.yaml',
            description='Full path to the ROS2 parameters file to use'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=remappings),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=remappings),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=remappings),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=remappings),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=remappings),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}]),
    ])
```

## Advanced Humanoid Navigation Features

### Footstep Planning
For humanoid robots, path planning involves:
- Generating safe footholds
- Ensuring balance during stepping
- Adapting to terrain variations
- Handling obstacles that require stepping over

### Balance Maintenance
Critical for humanoid navigation:
- Center of Mass (CoM) control
- Zero Moment Point (ZMP) calculation
- Ankle stiffness adjustment
- Upper body stabilization

### Terrain Analysis
Humanoid robots can navigate more complex terrain:
- Stairs and steps
- Uneven surfaces
- Small obstacles that can be stepped over
- Sloped surfaces

## Practice Task 1: Nav2 Configuration
1. Create a Nav2 configuration file for a humanoid robot
2. Adjust parameters for humanoid-specific characteristics
3. Launch Nav2 with your configuration
4. Test navigation in a simple environment
5. Observe how the robot adapts to obstacles

## Practice Task 2: Path Planning Integration
1. Integrate Isaac ROS perception with Nav2
2. Use visual SLAM for localization
3. Plan paths through dynamic environments
4. Implement recovery behaviors for humanoid robots
5. Test navigation performance in various scenarios

## Troubleshooting Common Issues

### Localization Problems
- Ensure proper TF tree setup
- Verify sensor calibration
- Check map quality and resolution
- Validate coordinate frame relationships

### Path Planning Issues
- Adjust inflation parameters for humanoid size
- Tune velocity limits for stability
- Verify footprint configuration
- Check obstacle detection range

### Navigation Failures
- Implement appropriate recovery behaviors
- Adjust planner tolerances
- Verify controller parameters
- Check for kinematic constraints

## Integration with Isaac ROS

### Perception Integration
Combine Isaac ROS perception with Nav2:
- Use Isaac ROS for enhanced obstacle detection
- Integrate semantic maps with navigation
- Leverage 3D perception for better path planning

### AI-Enhanced Navigation
- Use deep learning for terrain classification
- Implement learned navigation behaviors
- Adaptive path planning based on environment

## Best Practices for Humanoid Navigation

### Safety Considerations
- Implement emergency stop mechanisms
- Monitor balance metrics continuously
- Use conservative velocity profiles
- Plan for graceful failure recovery

### Performance Optimization
- Optimize costmap resolution for humanoid size
- Use appropriate planning frequencies
- Implement efficient obstacle detection
- Balance computational load with real-time requirements

## Summary
In this chapter, you learned:
- How Navigation2 works and its key components
- Unique challenges of humanoid robot navigation
- How to configure Nav2 for humanoid robots
- How to customize parameters for bipedal locomotion
- Best practices for safe and effective navigation

## Next Steps
Congratulations! You've completed Module 3: The AI-Robot Brain. You now understand how to:
1. Generate synthetic data using Isaac Sim
2. Implement perception pipelines with Isaac ROS
3. Configure Nav2 for humanoid path planning

These skills form the foundation of advanced robotics AI, enabling robots to perceive, understand, and navigate in complex environments!