import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'The Robotic Nervous System (ROS 2)',
      items: [
        'ros2-basics/ros2-intro',
        'ai-robot-control/ai-robot-bridge',
        'humanoid-urdf/humanoid-structure'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo & Unity)',
      items: [
        'digital-twin/index',
        'digital-twin/gazebo-basics',
        'digital-twin/digital-twin-unity',
        'digital-twin/sensor-simulation'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'ai-robot-brain/index',
        'ai-robot-brain/isaac-sim-synthetic-data',
        'ai-robot-brain/isaac-ros-vslam',
        'ai-robot-brain/nav2-humanoid-planning'
      ],
    },
    {
      type: 'category',
      label: 'Advanced Topics',
      items: [
        'tutorial-basics/congratulations',
        'tutorial-extras/translate-your-site'
      ],
    },
  ],
};

export default sidebars;