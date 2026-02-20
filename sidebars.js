const sidebars = {
  docs: [
    {
      type: 'category',
      label: 'Module 1: ROS 2 Basics',
      items: [
        'ros2-basics/introduction',
        'ros2-basics/core-concepts',
        'ros2-basics/setup',
        {
          type: 'category',
          label: 'Tutorials',
          items: [
            'ros2-basics/examples/publisher-tutorial',
            'ros2-basics/examples/subscriber-tutorial',
            'ros2-basics/examples/service-tutorial',
            'ros2-basics/examples/custom-messages',
          ],
        },
        'ros2-basics/exercises',
      ],
    },

    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo & Unity)',
      items: [
        'digital-twin/index',  // ✅ FIXED
        'digital-twin/gazebo-basics',
        'digital-twin/digital-twin-unity',
        'digital-twin/sensor-simulation',
      ],
    },

    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'ai-robot-brain/index',  // ✅ FIXED
      ],
    },

    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'vision-language-action/index',  // ✅ FIXED
      ],
    },
  ],
};

export default sidebars;