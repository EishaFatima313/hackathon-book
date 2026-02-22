import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '5c9'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'e7b'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '1bc'),
            routes: [
              {
                path: '/docs/ai-robot-brain/',
                component: ComponentCreator('/docs/ai-robot-brain/', 'a71'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ai-robot-control/',
                component: ComponentCreator('/docs/ai-robot-control/', '70b'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ai-robot-control/ai-robot-bridge',
                component: ComponentCreator('/docs/ai-robot-control/ai-robot-bridge', '454'),
                exact: true
              },
              {
                path: '/docs/code-examples/',
                component: ComponentCreator('/docs/code-examples/', '314'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/digital-twin/',
                component: ComponentCreator('/docs/digital-twin/', 'e82'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/digital-twin/digital-twin-unity',
                component: ComponentCreator('/docs/digital-twin/digital-twin-unity', '7d1'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/digital-twin/gazebo-basics',
                component: ComponentCreator('/docs/digital-twin/gazebo-basics', 'c75'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/digital-twin/sensor-simulation',
                component: ComponentCreator('/docs/digital-twin/sensor-simulation', 'c81'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ros2-basics/core-concepts',
                component: ComponentCreator('/docs/ros2-basics/core-concepts', '5d2'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ros2-basics/examples/custom-messages',
                component: ComponentCreator('/docs/ros2-basics/examples/custom-messages', '8dc'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ros2-basics/examples/publisher-tutorial',
                component: ComponentCreator('/docs/ros2-basics/examples/publisher-tutorial', 'c91'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ros2-basics/examples/service-tutorial',
                component: ComponentCreator('/docs/ros2-basics/examples/service-tutorial', '232'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ros2-basics/examples/subscriber-tutorial',
                component: ComponentCreator('/docs/ros2-basics/examples/subscriber-tutorial', 'a5e'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ros2-basics/exercises',
                component: ComponentCreator('/docs/ros2-basics/exercises', 'c60'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ros2-basics/introduction',
                component: ComponentCreator('/docs/ros2-basics/introduction', 'bdd'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ros2-basics/setup',
                component: ComponentCreator('/docs/ros2-basics/setup', 'a8e'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/vision-language-action/',
                component: ComponentCreator('/docs/vision-language-action/', '3a7'),
                exact: true,
                sidebar: "docs"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
