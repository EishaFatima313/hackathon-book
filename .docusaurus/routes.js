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
    component: ComponentCreator('/docs', '173'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'eb9'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '725'),
            routes: [
              {
                path: '/docs/ai-robot-brain/',
                component: ComponentCreator('/docs/ai-robot-brain/', 'a7f'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ai-robot-control/',
                component: ComponentCreator('/docs/ai-robot-control/', '935'),
                exact: true
              },
              {
                path: '/docs/digital-twin/',
                component: ComponentCreator('/docs/digital-twin/', 'a52'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/digital-twin/digital-twin-unity',
                component: ComponentCreator('/docs/digital-twin/digital-twin-unity', 'cd9'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/digital-twin/gazebo-basics',
                component: ComponentCreator('/docs/digital-twin/gazebo-basics', '60d'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/digital-twin/sensor-simulation',
                component: ComponentCreator('/docs/digital-twin/sensor-simulation', 'ea6'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ros2-basics/core-concepts',
                component: ComponentCreator('/docs/ros2-basics/core-concepts', '5c6'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ros2-basics/examples/custom-messages',
                component: ComponentCreator('/docs/ros2-basics/examples/custom-messages', '57d'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ros2-basics/examples/publisher-tutorial',
                component: ComponentCreator('/docs/ros2-basics/examples/publisher-tutorial', '817'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ros2-basics/examples/service-tutorial',
                component: ComponentCreator('/docs/ros2-basics/examples/service-tutorial', 'e43'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ros2-basics/examples/subscriber-tutorial',
                component: ComponentCreator('/docs/ros2-basics/examples/subscriber-tutorial', 'f88'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ros2-basics/exercises',
                component: ComponentCreator('/docs/ros2-basics/exercises', '620'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ros2-basics/introduction',
                component: ComponentCreator('/docs/ros2-basics/introduction', 'c6b'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/ros2-basics/setup',
                component: ComponentCreator('/docs/ros2-basics/setup', '139'),
                exact: true,
                sidebar: "docs"
              },
              {
                path: '/docs/vision-language-action/',
                component: ComponentCreator('/docs/vision-language-action/', 'eca'),
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
