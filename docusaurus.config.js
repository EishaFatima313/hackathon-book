// @ts-check

const {themes} = require('prism-react-renderer');

const lightCodeTheme = themes.github;
const darkCodeTheme = themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Robotics Education Platform',
  tagline: 'Learn Robotics with ROS 2 and AI Integration',
  favicon: 'img/favicon.ico',
  url: 'https://your-docusaurus-test-site.com',
  baseUrl: '/',

  organizationName: 'your-github-username',
  projectName: 'robotics-education-platform',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
        },

        // ✅ BLOG DISABLED
        blog: false,

        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    ({
      image: 'img/docusaurus-social-card.jpg',

      navbar: {
        title: 'Robotics Education',
        logo: {
          alt: 'Robotics Logo',
          src: 'img/docusaurus-logo-png.png',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'docs',
            position: 'left',
            label: 'Modules',
          },
          {
            href: 'https://github.com/your-github-username/robotics-education-platform',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },

      footer: {
        style: 'dark',
        links: [
          {
            title: 'Start Learning',
            items: [
              {
                label: 'ROS 2 Introduction',
                to: '/docs/ros2-basics/introduction',
              },
            ],
          },
          {
            title: 'Platform',
            items: [
              {
                label: 'Digital Twin',
                to: '/docs/digital-twin/',
              },
              {
                label: 'AI Robot Brain',
                to: '/docs/ai-robot-brain/',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Robotics Education Platform`,
      },

      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
      },
    }),
};

module.exports = config;