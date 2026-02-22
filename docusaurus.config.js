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

  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

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
          editUrl: 'https://github.com/your-github-username/robotics-education-platform/edit/main/',
          showLastUpdateAuthor: true,
          showLastUpdateTime: true,
          routeBasePath: '/docs',
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
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Social Card
      image: 'img/docusaurus-social-card.jpg',

      // Color Mode Configuration
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },

      // Announcement Bar (Optional - for important notices)
      // announcementBar: {
      //   id: 'new_content',
      //   content: '🎉 New Module: Vision-Language-Action Models now available!',
      //   backgroundColor: 'var(--ifm-color-primary)',
      //   textColor: '#ffffff',
      //   isCloseable: true,
      // },

      // Navbar Configuration
      navbar: {
        title: 'Robotics Education',
        logo: {
          alt: 'Robotics Logo',
          src: 'img/docusaurus-logo-png.png',
          width: 32,
          height: 32,
        },
        hideOnScroll: false,
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'docs',
            position: 'left',
            label: 'Modules',
          },
          {
            type: 'dropdown',
            label: 'Resources',
            position: 'left',
            items: [
              {
                label: 'Code Examples',
                to: '/docs/code-examples/',
              },
              {
                label: 'AI Robot Control',
                to: '/docs/ai-robot-control/',
              },
            ],
          },
          {
            href: 'https://github.com/your-github-username/robotics-education-platform',
            label: 'GitHub',
            position: 'right',
            className: 'header-github-link',
            'aria-label': 'GitHub repository',
          },
        ],
      },

      // Footer Configuration
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
              {
                label: 'Setup Guide',
                to: '/docs/ros2-basics/setup',
              },
              {
                label: 'Tutorials',
                to: '/docs/ros2-basics/examples/publisher-tutorial',
              },
            ],
          },
          {
            title: 'Advanced Topics',
            items: [
              {
                label: 'Digital Twin',
                to: '/docs/digital-twin/',
              },
              {
                label: 'AI Robot Brain',
                to: '/docs/ai-robot-brain/',
              },
              {
                label: 'Vision-Language-Action',
                to: '/docs/vision-language-action/',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/your-github-username/robotics-education-platform',
              },
              {
                label: 'Discussions',
                href: 'https://github.com/your-github-username/robotics-education-platform/discussions',
              },
              {
                label: 'Issues',
                href: 'https://github.com/your-github-username/robotics-education-platform/issues',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Robotics Education Platform. Built with Docusaurus.`,
      },

      // Prism Theme for Code Blocks
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['python', 'bash', 'json', 'yaml', 'docker', 'cmake'],
      },

      // Table of Contents Configuration
      tableOfContents: {
        minHeadingLevel: 2,
        maxHeadingLevel: 5,
      },

      // Docs Sidebar Configuration
      docs: {
        sidebar: {
          hideable: true,
          autoCollapseCategories: true,
        },
      },

      // Metadata Configuration
      metadata: [
        {name: 'keywords', content: 'robotics, ROS 2, AI, digital twin, simulation, Gazebo, Unity, NVIDIA Isaac, VLA, humanoid robots'},
        {name: 'description', content: 'Comprehensive robotics education platform covering ROS 2, AI integration, digital twins, and advanced robot control.'},
        {name: 'author', content: 'Robotics Education Platform'},
        {name: 'robots', content: 'index, follow'},
      ],

      // Algolia Search Configuration (Optional - add your credentials)
      // algolia: {
      //   appId: 'YOUR_APP_ID',
      //   apiKey: 'YOUR_API_KEY',
      //   indexName: 'robotics-education',
      //   contextualSearch: true,
      //   searchPagePath: 'search',
      // },
    }),

  // Plugins Configuration
  plugins: [
    // Custom plugins can be added here
  ],

  // Client Modules (for custom scripts)
  clientModules: [
    // require.resolve('./src/clientModules.js'),
  ],

  // Stylesheets
  stylesheets: [
    // Add external stylesheets if needed
  ],

  // Scripts
  scripts: [
    // Add external scripts if needed
  ],
};

module.exports = config;
