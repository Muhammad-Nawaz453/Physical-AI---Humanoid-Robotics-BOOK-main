import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Master the Future of Embodied Intelligence',
  favicon: 'img/favicon.ico',

  url: 'https://github.com/Muhammad-Nawaz453/',
  baseUrl: 'https://github.com/Muhammad-Nawaz453/Physical-AI---Humanoid-Robotics',

  organizationName: 'https://github.com/Muhammad-Nawaz453/',
  projectName: 'Physical-AI---Humanoid-Robotics',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: '/',
          editUrl: 'https://github.com/Muhammad-Nawaz453/Physical-AI---Humanoid-Robotics/tree/main/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {
          type: 'localeDropdown',
          position: 'right',
        },
        {
          href: 'https://github.com/YOUR-USERNAME/physical-ai-textbook',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Modules',
          items: [
            {
              label: 'ROS 2 Fundamentals',
              to: '/module-1/introduction',
            },
            {
              label: 'Gazebo & Unity',
              to: '/module-2/introduction',
            },
            {
              label: 'NVIDIA Isaac',
              to: '/module-3/introduction',
            },
            {
              label: 'Vision-Language-Action',
              to: '/module-4/introduction',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Panaversity',
              href: 'https://panaversity.org',
            },
            {
              label: 'ROS 2 Docs',
              href: 'https://docs.ros.org',
            },
            {
              label: 'NVIDIA Isaac',
              href: 'https://developer.nvidia.com/isaac',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Textbook. Built for Panaversity Hackathon.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      // Remove additionalLanguages entirely to avoid loading issues
      additionalLanguages: ['python', 'bash'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;