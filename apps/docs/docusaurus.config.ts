import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';
import 'dotenv/config';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'AI-Native Learning Platform for ROS 2, NVIDIA Isaac, and VLA Systems',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  url: 'https://your-domain.com',
  baseUrl: '/',

  organizationName: 'panaversity',
  projectName: 'physical-ai-textbook',

  onBrokenLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // Custom fields for environment variables (accessible in React components)
  customFields: {
    apiUrl: process.env.REACT_APP_API_URL || 'http://localhost:8000',
    githubClientId: process.env.BETTER_AUTH_GITHUB_CLIENT_ID || '',
  },

  // Enable Mermaid support
  markdown: {
    mermaid: true,
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  themes: ['@docusaurus/theme-mermaid'],

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: 'modules',
        },
        blog: false,  // Disable blog for textbook focus
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/social-card.jpg',
    colorMode: {
      defaultMode: 'dark',
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI Textbook',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'hardwareSidebar',
          position: 'left',
          label: 'Hardware Lab',
        },
        {
          type: 'docSidebar',
          sidebarId: 'module1Sidebar',
          position: 'left',
          label: 'Module 1: ROS 2',
        },
        {
          type: 'docSidebar',
          sidebarId: 'module2Sidebar',
          position: 'left',
          label: 'Module 2: Digital Twin',
        },
        {
          type: 'docSidebar',
          sidebarId: 'module3Sidebar',
          position: 'left',
          label: 'Module 3: Isaac Sim',
        },
        {
          type: 'docSidebar',
          sidebarId: 'module4Sidebar',
          position: 'left',
          label: 'Module 4: VLA',
        },
      ],
    },
    footer: {
      style: 'dark',
      copyright: `Copyright © ${new Date().getFullYear()} Rameez Badruddin Khwaja. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'cpp', 'bash', 'yaml'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
