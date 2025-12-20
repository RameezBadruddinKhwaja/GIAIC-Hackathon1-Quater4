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

  url: 'https://giaic-hackathon1-quater4.vercel.app/',
  baseUrl: '/',

  organizationName: 'Rameez-Bader',
  projectName: 'physical-ai-textbook',

  onBrokenLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // Proxy API requests to the deployed backend
  // Custom fields for environment variables (accessible in React components)
  customFields: {},

  // Enable Mermaid support
  markdown: {
    mermaid: true,
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  themes: ['@docusaurus/theme-mermaid'],

  staticDirectories: ['static'],

  // Development server proxy configuration to forward API requests to the deployed backend
  customFields: {
    // Use environment variable for backend API URL, default to deployed URL
    BACKEND_API_URL: process.env.BACKEND_API_URL || 'https://rameez12-physical-ai-textbook.hf.space',
  },

  // Development server configuration with proxy for API requests
  devServer: {
    proxy: {
      '/api/chatbot': {
        target: process.env.BACKEND_API_URL || 'https://rameez12-physical-ai-textbook.hf.space',
        changeOrigin: true,
        secure: true,
        logLevel: 'debug',
        onProxyReq: (proxyReq, req, res) => {
          console.log(`Proxying ${req.method} ${req.url} to ${process.env.BACKEND_API_URL || 'https://rameez12-physical-ai-textbook.hf.space'}`);
        },
        onProxyRes: (proxyRes, req, res) => {
          console.log(`Response from ${req.url}: ${proxyRes.statusCode}`);
        }
      },
    },
  },

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
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Content',
        },
        {
          type: 'docSidebar',
          sidebarId: 'hardwareSidebar',
          position: 'left',
          label: 'Hardware Lab',
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
