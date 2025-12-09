# docusaurus-deployer Skill

## Purpose
Configure and deploy Docusaurus sites to Vercel, GitHub Pages, or other platforms.

## Key Patterns

### Vercel Deployment (Recommended)
```bash
# Install Vercel CLI
npm install -g vercel

# Deploy from apps/docs directory
cd apps/docs
vercel --prod

# Environment variables (set in Vercel dashboard)
REACT_APP_API_URL=https://api.yourdomain.com
```

### docusaurus.config.ts Configuration
```typescript
import {themes as prismThemes} from 'prism-react-renderer';

export default {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'AI-Native Learning Platform for ROS 2, NVIDIA Isaac, and VLA Systems',
  favicon: 'img/favicon.ico',

  url: 'https://your-textbook.vercel.app',
  baseUrl: '/',

  organizationName: 'your-org',
  projectName: 'physical-ai-textbook',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
  },

  markdown: {
    mermaid: true,
  },

  themes: ['@docusaurus/theme-mermaid'],

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: '/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],

  themeConfig: {
    navbar: {
      title: 'Physical AI Textbook',
      items: [
        {to: '/', label: 'Content', position: 'left'},
        {to: '/hardware-lab', label: 'Hardware Lab', position: 'left'},
        {to: '/chat', label: 'Chat', position: 'left'},
      ],
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'cpp', 'bash', 'yaml'],
    },
  },
};
```

### Build Verification
```bash
# Local build test
cd apps/docs
npm run build
npm run serve

# Check for broken links
npm run build -- --out-dir build --config docusaurus.config.ts
```

## Usage Context
- Initial deployment
- CI/CD setup
- Production releases
- Environment configuration
