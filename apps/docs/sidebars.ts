import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  hardwareSidebar: [
    {
      type: 'category',
      label: 'Hardware Lab Guide',
      items: ['hardware-lab/hardware-setup'],
    },
  ],

  module1Sidebar: [
    {
      type: 'category',
      label: 'Module 1: The Nervous System (ROS 2)',
      items: [
        'module-1/ros2-fundamentals',
        'module-1/nodes-and-topics',
      ],
    },
  ],

  module2Sidebar: [
    {
      type: 'category',
      label: 'Module 2: The Digital Twin',
      items: [
        'module-2/gazebo-simulation',
        'module-2/urdf-guides',
      ],
    },
  ],

  module3Sidebar: [
    {
      type: 'category',
      label: 'Module 3: The Brain (Isaac Sim)',
      items: [
        'module-3/isaac-sim-setup',
        'module-3/nav2-planning',
      ],
    },
  ],

  module4Sidebar: [
    {
      type: 'category',
      label: 'Module 4: VLA & Humanoids',
      items: [
        'module-4/vla-introduction',
        'module-4/voice-to-action',
      ],
    },
  ],
};

export default sidebars;
