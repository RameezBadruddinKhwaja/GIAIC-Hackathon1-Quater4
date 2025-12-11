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
  // Main unified sidebar with all modules (13 weeks)
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Part 1: The Nervous System (ROS 2)',
      collapsible: true,
      collapsed: false,
      items: [
        'week-01-ros2-basics/index',
        'week-02-nodes-topics/index',
        'week-03-urdf-modeling/index',
        'week-04-services-actions/index',
        'week-05-nav2/index',
      ],
    },
    {
      type: 'category',
      label: 'Part 2: The Digital Twin (Simulation)',
      collapsible: true,
      collapsed: true,
      items: [
        'week-06-gazebo-sim/index',
        'week-07-unity-sim/index',
      ],
    },
    {
      type: 'category',
      label: 'Part 3: The Brain (NVIDIA Isaac)',
      collapsible: true,
      collapsed: true,
      items: [
        'week-08-isaac-sim-basics/index',
        'week-09-isaac-ros/index',
        'week-10-isaac-orbit/index',
      ],
    },
    {
      type: 'category',
      label: 'Part 4: VLA & Humanoids',
      collapsible: true,
      collapsed: true,
      items: [
        'week-11-vla-intro/index',
        'week-12-droid-deployment/index',
        'week-13-humanoid-control/index',
      ],
    },
  ],

  hardwareSidebar: [
    {
      type: 'category',
      label: 'Hardware Lab Guide',
      items: [
        'hardware-lab/rtx-4090-setup',
        'hardware-lab/jetson-orin-nano-setup',
        'hardware-lab/ros2-workspace-setup',
        'hardware-lab/troubleshooting',
      ],
    },
  ],
};

export default sidebars;
