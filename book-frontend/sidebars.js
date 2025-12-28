// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Getting Started',
      items: [
        'intro',
        'installation',
        'quickstart',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Nervous System',
      items: [
        'module-1-ros2-nervous-system/01-ros2-fundamentals',
        'module-1-ros2-nervous-system/02-python-agents-with-rclpy',
        'module-1-ros2-nervous-system/03-humanoid-robot-description-urdf',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      items: [
        'module-2/01-digital-twins',
        'module-2/02-unity-visualization',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: AI Robot Brain',
      items: [
        'module-3-ai-robot-brain/01-isaac-overview',
        'module-3-ai-robot-brain/02-isaac-sim',
        'module-3-ai-robot-brain/03-isaac-ros',
        'module-3-ai-robot-brain/04-nav-2-humanoids',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: VLA Autonomous Pipeline',
      items: [
        'module-4-vla/vla-overview',
        'module-4-vla/llm-task-planning',
        'module-4-vla/voice-to-action',
        'module-4-vla/autonomous-pipeline',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Advanced Topics',
      items: [
        'advanced/ros2-navigation',
        'advanced/hardware-integration',
        'advanced/troubleshooting',
      ],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'appendices/glossary',
        'appendices/resources',
        'appendices/faq',
      ],
      collapsed: true,
    },
  ],
};

module.exports = sidebars;