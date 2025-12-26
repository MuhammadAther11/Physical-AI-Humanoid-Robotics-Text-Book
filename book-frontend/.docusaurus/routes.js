import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/ar/blog',
    component: ComponentCreator('/ar/blog', '517'),
    exact: true
  },
  {
    path: '/ar/blog/archive',
    component: ComponentCreator('/ar/blog/archive', '085'),
    exact: true
  },
  {
    path: '/ar/blog/authors',
    component: ComponentCreator('/ar/blog/authors', '572'),
    exact: true
  },
  {
    path: '/ar/blog/authors/all-sebastien-lorber-articles',
    component: ComponentCreator('/ar/blog/authors/all-sebastien-lorber-articles', '7b4'),
    exact: true
  },
  {
    path: '/ar/blog/authors/yangshun',
    component: ComponentCreator('/ar/blog/authors/yangshun', 'b8c'),
    exact: true
  },
  {
    path: '/ar/blog/first-blog-post',
    component: ComponentCreator('/ar/blog/first-blog-post', 'cd1'),
    exact: true
  },
  {
    path: '/ar/blog/long-blog-post',
    component: ComponentCreator('/ar/blog/long-blog-post', '192'),
    exact: true
  },
  {
    path: '/ar/blog/mdx-blog-post',
    component: ComponentCreator('/ar/blog/mdx-blog-post', '117'),
    exact: true
  },
  {
    path: '/ar/blog/tags',
    component: ComponentCreator('/ar/blog/tags', '7ca'),
    exact: true
  },
  {
    path: '/ar/blog/tags/docusaurus',
    component: ComponentCreator('/ar/blog/tags/docusaurus', 'fcd'),
    exact: true
  },
  {
    path: '/ar/blog/tags/facebook',
    component: ComponentCreator('/ar/blog/tags/facebook', 'c8f'),
    exact: true
  },
  {
    path: '/ar/blog/tags/hello',
    component: ComponentCreator('/ar/blog/tags/hello', 'b0b'),
    exact: true
  },
  {
    path: '/ar/blog/tags/hola',
    component: ComponentCreator('/ar/blog/tags/hola', '803'),
    exact: true
  },
  {
    path: '/ar/blog/welcome',
    component: ComponentCreator('/ar/blog/welcome', 'c05'),
    exact: true
  },
  {
    path: '/ar/markdown-page',
    component: ComponentCreator('/ar/markdown-page', 'a73'),
    exact: true
  },
  {
    path: '/ar/docs',
    component: ComponentCreator('/ar/docs', '4a6'),
    routes: [
      {
        path: '/ar/docs',
        component: ComponentCreator('/ar/docs', 'f8e'),
        routes: [
          {
            path: '/ar/docs',
            component: ComponentCreator('/ar/docs', '316'),
            routes: [
              {
                path: '/ar/docs/module-1-ros2-nervous-system/humanoid-robot-description-urdf',
                component: ComponentCreator('/ar/docs/module-1-ros2-nervous-system/humanoid-robot-description-urdf', '065'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ar/docs/module-1-ros2-nervous-system/python-agents-with-rclpy',
                component: ComponentCreator('/ar/docs/module-1-ros2-nervous-system/python-agents-with-rclpy', 'c8c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ar/docs/module-1-ros2-nervous-system/ros2-fundamentals',
                component: ComponentCreator('/ar/docs/module-1-ros2-nervous-system/ros2-fundamentals', 'a55'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ar/docs/module-2/digital-twins',
                component: ComponentCreator('/ar/docs/module-2/digital-twins', 'aef'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ar/docs/module-2/unity-visualization',
                component: ComponentCreator('/ar/docs/module-2/unity-visualization', '279'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ar/docs/module-3-ai-robot-brain/isaac-overview',
                component: ComponentCreator('/ar/docs/module-3-ai-robot-brain/isaac-overview', '15f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ar/docs/module-3-ai-robot-brain/isaac-ros',
                component: ComponentCreator('/ar/docs/module-3-ai-robot-brain/isaac-ros', 'dd5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ar/docs/module-3-ai-robot-brain/isaac-sim',
                component: ComponentCreator('/ar/docs/module-3-ai-robot-brain/isaac-sim', '718'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ar/docs/module-3-ai-robot-brain/nav2-humanoids',
                component: ComponentCreator('/ar/docs/module-3-ai-robot-brain/nav2-humanoids', '3ab'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ar/docs/module-4-vla/autonomous-pipeline',
                component: ComponentCreator('/ar/docs/module-4-vla/autonomous-pipeline', '844'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ar/docs/module-4-vla/llm-task-planning',
                component: ComponentCreator('/ar/docs/module-4-vla/llm-task-planning', '7ad'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ar/docs/module-4-vla/vla-overview',
                component: ComponentCreator('/ar/docs/module-4-vla/vla-overview', '909'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ar/docs/module-4-vla/voice-to-action',
                component: ComponentCreator('/ar/docs/module-4-vla/voice-to-action', 'd40'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/ar/',
    component: ComponentCreator('/ar/', 'cb7'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
