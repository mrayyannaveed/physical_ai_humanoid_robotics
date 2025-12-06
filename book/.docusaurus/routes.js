import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '36a'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', 'e56'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'f47'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c7d'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '468'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '2fe'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '82f'),
    exact: true
  },
  {
    path: '/auth',
    component: ComponentCreator('/auth', 'bcb'),
    exact: true
  },
  {
    path: '/blog',
    component: ComponentCreator('/blog', '721'),
    exact: true
  },
  {
    path: '/blog/archive',
    component: ComponentCreator('/blog/archive', '691'),
    exact: true
  },
  {
    path: '/blog/tags',
    component: ComponentCreator('/blog/tags', '40a'),
    exact: true
  },
  {
    path: '/blog/tags/ai',
    component: ComponentCreator('/blog/tags/ai', '7e7'),
    exact: true
  },
  {
    path: '/blog/tags/introduction',
    component: ComponentCreator('/blog/tags/introduction', 'b6f'),
    exact: true
  },
  {
    path: '/blog/tags/robotics',
    component: ComponentCreator('/blog/tags/robotics', '1dd'),
    exact: true
  },
  {
    path: '/blog/welcome',
    component: ComponentCreator('/blog/welcome', '3a7'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '42c'),
    routes: [
      {
        path: '/docs/',
        component: ComponentCreator('/docs/', 'a8c'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/chapter-1/core-concepts',
        component: ComponentCreator('/docs/chapter-1/core-concepts', '0c2'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/chapter-1/history-and-evolution',
        component: ComponentCreator('/docs/chapter-1/history-and-evolution', 'c90'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/chapter-1/introduction',
        component: ComponentCreator('/docs/chapter-1/introduction', '9e0'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/chapter-2/actuators-and-sensors',
        component: ComponentCreator('/docs/chapter-2/actuators-and-sensors', '52d'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/chapter-2/kinematics-and-dynamics',
        component: ComponentCreator('/docs/chapter-2/kinematics-and-dynamics', 'd84'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/chapter-2/mechanical-design-and-materials',
        component: ComponentCreator('/docs/chapter-2/mechanical-design-and-materials', 'd30'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/chapter-3/computer-vision-for-robotics',
        component: ComponentCreator('/docs/chapter-3/computer-vision-for-robotics', 'd60'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/chapter-3/semantic-world-modeling',
        component: ComponentCreator('/docs/chapter-3/semantic-world-modeling', 'ec1'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/chapter-3/sensor-fusion',
        component: ComponentCreator('/docs/chapter-3/sensor-fusion', '58f'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/chapter-4/advanced-control-systems',
        component: ComponentCreator('/docs/chapter-4/advanced-control-systems', 'b38'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/chapter-4/motion-planning-algorithms',
        component: ComponentCreator('/docs/chapter-4/motion-planning-algorithms', '812'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/chapter-4/reinforcement-learning-and-sim-to-real',
        component: ComponentCreator('/docs/chapter-4/reinforcement-learning-and-sim-to-real', '92e'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/chapter-5/system-integration-and-ros',
        component: ComponentCreator('/docs/chapter-5/system-integration-and-ros', 'd23'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/chapter-5/testing-and-validation',
        component: ComponentCreator('/docs/chapter-5/testing-and-validation', 'b13'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/docs/chapter-5/the-future-of-humanoid-robotics',
        component: ComponentCreator('/docs/chapter-5/the-future-of-humanoid-robotics', '8a6'),
        exact: true,
        sidebar: "tutorialSidebar"
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '08c'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
