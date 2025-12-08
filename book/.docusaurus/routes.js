import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/assessment',
    component: ComponentCreator('/assessment', '202'),
    exact: true
  },
  {
    path: '/auth',
    component: ComponentCreator('/auth', '4fe'),
    exact: true
  },
  {
    path: '/blog',
    component: ComponentCreator('/blog', '4e4'),
    exact: true
  },
  {
    path: '/blog/archive',
    component: ComponentCreator('/blog/archive', '182'),
    exact: true
  },
  {
    path: '/blog/tags',
    component: ComponentCreator('/blog/tags', '287'),
    exact: true
  },
  {
    path: '/blog/tags/ai',
    component: ComponentCreator('/blog/tags/ai', '9b8'),
    exact: true
  },
  {
    path: '/blog/tags/introduction',
    component: ComponentCreator('/blog/tags/introduction', '662'),
    exact: true
  },
  {
    path: '/blog/tags/robotics',
    component: ComponentCreator('/blog/tags/robotics', '49b'),
    exact: true
  },
  {
    path: '/blog/welcome',
    component: ComponentCreator('/blog/welcome', 'f85'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'eb5'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '264'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '33a'),
            routes: [
              {
                path: '/docs/',
                component: ComponentCreator('/docs/', '4a8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter-1/core-concepts',
                component: ComponentCreator('/docs/chapter-1/core-concepts', 'b8e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter-1/history-and-evolution',
                component: ComponentCreator('/docs/chapter-1/history-and-evolution', '844'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter-1/introduction',
                component: ComponentCreator('/docs/chapter-1/introduction', '4a0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter-2/actuators-and-sensors',
                component: ComponentCreator('/docs/chapter-2/actuators-and-sensors', '1cb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter-2/kinematics-and-dynamics',
                component: ComponentCreator('/docs/chapter-2/kinematics-and-dynamics', '771'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter-2/mechanical-design-and-materials',
                component: ComponentCreator('/docs/chapter-2/mechanical-design-and-materials', '349'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter-3/computer-vision-for-robotics',
                component: ComponentCreator('/docs/chapter-3/computer-vision-for-robotics', 'a88'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter-3/semantic-world-modeling',
                component: ComponentCreator('/docs/chapter-3/semantic-world-modeling', 'bc0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter-3/sensor-fusion',
                component: ComponentCreator('/docs/chapter-3/sensor-fusion', '1e0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter-4/advanced-control-systems',
                component: ComponentCreator('/docs/chapter-4/advanced-control-systems', '44b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter-4/motion-planning-algorithms',
                component: ComponentCreator('/docs/chapter-4/motion-planning-algorithms', 'a16'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter-4/reinforcement-learning-and-sim-to-real',
                component: ComponentCreator('/docs/chapter-4/reinforcement-learning-and-sim-to-real', 'c53'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter-5/system-integration-and-ros',
                component: ComponentCreator('/docs/chapter-5/system-integration-and-ros', '4d4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter-5/testing-and-validation',
                component: ComponentCreator('/docs/chapter-5/testing-and-validation', 'aca'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chapter-5/the-future-of-humanoid-robotics',
                component: ComponentCreator('/docs/chapter-5/the-future-of-humanoid-robotics', '84d'),
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
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
