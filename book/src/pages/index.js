import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';

const modules = [
  {
    title: 'Chapter 1: Introduction to Humanoid Robotics',
    description: 'History, evolution, and core concepts of humanoid robotics.',
    link: '/docs/chapter-1/introduction',
  },
  {
    title: 'Chapter 2: Mechanical Design and Materials',
    description: 'Kinematics, dynamics, actuators, and sensors.',
    link: '/docs/chapter-2/mechanical-design-and-materials',
  },
  {
    title: 'Chapter 3: Perception and World Modeling',
    description: 'Sensor fusion, computer vision, and semantic world modeling.',
    link: '/docs/chapter-3/sensor-fusion',
  },
  {
    title: 'Chapter 4: Motion Planning and Control',
    description: 'Advanced algorithms for motion planning and control.',
    link: '/docs/chapter-4/motion-planning-algorithms',
  },
  {
    title: 'Chapter 5: System Integration and Future of Humanoid Robotics',
    description: 'System Integration, ROS, and the future of humanoid robotics.',
    link: '/docs/chapter-5/system-integration-and-ros',
  },
];

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', 'heroBanner', 'hero-fade-in')}>
      <div className="container">
        <h1 className="hero__title">Physical Ai And Humanoid Robotics</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={clsx(styles.buttons, 'hero__buttons')}>
          <Link
            className="button button--secondary button--lg cursor-pointer"
            href="/docs/chapter-1/introduction">
            Get Started
          </Link>
          <Link
            className="button button--info button--lg cursor-pointer"
            href="/assessment">
            Assessment
          </Link>
          <Link
            className="button button--github button--lg cursor-pointer"
            href="https://github.com/mrayyannaveed/physical_ai_native_book"
            target="_blank"
            rel="noopener noreferrer">
            <span className="github-icon"></span> Github
          </Link>
        </div>
      </div>
    </header>
  );
}

function ModulesSection() {
  return (
    <section className={clsx('modules-section', 'modules-fade-in')}>
      <div className="container">
        <h2 className="modules-title">Course Outline</h2>
        <div className="row">
          {modules.map((module, idx) => (
            <div className={clsx('col col--4', styles.module)} key={idx}>
              <div className="card">
                <div className="card__header">
                  <h3>{module.title}</h3>
                </div>
                <div className="card__body">
                  <p>{module.description}</p>
                </div>
                <div className="card__footer">
                  <Link
                    className="button button--primary button--block"
                    to={module.link}>
                    Explore
                  </Link>
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Learn about Physical AI and Humanoid Robotics, from core concepts to advanced applications.">
      <HomepageHeader />
      <main>
        <ModulesSection />
      </main>
    </Layout>
  );
}