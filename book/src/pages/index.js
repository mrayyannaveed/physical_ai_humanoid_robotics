import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';
import HomepageFeatures from '../components/HomepageFeatures';
import { useI18n } from '../utils/i18n'; // Import useI18n hook

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  const { t } = useI18n(); // Use the i18n hook

  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{t('welcome')}</p> {/* Use translated text */}
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            {t('docs')} - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

// Placeholder for HomepageFeatures - create this component if it doesn't exist
// book/src/components/HomepageFeatures.js
function HomepageFeaturesPlaceholder() {
  const { t } = useI18n();
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {[
            {
              title: 'Easy to Use',
              description: 'Docusaurus was designed from the ground up to be easily installed and used to get your website up and running quickly.',
            },
            {
              title: 'Focus on What Matters',
              description: 'Docusaurus lets you focus on your docs, and we&apos;ll do the chores. Go ahead and move your docs into the `docs` directory.',
            },
            {
              title: 'Powered by React',
              description: 'Extend or customize your project layout by reusing React. Docusaurus can be extended while reusing the same header and footer.',
            },
          ].map((props, idx) => (
            <div key={idx} className={clsx('col col--4')}>
              <div className="text--center padding-horiz--md">
                <h3>{t(props.title)}</h3> {/* Translate feature titles */}
                <p>{t(props.description)}</p> {/* Translate feature descriptions */}
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
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <HomepageFeaturesPlaceholder /> {/* Using the placeholder here */}
      </main>
    </Layout>
  );
}