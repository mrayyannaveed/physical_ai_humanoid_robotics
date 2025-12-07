import React from 'react';
import clsx from 'clsx';
import styles from '../pages/index.module.css'; // Adjust path if necessary
import { useI18n } from '../utils/i18n'; // Import useI18n hook

// Import SVG assets directly
import mountainSvg from '../../static/img/undraw_docusaurus_mountain.svg';
import treeSvg from '../../static/img/undraw_docusaurus_tree.svg';
import reactSvg from '../../static/img/undraw_docusaurus_react.svg';

const FeatureList = [
  {
    title: 'Easy to Use',
    Svg: mountainSvg, // Use imported SVG
    description: 'Docusaurus was designed from the ground up to be easily installed and used to get your website up and running quickly.',
  },
  {
    title: 'Focus on What Matters',
    Svg: treeSvg, // Use imported SVG
    description: 'Docusaurus lets you focus on your docs, and we&apos;ll do the chores. Go ahead and move your docs into the `docs` directory.',
  },
  {
    title: 'Powered by React',
    Svg: reactSvg, // Use imported SVG
    description: 'Extend or customize your project layout by reusing React. Docusaurus can be extended while reusing the same header and footer.',
  },
];

function Feature({ Svg, title, description }) {
  const { t } = useI18n(); // Use the i18n hook
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{t(title)}</h3> {/* Translate feature titles */}
        <p>{t(description)}</p> {/* Translate feature descriptions */}
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={clsx(styles.features, 'features-fade-in')}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
