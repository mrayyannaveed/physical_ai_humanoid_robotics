import React from 'react';
import { useI18n } from '../utils/i18n';

// This is a placeholder custom Footer component.
// In Docusaurus, you typically customize the theme's Footer component
// via swizzling or by providing props in docusaurus.config.js.

function CustomFooter() {
  const { t } = useI18n();

  return (
    <footer className="footer footer--dark">
      <div className="container container--fluid">
        <div className="row footer__links">
          <div className="col footer__col">
            <h4 className="footer__title">Links</h4>
            <ul className="footer__items">
              <li className="footer__item">
                <a href="/docs" className="footer__link-item">
                  {t('docs')}
                </a>
              </li>
              <li className="footer__item">
                <a href="/blog" className="footer__link-item">
                  {t('blog')}
                </a>
              </li>
              <li className="footer__item">
                <a href="https://github.com/your-org/your-repo" target="_blank" rel="noopener noreferrer" className="footer__link-item">
                  GitHub
                </a>
              </li>
            </ul>
          </div>
          {/* Add more columns for other links or social media */}
        </div>
        <div className="text--center margin-bottom--sm">
          Copyright © {new Date().getFullYear()} My Project, Inc. Built with Docusaurus.
        </div>
      </div>
    </footer>
  );
}

export default CustomFooter;
