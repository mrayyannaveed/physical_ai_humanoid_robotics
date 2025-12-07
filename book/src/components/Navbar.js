import React from 'react';
import { useI18n } from '../utils/i18n';
import { useColorMode } from '@docusaurus/theme-common'; // Docusaurus hook for dark mode

// This is a placeholder custom Navbar component.
// In Docusaurus, you typically customize the theme's Navbar component
// via swizzling or by providing props in docusaurus.config.js.
// For the purpose of this task, we'll assume a custom component here.

function CustomNavbar() {
  const { t, locale, setLanguage } = useI18n();
  const { colorMode, setColorMode } = useColorMode();

  const toggleLocale = () => {
    setLanguage(locale === 'en' ? 'ur' : 'en');
  };

  const toggleDarkMode = () => {
    setColorMode(colorMode === 'dark' ? 'light' : 'dark');
  };

  return (
    <nav className="navbar navbar--fixed-top navbar-fade-in">
      <div className="navbar__inner">
        <a className="navbar__brand" href="/">
          {/* You might want to use a custom logo here */}
          <strong>{t('home')}</strong>
        </a>
        <div className="navbar__items">
          <a className="navbar__item navbar__link" href="/docs">
            {t('docs')}
          </a>
          <a className="navbar__item navbar__link" href="/blog">
            {t('blog')}
          </a>
          <a className="navbar__item navbar__link" href="/auth">
            {t('login')}
          </a>
        </div>
        <div className="navbar__items navbar__items--right">
          {/* Language Toggle */}
          <div className="navbar__item dropdown dropdown--hoverable">
            <a className="navbar__link">
              {locale.toUpperCase()}
            </a>
            <ul className="dropdown__menu">
              <li>
                <a className="dropdown__link" onClick={() => setLanguage('en')}>
                  English
                </a>
              </li>
              <li>
                <a className="dropdown__link" onClick={() => setLanguage('ur')}>
                  اردو
                </a>
              </li>
            </ul>
          </div>

          {/* Dark/Light Mode Toggle */}
          <div className="navbar__item dropdown dropdown--hoverable">
            <a className="navbar__link">
              {colorMode === 'dark' ? t('dark_mode') : t('light_mode')}
            </a>
            <ul className="dropdown__menu">
              <li>
                <a className="dropdown__link" onClick={() => setColorMode('dark')}>
                  {t('dark_mode')}
                </a>
              </li>
              <li>
                <a className="dropdown__link" onClick={() => setColorMode('light')}>
                  {t('light_mode')}
                </a>
              </li>
            </ul>
          </div>
        </div>
      </div>
    </nav>
  );
}

export default CustomNavbar;
