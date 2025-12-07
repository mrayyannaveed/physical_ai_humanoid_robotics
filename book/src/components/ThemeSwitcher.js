import React from 'react';
import { useColorMode } from '@docusaurus/theme-common';
import { useI18n } from '../utils/i18n'; // Assuming i18n is used for labels

function ThemeSwitcher() {
  const { colorMode, setColorMode } = useColorMode();
  const { t } = useI18n();

  const toggleColorMode = (newMode) => {
    setColorMode(newMode);
    // TODO: Optionally save user preference to backend via API (T037 related)
  };

  return (
    <div className="dropdown dropdown--hoverable theme-switcher-fade-in">
      <a className="button button--secondary">
        {colorMode === 'dark' ? t('dark_mode') : t('light_mode')}
      </a>
      <ul className="dropdown__menu">
        <li>
          <a className="dropdown__link" onClick={() => toggleColorMode('light')}>
            {t('light_mode')}
          </a>
        </li>
        <li>
          <a className="dropdown__link" onClick={() => toggleColorMode('dark')}>
            {t('dark_mode')}
          </a>
        </li>
      </ul>
    </div>
  );
}

export default ThemeSwitcher;
