import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import { useI18n } from '../utils/i18n';
import styles from './ChapterControls.module.css';
import { useColorMode } from '@docusaurus/theme-common';

export default function ChapterControls() {
  const location = useLocation();
  const { locale, setLanguage } = useI18n();

  // Theme logic with fallback
  const [theme, setTheme] = useState('light');
  const [hasThemeContext, setHasThemeContext] = useState(false);
  
  let contextColorMode, contextSetColorMode;
  try {
     const ctx = useColorMode();
     contextColorMode = ctx.colorMode;
     contextSetColorMode = ctx.setColorMode;
  } catch (e) {
     // context missing
  }

  useEffect(() => {
    if (contextSetColorMode) {
      setHasThemeContext(true);
      setTheme(contextColorMode);
    } else {
      // Fallback: Read from DOM
      const current = document.documentElement.getAttribute('data-theme') || 'light';
      setTheme(current);
      
      const observer = new MutationObserver((mutations) => {
        mutations.forEach((mutation) => {
          if (mutation.type === 'attributes' && mutation.attributeName === 'data-theme') {
            setTheme(document.documentElement.getAttribute('data-theme'));
          }
        });
      });
      observer.observe(document.documentElement, { attributes: true });
      return () => observer.disconnect();
    }
  }, [contextColorMode, contextSetColorMode]);

  const toggleTheme = () => {
    if (hasThemeContext) {
      contextSetColorMode(theme === 'dark' ? 'light' : 'dark');
    } else {
      const newTheme = theme === 'dark' ? 'light' : 'dark';
      document.documentElement.setAttribute('data-theme', newTheme);
      localStorage.setItem('theme', newTheme);
      setTheme(newTheme);
    }
  };

  const toggleLang = () => setLanguage(locale === 'en' ? 'ur' : 'en');

  // Only show on docs pages (chapter pages)
  if (!location.pathname.startsWith('/docs')) {
    return null;
  }

  return (
    <div className={styles.controlsContainer}>
      <button 
        className={styles.controlButton} 
        onClick={toggleTheme}
        title={`Switch to ${theme === 'dark' ? 'light' : 'dark'} mode`}
      >
        {theme === 'dark' ? '☀️' : '🌙'}
      </button>
      <button 
        className={styles.controlButton} 
        onClick={toggleLang}
        title={`Switch to ${locale === 'en' ? 'Urdu' : 'English'}`}
      >
        {locale === 'en' ? 'UR' : 'EN'}
      </button>
    </div>
  );
}