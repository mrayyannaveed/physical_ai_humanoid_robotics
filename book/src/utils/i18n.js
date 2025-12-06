// book/src/utils/i18n.js
import { createContext, useContext, useState } from 'react';

const i18nContext = createContext(null);

export const translations = {
  en: {
    greeting: 'Hello',
    welcome: 'Welcome to the Humanoid Robot Dashboard',
    login: 'Login',
    signup: 'Signup',
    username: 'Username',
    email: 'Email',
    password: 'Password',
    need_account: 'Need an account?',
    already_account: 'Already have an account?',
    login_successful: 'Login successful!',
    registration_successful: 'Registration successful!',
    error_occurred: 'An error occurred.',
    network_error: 'Network error or server unavailable.',
    home: 'Home',
    docs: 'Docs',
    blog: 'Blog',
    dark_mode: 'Dark Mode',
    light_mode: 'Light Mode',
    preferences_updated: 'Preferences updated successfully',
    invalid_input: 'Invalid input.',
    unauthorized: 'Unauthorized.',
  },
  ur: {
    greeting: 'ہیلو', // Hello
    welcome: 'ہیومنائیڈ روبوٹ ڈیش بورڈ میں خوش آمدید', // Welcome to the Humanoid Robot Dashboard
    login: 'لاگ ان', // Login
    signup: 'سائن اپ', // Signup
    username: 'صارف نام', // Username
    email: 'ای میل', // Email
    password: 'پاس ورڈ', // Password
    need_account: 'اکاؤنٹ کی ضرورت ہے؟', // Need an account?
    already_account: 'پہلے سے ہی اکاؤنٹ ہے؟', // Already have an account?
    login_successful: 'لاگ ان کامیاب!', // Login successful!
    registration_successful: 'رجسٹریشن کامیاب!', // Registration successful!
    error_occurred: 'ایک خامی پیش آئی۔', // An error occurred.
    network_error: 'نیٹ ورک کی خرابی یا سرور دستیاب نہیں ہے۔', // Network error or server unavailable.
    home: 'ہوم', // Home
    docs: 'دستاویزات', // Docs
    blog: 'بلاگ', // Blog
    dark_mode: 'ڈارک موڈ', // Dark Mode
    light_mode: 'لائٹ موڈ', // Light Mode
    preferences_updated: 'ترجیحات کامیابی سے اپ ڈیٹ ہو گئیں', // Preferences updated successfully
    invalid_input: 'غلط ان پٹ', // Invalid input.
    unauthorized: 'غیر مجاز', // Unauthorized.
  },
};

export function I18nProvider({ children }) {
  const [locale, setLocale] = useState('en'); // Default locale

  const t = (key) => {
    return translations[locale][key] || key;
  };

  const setLanguage = (lang) => {
    setLocale(lang);
    // You might want to save this preference in localStorage or a user profile
  };

  return (
    <i18nContext.Provider value={{ t, locale, setLanguage }}>
      {children}
    </i18nContext.Provider>
  );
}

export function useI18n() {
  const context = useContext(i18nContext);
  if (!context) {
    throw new Error('useI18n must be used within an I18nProvider');
  }
  return context;
}
