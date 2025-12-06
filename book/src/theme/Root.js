import React from 'react';
import { I18nProvider } from '@site/src/utils/i18n';

// Default implementation, that you might want to customize
export default function Root({ children }) {
  return <I18nProvider>{children}</I18nProvider>;
}
