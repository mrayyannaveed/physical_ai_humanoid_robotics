import React from 'react';
import { I18nProvider } from '@site/src/utils/i18n';
import ChapterControls from '@site/src/components/ChapterControls';

// Default implementation, that you might want to customize
export default function Root({ children }) {
  return (
    <I18nProvider>
      <div className="root-fade-in">{children}</div>
      <ChapterControls />
    </I18nProvider>
  );
}