import React from 'react';
import { I18nProvider } from '@site/src/utils/i18n';
import ChapterControls from '@site/src/components/ChapterControls';
import ChatWidget from '@site/src/components/ChatWidget';

// Default implementation, that you might want to customize
export default function Root({ children }) {
  return (
    <I18nProvider>
      <div className="root-fade-in">{children}</div>
      <ChapterControls />
      <ChatWidget />
    </I18nProvider>
  );
}