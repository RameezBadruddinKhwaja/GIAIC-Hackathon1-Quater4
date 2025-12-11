import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';
import { AuthProvider } from '@site/src/components/AuthProvider';
import TranslateButton from '@site/src/components/TranslateButton';
import PersonalizeButton from '@site/src/components/PersonalizeButton';

export default function Root({children}) {
  return (
    <AuthProvider>
      {children}
      <ChatWidget />
      <TranslateButton />
      <PersonalizeButton />
    </AuthProvider>
  );
}
