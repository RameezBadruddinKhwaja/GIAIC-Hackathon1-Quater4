import React from 'react';
import { AuthProvider } from '@site/src/context/AuthContext';
import ChatWidget from '@site/src/components/ChatWidget';

export default function Root({children}) {
  return (
    <AuthProvider>
      {children}
      <ChatWidget />
    </AuthProvider>
  );
}
