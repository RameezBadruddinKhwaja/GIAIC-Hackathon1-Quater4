import React from 'react';
import { AuthProvider } from '../context/AuthContext';
import ChatWidget from '../components/ChatWidget';

export default function Root({children}) {
  return (
    <AuthProvider>
      {children}
      <ChatWidget />
    </AuthProvider>
  );
}
