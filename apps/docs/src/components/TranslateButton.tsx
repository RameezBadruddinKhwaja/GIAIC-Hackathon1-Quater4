import React, { useState } from 'react';
import { useLocation } from '@docusaurus/router';
import styles from './TranslateButton.module.css';

interface TranslateButtonProps {
  apiUrl?: string;
}

export function TranslateButton({ apiUrl = 'http://localhost:8000' }: TranslateButtonProps) {
  const [language, setLanguage] = useState<'en' | 'ur'>('en');
  const [loading, setLoading] = useState(false);
  const location = useLocation();

  const toggleLanguage = async () => {
    if (language === 'en') {
      // Switch to Urdu - call backend to translate
      setLoading(true);
      try {
        const currentPath = location.pathname;

        // Fetch current page content
        const response = await fetch(`${apiUrl}/api/translate/page`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            page_path: currentPath,
            target_lang: 'ur',
          }),
        });

        if (response.ok) {
          const data = await response.json();

          // Create Urdu version of page
          const urduPath = currentPath.replace(/\/$/, '') + '/ur';
          window.location.href = urduPath;
          setLanguage('ur');
        } else {
          console.error('Translation failed');
        }
      } catch (error) {
        console.error('Error translating:', error);
      } finally {
        setLoading(false);
      }
    } else {
      // Switch back to English
      const currentPath = location.pathname;
      const enPath = currentPath.replace('/ur', '');
      window.location.href = enPath;
      setLanguage('en');
    }
  };

  return (
    <button
      onClick={toggleLanguage}
      disabled={loading}
      className={styles.translateButton}
      aria-label={language === 'en' ? 'Translate to Urdu' : 'Translate to English'}
    >
      {loading ? (
        <span>⏳</span>
      ) : language === 'en' ? (
        <>
          <span className={styles.icon}>🌐</span>
          <span>اردو میں پڑھیں</span>
        </>
      ) : (
        <>
          <span className={styles.icon}>🌐</span>
          <span>Read in English</span>
        </>
      )}
    </button>
  );
}
