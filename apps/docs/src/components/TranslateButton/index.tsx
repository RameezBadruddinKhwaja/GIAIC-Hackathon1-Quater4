import React, { useState } from 'react';
import { useAuth } from '../AuthProvider';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

interface TranslateButtonProps {
  onContentUpdate?: (translatedMdx: string) => void;
}

export default function TranslateButton({ onContentUpdate }: TranslateButtonProps): JSX.Element | null {
  const { siteConfig } = useDocusaurusContext();
  const API_URL = (siteConfig.customFields?.apiUrl as string) || 'http://localhost:8000';
  const { isAuthenticated, token } = useAuth();
  const location = useLocation();
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isTranslated, setIsTranslated] = useState(false);
  const [currentLanguage, setCurrentLanguage] = useState<'english' | 'roman_urdu' | 'formal_urdu'>('english');

  // Don't show button if not authenticated
  if (!isAuthenticated) {
    return null;
  }

  // Extract chapter_id from current URL path
  // Example: /modules/module-1/ros2-fundamentals → module-1/ros2-fundamentals
  const getChapterId = (): string | null => {
    const path = location.pathname;
    const match = path.match(/\/modules\/(.+)/);
    return match ? match[1] : null;
  };

  const handleTranslate = async (targetLanguage: 'roman_urdu' | 'formal_urdu') => {
    const chapterId = getChapterId();
    if (!chapterId) {
      setError('Unable to determine chapter ID');
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_URL}/api/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          target_language: targetLanguage,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Translation failed');
      }

      const data = await response.json();

      // Update DOM with translated content
      if (data.translated_mdx) {
        // Find the main article content
        const articleContent = document.querySelector('article.markdown');
        if (articleContent) {
          // Create a temporary div to parse the markdown
          const tempDiv = document.createElement('div');
          tempDiv.innerHTML = data.translated_mdx;

          // Replace the article content
          articleContent.innerHTML = tempDiv.innerHTML;
        }
      }

      // Call parent callback if provided
      if (onContentUpdate) {
        onContentUpdate(data.translated_mdx);
      }

      setIsTranslated(true);
      setCurrentLanguage(targetLanguage);

      console.log(
        `Translated to ${targetLanguage}. Cache hit: ${data.cache_hit}`
      );
    } catch (err) {
      console.error('Translation error:', err);
      setError(err.message || 'Failed to translate content');
    } finally {
      setIsLoading(false);
    }
  };

  const handleRevertToEnglish = () => {
    // Reload page to get original English content
    window.location.reload();
  };

  return (
    <div className={styles.container}>
      <div className={styles.buttonGroup}>
        {!isTranslated ? (
          <>
            <button
              onClick={() => handleTranslate('roman_urdu')}
              disabled={isLoading}
              className={styles.button}
            >
              {isLoading ? (
                <>
                  <span className={styles.spinner}></span>
                  Translating...
                </>
              ) : (
                <>
                  <span className={styles.icon}>🇵🇰</span>
                  Translate to Roman Urdu
                </>
              )}
            </button>

            <button
              onClick={() => handleTranslate('formal_urdu')}
              disabled={isLoading}
              className={styles.button}
            >
              {isLoading ? (
                <>
                  <span className={styles.spinner}></span>
                  Translating...
                </>
              ) : (
                <>
                  <span className={styles.icon}>اردو</span>
                  Translate to Formal Urdu
                </>
              )}
            </button>
          </>
        ) : (
          <div className={styles.translatedState}>
            <div className={styles.languageBadge}>
              <span className={styles.checkmark}>✓</span>
              {currentLanguage === 'roman_urdu' ? 'Roman Urdu' : 'Formal Urdu (اردو)'}
            </div>
            <button
              onClick={handleRevertToEnglish}
              className={styles.revertButton}
            >
              <span className={styles.icon}>🔄</span>
              Back to English
            </button>
          </div>
        )}
      </div>

      {error && (
        <div className={styles.toast + ' ' + styles.error}>
          <span className={styles.toastIcon}>✗</span>
          {error}
        </div>
      )}
    </div>
  );
}
