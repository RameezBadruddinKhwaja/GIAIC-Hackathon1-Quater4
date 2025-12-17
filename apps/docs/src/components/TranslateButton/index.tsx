import React, { useState } from 'react';
import { useAuth } from '../../context/AuthContext';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

interface TranslateButtonProps {
  onContentUpdate?: (translatedMdx: string) => void;
}

export default function TranslateButton({ onContentUpdate }: TranslateButtonProps): JSX.Element | null {
  const { siteConfig } = useDocusaurusContext();
  const API_URL = (siteConfig.customFields?.apiUrl as string) || 'https://giaic-hackathon1-quater4.vercel.app';
  const { user, token } = useAuth();
  const location = useLocation();
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isTranslated, setIsTranslated] = useState(false);

  // Extract chapter_id from current URL path
  // Example: /modules/week-01-ros2-basics/ → week-01-ros2-basics
  const getChapterId = (): string | null => {
    const path = location.pathname;
    // Match /modules/<chapter-id> or /modules/<chapter-id>/
    const match = path.match(/\/modules\/([^/]+)/);
    if (match && match[1]) {
      // Remove trailing slashes and return clean chapter ID
      return match[1].replace(/\/$/, '');
    }
    return null;
  };

  const handleTranslate = async () => {
    const chapterId = getChapterId();

    // Enhanced logging
    console.log('Current pathname:', location.pathname);
    console.log('Extracted chapter_id:', chapterId);

    if (!chapterId) {
      setError(`Unable to determine chapter ID from URL: ${location.pathname}`);
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      console.log('Sending translation request with chapter_id:', chapterId);

      const response = await fetch(`${API_URL}/api/translate?lang=ur`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          target_language: 'urdu',
        }),
      });

      if (!response.ok) {
        let errorMessage = 'Translation failed';
        try {
          // Clone response to read it twice if needed
          const responseClone = response.clone();
          const responseText = await responseClone.text();
          console.error('API Error Response (raw):', responseText);

          const errorData = await response.json();
          console.error('API Error Response (parsed):', errorData);

          // Extract error message from various possible formats
          errorMessage = errorData.detail ||
                        errorData.message ||
                        errorData.error ||
                        (responseText.length < 200 ? responseText : JSON.stringify(errorData));
        } catch (parseError) {
          // If JSON parsing fails, use status text
          errorMessage = `HTTP ${response.status}: ${response.statusText}`;
          console.error('Failed to parse error response:', parseError);
        }
        throw new Error(errorMessage);
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

      console.log(
        `Translated to Urdu. Cache hit: ${data.cache_hit}`
      );
    } catch (err: any) {
      console.error('Translation error:', err);
      const errorMessage = err?.message || err?.toString() || 'Failed to translate content';
      setError(errorMessage);
    } finally {
      setIsLoading(false);
    }
  };

  const handleRevertToEnglish = () => {
    // Reload page to get original English content
    window.location.reload();
  };

  // Don't show button if not authenticated
  if (!user || !token) {
    return null;
  }

  return (
    <div className={styles.container}>
      {!isTranslated ? (
        <button
          onClick={handleTranslate}
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
              Translate to Urdu
            </>
          )}
        </button>
      ) : (
        <div className={styles.translatedState}>
          <div className={styles.languageBadge}>
            <span className={styles.checkmark}>✓</span>
            Urdu (اردو)
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

      {error && (
        <div className={styles.toast + ' ' + styles.error}>
          <span className={styles.toastIcon}>✗</span>
          {error}
        </div>
      )}
    </div>
  );
}
