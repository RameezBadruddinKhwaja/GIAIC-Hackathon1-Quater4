import React, { useState } from 'react';
import { useAuth } from '../AuthProvider';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

interface PersonalizeButtonProps {
  onContentUpdate?: (personalizedMdx: string) => void;
}

export default function PersonalizeButton({ onContentUpdate }: PersonalizeButtonProps): JSX.Element | null {
  const { siteConfig } = useDocusaurusContext();
  const API_URL = (siteConfig.customFields?.apiUrl as string) || 'https://giaic-hackathon1-quater4.vercel.app';
  const { isAuthenticated, user, token } = useAuth();
  const location = useLocation();
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState(false);
  const [isPersonalized, setIsPersonalized] = useState(false);

  // Don't show button if not authenticated or user has no hardware profile
  if (!isAuthenticated || !user?.hardware_profile) {
    return null;
  }

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

  const handlePersonalize = async () => {
    const chapterId = getChapterId();
    if (!chapterId) {
      setError('Unable to determine chapter ID');
      return;
    }

    setIsLoading(true);
    setError(null);
    setSuccess(false);

    try {
      const response = await fetch(`${API_URL}/api/personalize`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify({
          chapter_id: chapterId,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Personalization failed');
      }

      const data = await response.json();

      // Update DOM with personalized content
      if (data.personalized_mdx) {
        // Find the main article content
        const articleContent = document.querySelector('article.markdown');
        if (articleContent) {
          // Create a temporary div to parse the markdown
          const tempDiv = document.createElement('div');
          tempDiv.innerHTML = data.personalized_mdx;

          // Replace the article content
          articleContent.innerHTML = tempDiv.innerHTML;
        }
      }

      // Call parent callback if provided
      if (onContentUpdate) {
        onContentUpdate(data.personalized_mdx);
      }

      setSuccess(true);
      setIsPersonalized(true);
      setTimeout(() => setSuccess(false), 3000);

      console.log(
        `Personalized for ${data.hardware_profile}. Cache hit: ${data.cache_hit}`
      );
    } catch (err) {
      console.error('Personalization error:', err);
      setError(err.message || 'Failed to personalize content');
    } finally {
      setIsLoading(false);
    }
  };

  // Get hardware label
  const getHardwareLabel = (): string => {
    if (user.hardware_profile === 'rtx_4090') {
      return 'RTX 4090';
    } else if (user.hardware_profile === 'jetson_orin_nano') {
      return 'Jetson Orin Nano';
    }
    return 'Your Hardware';
  };

  return (
    <div className={styles.container}>
      <button
        onClick={handlePersonalize}
        disabled={isLoading || isPersonalized}
        className={`${styles.button} ${isPersonalized ? styles.personalized : ''}`}
      >
        {isLoading ? (
          <>
            <span className={styles.spinner}></span>
            Personalizing...
          </>
        ) : isPersonalized ? (
          <>
            <span className={styles.checkmark}>✓</span>
            Personalized for {getHardwareLabel()}
          </>
        ) : (
          <>
            <span className={styles.icon}>🎯</span>
            Personalize for {getHardwareLabel()}
          </>
        )}
      </button>

      {error && (
        <div className={styles.toast + ' ' + styles.error}>
          <span className={styles.toastIcon}>✗</span>
          {error}
        </div>
      )}

      {success && (
        <div className={styles.toast + ' ' + styles.success}>
          <span className={styles.toastIcon}>✓</span>
          Content personalized successfully!
        </div>
      )}
    </div>
  );
}
