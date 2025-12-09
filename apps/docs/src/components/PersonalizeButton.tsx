import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import styles from './PersonalizeButton.module.css';

interface PersonalizeButtonProps {
  apiUrl?: string;
}

export function PersonalizeButton({ apiUrl = 'http://localhost:8000' }: PersonalizeButtonProps) {
  const [personalized, setPersonalized] = useState(false);
  const [loading, setLoading] = useState(false);
  const [hardwareProfile, setHardwareProfile] = useState<string | null>(null);
  const location = useLocation();

  useEffect(() => {
    // Get user's hardware profile from localStorage
    const profile = localStorage.getItem('hardware_profile');
    setHardwareProfile(profile);
  }, []);

  const togglePersonalization = async () => {
    if (!hardwareProfile) {
      alert('Please complete the hardware onboarding quiz first!');
      window.location.href = '/onboarding';
      return;
    }

    setLoading(true);
    try {
      const currentPath = location.pathname;

      if (!personalized) {
        // Personalize content
        const response = await fetch(`${apiUrl}/api/personalize`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            chapter_id: currentPath,
            hardware_profile: hardwareProfile,
          }),
        });

        if (response.ok) {
          const data = await response.json();

          // Replace content on page (this is a simplified approach)
          // In production, you'd want to navigate to a personalized version
          // or dynamically update the page content

          alert('Content personalized for your hardware!');
          setPersonalized(true);

          // Store personalization state
          sessionStorage.setItem(`personalized_${currentPath}`, 'true');
        } else {
          console.error('Personalization failed');
        }
      } else {
        // Revert to original content
        sessionStorage.removeItem(`personalized_${currentPath}`);
        setPersonalized(false);
        window.location.reload();
      }
    } catch (error) {
      console.error('Error personalizing:', error);
    } finally {
      setLoading(false);
    }
  };

  // Check if already personalized
  useEffect(() => {
    const isPersonalized = sessionStorage.getItem(`personalized_${location.pathname}`) === 'true';
    setPersonalized(isPersonalized);
  }, [location.pathname]);

  if (!hardwareProfile) {
    return null; // Don't show button if no hardware profile set
  }

  return (
    <button
      onClick={togglePersonalization}
      disabled={loading}
      className={`${styles.personalizeButton} ${personalized ? styles.active : ''}`}
      aria-label={personalized ? 'Show original content' : 'Personalize for your hardware'}
    >
      {loading ? (
        <span>⏳</span>
      ) : personalized ? (
        <>
          <span className={styles.icon}>✨</span>
          <span>Original</span>
        </>
      ) : (
        <>
          <span className={styles.icon}>🎯</span>
          <span>Personalize</span>
        </>
      )}
      <span className={styles.badge}>{hardwareProfile === 'rtx_4090' ? 'RTX' : 'Jetson'}</span>
    </button>
  );
}
