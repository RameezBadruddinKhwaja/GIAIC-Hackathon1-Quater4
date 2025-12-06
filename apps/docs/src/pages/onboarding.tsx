import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../components/AuthProvider';
import { useHistory } from '@docusaurus/router';
import styles from './onboarding.module.css';

export default function Onboarding(): JSX.Element {
  const [hardwareProfile, setHardwareProfile] = useState('');
  const [programmingLanguage, setProgrammingLanguage] = useState('');
  const [error, setError] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const { isAuthenticated, user, submitOnboarding } = useAuth();
  const history = useHistory();

  // Redirect if not authenticated or already completed onboarding
  useEffect(() => {
    if (!isAuthenticated) {
      history.push('/login');
    } else if (user?.hardware_profile && user?.programming_language) {
      history.push('/');
    }
  }, [isAuthenticated, user, history]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');

    if (!hardwareProfile || !programmingLanguage) {
      setError('Please answer both questions');
      return;
    }

    setIsLoading(true);
    try {
      await submitOnboarding(hardwareProfile, programmingLanguage);
      // Redirect to home after successful onboarding
      history.push('/');
    } catch (err) {
      setError(err.message || 'Onboarding failed');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <Layout title="Hardware Onboarding" description="Complete your hardware setup profile">
      <div className={styles.container}>
        <div className={styles.card}>
          <div className={styles.header}>
            <h1 className={styles.title}>Welcome to the Matrix 🤖</h1>
            <p className={styles.subtitle}>
              Let's personalize your learning experience based on your hardware setup
            </p>
          </div>

          <form onSubmit={handleSubmit} className={styles.form}>
            {/* Hardware Profile Question */}
            <div className={styles.question}>
              <label className={styles.questionLabel}>
                1. What hardware will you use for Physical AI development?
              </label>
              <p className={styles.questionHint}>
                Your choice will personalize code examples and performance tips
              </p>

              <div className={styles.optionsGrid}>
                <button
                  type="button"
                  className={`${styles.optionCard} ${
                    hardwareProfile === 'rtx_4090' ? styles.selected : ''
                  }`}
                  onClick={() => setHardwareProfile('rtx_4090')}
                >
                  <div className={styles.optionIcon}>🖥️</div>
                  <div className={styles.optionTitle}>RTX 4090 Digital Twin</div>
                  <div className={styles.optionDescription}>
                    High-fidelity simulation with NVIDIA Isaac Sim
                  </div>
                </button>

                <button
                  type="button"
                  className={`${styles.optionCard} ${
                    hardwareProfile === 'jetson_orin_nano' ? styles.selected : ''
                  }`}
                  onClick={() => setHardwareProfile('jetson_orin_nano')}
                >
                  <div className={styles.optionIcon}>🤖</div>
                  <div className={styles.optionTitle}>Jetson Orin Nano</div>
                  <div className={styles.optionDescription}>
                    Edge deployment for real robots
                  </div>
                </button>
              </div>
            </div>

            {/* Programming Language Question */}
            <div className={styles.question}>
              <label className={styles.questionLabel}>
                2. What's your primary programming language for robotics?
              </label>
              <p className={styles.questionHint}>
                Code examples will be tailored to your language preference
              </p>

              <div className={styles.optionsGrid}>
                <button
                  type="button"
                  className={`${styles.optionCard} ${
                    programmingLanguage === 'python' ? styles.selected : ''
                  }`}
                  onClick={() => setProgrammingLanguage('python')}
                >
                  <div className={styles.optionIcon}>🐍</div>
                  <div className={styles.optionTitle}>Python</div>
                  <div className={styles.optionDescription}>
                    Fast prototyping with ROS 2 Python
                  </div>
                </button>

                <button
                  type="button"
                  className={`${styles.optionCard} ${
                    programmingLanguage === 'cpp' ? styles.selected : ''
                  }`}
                  onClick={() => setProgrammingLanguage('cpp')}
                >
                  <div className={styles.optionIcon}>⚡</div>
                  <div className={styles.optionTitle}>C++</div>
                  <div className={styles.optionDescription}>
                    High performance with ROS 2 C++
                  </div>
                </button>
              </div>
            </div>

            {error && <div className={styles.error}>{error}</div>}

            <button type="submit" className={styles.submitButton} disabled={isLoading}>
              {isLoading ? 'Saving...' : 'Complete Setup →'}
            </button>
          </form>
        </div>
      </div>
    </Layout>
  );
}
