import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../components/AuthProvider';
import { useHistory } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './login.module.css';

export default function Login(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  const API_URL = (siteConfig.customFields?.apiUrl as string) || 'http://localhost:8000';
  const [isSignup, setIsSignup] = useState(false);
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const { signin, signup, isAuthenticated } = useAuth();
  const history = useHistory();

  // Redirect if already authenticated
  React.useEffect(() => {
    if (isAuthenticated) {
      history.push('/');
    }
  }, [isAuthenticated, history]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setIsLoading(true);

    try {
      if (isSignup) {
        await signup(email, password);
        // Redirect to onboarding after signup
        history.push('/onboarding');
      } else {
        await signin(email, password);
        // Redirect to home after signin
        history.push('/');
      }
    } catch (err) {
      setError(err.message || 'Authentication failed');
    } finally {
      setIsLoading(false);
    }
  };

  const handleGitHubLogin = () => {
    // Redirect to GitHub OAuth endpoint
    window.location.href = `${API_URL}/api/auth/github/login`;
  };

  return (
    <Layout title="Sign In" description="Sign in to access personalized features">
      <div className={styles.container}>
        <div className={styles.card}>
          <h1 className={styles.title}>{isSignup ? 'Create Account' : 'Welcome Back'}</h1>
          <p className={styles.subtitle}>
            {isSignup
              ? 'Join the Physical AI learning community'
              : 'Sign in to access your personalized textbook'}
          </p>

          {!isSignup && (
            <>
              <button onClick={handleGitHubLogin} className={styles.githubButton}>
                <svg className={styles.githubIcon} viewBox="0 0 16 16" fill="currentColor">
                  <path d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.69-.94-.09-.23-.48-.94-.82-1.13-.28-.15-.68-.52-.01-.53.63-.01 1.08.58 1.23.82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78-.2-3.64-.89-3.64-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82.64-.18 1.32-.27 2-.27.68 0 1.36.09 2 .27 1.53-1.04 2.2-.82 2.2-.82.44 1.1.16 1.92.08 2.12.51.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.2 0 .21.15.46.55.38A8.013 8.013 0 0016 8c0-4.42-3.58-8-8-8z"/>
                </svg>
                Continue with GitHub
              </button>

              <div className={styles.divider}>
                <span className={styles.dividerText}>or</span>
              </div>
            </>
          )}

          <form onSubmit={handleSubmit} className={styles.form}>
            <div className={styles.formGroup}>
              <label htmlFor="email" className={styles.label}>
                Email Address
              </label>
              <input
                id="email"
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                className={styles.input}
                placeholder="you@example.com"
                required
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="password" className={styles.label}>
                Password
              </label>
              <input
                id="password"
                type="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                className={styles.input}
                placeholder="••••••••"
                minLength={8}
                required
              />
              {isSignup && (
                <p className={styles.hint}>Minimum 8 characters</p>
              )}
            </div>

            {error && <div className={styles.error}>{error}</div>}

            <button type="submit" className={styles.button} disabled={isLoading}>
              {isLoading ? 'Please wait...' : isSignup ? 'Create Account' : 'Sign In'}
            </button>
          </form>

          <div className={styles.toggle}>
            {isSignup ? 'Already have an account?' : "Don't have an account?"}{' '}
            <button
              type="button"
              onClick={() => {
                setIsSignup(!isSignup);
                setError('');
              }}
              className={styles.toggleButton}
            >
              {isSignup ? 'Sign In' : 'Create Account'}
            </button>
          </div>
        </div>
      </div>
    </Layout>
  );
}
