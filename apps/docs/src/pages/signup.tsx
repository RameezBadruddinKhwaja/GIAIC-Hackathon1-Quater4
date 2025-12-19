import React, { useState } from 'react';
import { useHistory } from '@docusaurus/router';
import { useAuth } from '../context/AuthContext';
import Layout from '@theme/Layout';
import styles from './signup.module.css';

export default function SignUp(): JSX.Element {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [softwareLevel, setSoftwareLevel] = useState('');
  const [hardwareLevel, setHardwareLevel] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const { signup } = useAuth();
  const history = useHistory();

  const validateEmail = (email: string): boolean => {
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    return emailRegex.test(email);
  };

  const validatePassword = (password: string): string | null => {
    if (password.length < 8) {
      return 'Password must be at least 8 characters';
    }
    if (!/[A-Za-z]/.test(password)) {
      return 'Password must contain at least one letter';
    }
    if (!/\d/.test(password)) {
      return 'Password must contain at least one number';
    }
    return null;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');

    // Client-side validation
    if (!validateEmail(email)) {
      setError('Please enter a valid email address');
      return;
    }

    const passwordError = validatePassword(password);
    if (passwordError) {
      setError(passwordError);
      return;
    }

    if (!softwareLevel) {
      setError('Please select your software experience level');
      return;
    }

    if (!hardwareLevel) {
      setError('Please select your hardware experience level');
      return;
    }

    setLoading(true);

    try {
      await signup(email, password, softwareLevel, hardwareLevel);
      // Redirect to homepage on success
      history.push('/');
    } catch (err: any) {
      // Handle errors from API
      if (err.response?.status === 409) {
        setError('Email already registered. Please sign in or use a different email.');
      } else if (err.response?.status === 400) {
        setError(err.response?.data?.detail || 'Invalid input. Please check your details.');
      } else if (err.message === 'Network Error') {
        setError('Connection error. Please check your internet connection.');
      } else {
        setError('Something went wrong. Please try again.');
      }
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="Sign Up" description="Create your account">
      <div className={styles.container}>
        <div className={styles.card}>
          <h1 className={styles.title}>Create Account</h1>
          <p className={styles.subtitle}>
            Join us to access personalized learning features!
          </p>

          <form onSubmit={handleSubmit} className={styles.form}>
            {error && <div className={styles.error}>{error}</div>}

            <div className={styles.formGroup}>
              <label htmlFor="email" className={styles.label}>
                Email
              </label>
              <input
                id="email"
                type="email"
                value={email}
                onChange={(e) => {
                  setEmail(e.target.value);
                  setError('');
                }}
                className={styles.input}
                placeholder="your@email.com"
                required
                disabled={loading}
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
                onChange={(e) => {
                  setPassword(e.target.value);
                  setError('');
                }}
                className={styles.input}
                placeholder="••••••••"
                required
                disabled={loading}
                minLength={8}
              />
              <p className={styles.hint}>
                At least 8 characters with one letter and one number
              </p>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="software-level" className={styles.label}>
                Software Experience Level
              </label>
              <select
                id="software-level"
                value={softwareLevel}
                onChange={(e) => {
                  setSoftwareLevel(e.target.value);
                  setError('');
                }}
                className={styles.select}
                required
                disabled={loading}
              >
                <option value="">Select your level</option>
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="hardware-level" className={styles.label}>
                Hardware Experience Level
              </label>
              <select
                id="hardware-level"
                value={hardwareLevel}
                onChange={(e) => {
                  setHardwareLevel(e.target.value);
                  setError('');
                }}
                className={styles.select}
                required
                disabled={loading}
              >
                <option value="">Select your level</option>
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>
            </div>

            <button
              type="submit"
              className={styles.submitButton}
              disabled={loading}
            >
              {loading ? 'Creating Account...' : 'Create Account'}
            </button>
          </form>

          <p className={styles.switchText}>
            Already have an account?{' '}
            <a href="/signin" className={styles.link}>
              Sign in
            </a>
          </p>
        </div>
      </div>
    </Layout>
  );
}
