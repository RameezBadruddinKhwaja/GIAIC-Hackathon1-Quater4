/**
 * Auth Button Component
 *
 * Displays Sign In/Sign Up buttons or user info based on auth state.
 */

import React, { useState, useRef, useEffect } from 'react';
import { useAuth } from '../../context/AuthContext';
import styles from './AuthButton.module.css';

export default function AuthButton() {
  const { user, loading, signout } = useAuth();
  const [showMenu, setShowMenu] = useState(false);
  const menuRef = useRef<HTMLDivElement>(null);

  // Close menu when clicking outside
  useEffect(() => {
    function handleClickOutside(event: MouseEvent) {
      if (menuRef.current && !menuRef.current.contains(event.target as Node)) {
        setShowMenu(false);
      }
    }

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  if (loading) {
    return <div className={styles.loading}>Loading...</div>;
  }

  if (user) {
    return (
      <div className={styles.userMenuContainer} ref={menuRef}>
        <button
          className={styles.userButton}
          onClick={() => setShowMenu(!showMenu)}
          aria-expanded={showMenu}
          aria-haspopup="true"
        >
          {user.email.split('@')[0]}
        </button>

        {showMenu && (
          <div className={styles.userMenu}>
            <div className={styles.userInfo}>
              <div className={styles.userName}>{user.email}</div>
              <div className={styles.userLevel}>
                Software: {user.software_level}
              </div>
              <div className={styles.userLevel}>
                Hardware: {user.hardware_level}
              </div>
            </div>
            <button
              className={styles.signoutButton}
              onClick={() => {
                signout();
                setShowMenu(false);
              }}
            >
              Sign Out
            </button>
          </div>
        )}
      </div>
    );
  }

  return (
    <div className={styles.authButtons}>
      <a href="/signin" className={styles.signinButton}>
        Sign In
      </a>
      <a href="/signup" className={styles.signupButton}>
        Sign Up
      </a>
    </div>
  );
}
