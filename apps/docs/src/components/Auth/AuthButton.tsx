/**
 * Auth Button Component
 * 
 * Displays Sign In/Sign Up buttons or user menu based on auth state.
 */

import React, { useState } from 'react';
import { useAuth } from '../../context/AuthContext';
import SignInModal from './SignInModal';
import SignUpModal from './SignUpModal';
import styles from './styles.module.css';

export default function AuthButton() {
  const { user, loading, logout } = useAuth();
  const [showSignIn, setShowSignIn] = useState(false);
  const [showSignUp, setShowSignUp] = useState(false);
  const [showUserMenu, setShowUserMenu] = useState(false);

  if (loading) {
    return <div className={styles.authButton}>Loading...</div>;
  }

  if (user) {
    return (
      <div className={styles.userMenuContainer}>
        <button
          className={styles.userButton}
          onClick={() => setShowUserMenu(!showUserMenu)}
        >
          {user.username}
        </button>
        
        {showUserMenu && (
          <div className={styles.userMenu}>
            <div className={styles.userInfo}>
              <div className={styles.userName}>{user.full_name || user.username}</div>
              <div className={styles.userEmail}>{user.email}</div>
            </div>
            <button
              className={styles.logoutButton}
              onClick={() => {
                logout();
                setShowUserMenu(false);
              }}
            >
              Logout
            </button>
          </div>
        )}
      </div>
    );
  }

  return (
    <>
      <div className={styles.authButtons}>
        <button
          className={styles.signInButton}
          onClick={() => setShowSignIn(true)}
        >
          Sign In
        </button>
        <button
          className={styles.signUpButton}
          onClick={() => setShowSignUp(true)}
        >
          Sign Up
        </button>
      </div>

      <SignInModal
        isOpen={showSignIn}
        onClose={() => setShowSignIn(false)}
        onSwitchToSignUp={() => {
          setShowSignIn(false);
          setShowSignUp(true);
        }}
      />

      <SignUpModal
        isOpen={showSignUp}
        onClose={() => setShowSignUp(false)}
        onSwitchToSignIn={() => {
          setShowSignUp(false);
          setShowSignIn(true);
        }}
      />
    </>
  );
}
