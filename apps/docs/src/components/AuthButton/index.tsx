import React from 'react';
import { useAuth } from '../AuthProvider';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

export default function AuthButton(): JSX.Element {
  const { isAuthenticated, user, signout } = useAuth();

  if (isAuthenticated && user) {
    return (
      <div className={styles.userMenu}>
        <span className={styles.userEmail}>{user.email}</span>
        <button onClick={signout} className={styles.signoutButton}>
          Sign Out
        </button>
      </div>
    );
  }

  return (
    <Link to="/login" className={styles.signinLink}>
      <button className={styles.signinButton}>Sign In</button>
    </Link>
  );
}
