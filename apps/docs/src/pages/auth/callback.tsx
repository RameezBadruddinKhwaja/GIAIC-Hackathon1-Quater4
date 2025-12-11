import React, { useEffect } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import Layout from '@theme/Layout';
import { useAuth } from '@site/src/components/AuthProvider';

export default function AuthCallback(): JSX.Element {
  const history = useHistory();
  const location = useLocation();
  const { isAuthenticated } = useAuth();

  useEffect(() => {
    const params = new URLSearchParams(location.search);
    const token = params.get('token');
    const userJson = params.get('user');

    if (token && userJson) {
      try {
        const user = JSON.parse(decodeURIComponent(userJson));

        localStorage.setItem('auth_token', token);
        localStorage.setItem('auth_user', JSON.stringify(user));

        if (!user.hardware_profile || !user.programming_language) {
          history.push('/onboarding');
        } else {
          history.push('/');
        }
      } catch (error) {
        console.error('Error parsing auth callback:', error);
        history.push('/login?error=invalid_callback');
      }
    } else if (isAuthenticated) {
      history.push('/');
    } else {
      history.push('/login?error=no_token');
    }
  }, [location, history, isAuthenticated]);

  return (
    <Layout title="Authenticating..." description="Completing authentication">
      <div style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        minHeight: '60vh',
        fontSize: '1.2rem'
      }}>
        <div>
          <div style={{ marginBottom: '1rem' }}>Completing authentication...</div>
          <div style={{ textAlign: 'center' }}>
            <span style={{
              display: 'inline-block',
              width: '30px',
              height: '30px',
              border: '3px solid #f3f3f3',
              borderTop: '3px solid #3498db',
              borderRadius: '50%',
              animation: 'spin 1s linear infinite'
            }}></span>
          </div>
        </div>
      </div>
      <style>{`
        @keyframes spin {
          0% { transform: rotate(0deg); }
          100% { transform: rotate(360deg); }
        }
      `}</style>
    </Layout>
  );
}
