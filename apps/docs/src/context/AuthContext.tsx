import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import axios from 'axios';

// API base URL
const API_URL = process.env.REACT_APP_API_URL || 'https://giaic-hackathon1-quater4.vercel.app';

// User interface
interface User {
  id: number;
  email: string;
  software_level: string;
  hardware_level: string;
  created_at: string;
}

// Auth context interface
interface AuthContextType {
  user: User | null;
  loading: boolean;
  error: string | null;
  signin: (email: string, password: string) => Promise<void>;
  signup: (email: string, password: string, softwareLevel: string, hardwareLevel: string) => Promise<void>;
  signout: () => void;
  loadUser: () => Promise<void>;
}

// Create context
const AuthContext = createContext<AuthContextType | undefined>(undefined);

// Auth provider component
export function AuthProvider({ children }: { children: ReactNode }) {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  // Load user on mount if token exists
  useEffect(() => {
    // Check if there's a token in the URL (from GitHub OAuth)
    const urlParams = new URLSearchParams(window.location.search);
    const tokenFromUrl = urlParams.get('token');

    if (tokenFromUrl) {
      // Store the token and remove it from URL
      localStorage.setItem('auth_token', tokenFromUrl);
      window.history.replaceState({}, document.title, window.location.pathname);
    }

    loadUser();
  }, []);

  const loadUser = async () => {
    const token = localStorage.getItem('auth_token');
    if (!token) {
      setLoading(false);
      return;
    }

    try {
      const response = await axios.get(`${API_URL}/api/auth/me`, {
        headers: {
          'Authorization': `Bearer ${token}`
        }
      });
      setUser(response.data);
    } catch (err) {
      // Token invalid or expired, clear it
      localStorage.removeItem('auth_token');
      setUser(null);
    } finally {
      setLoading(false);
    }
  };

  const signin = async (email: string, password: string) => {
    setError(null);
    try {
      const response = await axios.post(`${API_URL}/api/auth/signin`, {
        email,
        password
      });

      const { access_token } = response.data;
      localStorage.setItem('auth_token', access_token);

      // Load user data
      await loadUser();
    } catch (err: any) {
      setError(err.response?.data?.detail || 'Sign in failed');
      throw err;
    }
  };

  const signup = async (
    email: string,
    password: string,
    softwareLevel: string,
    hardwareLevel: string
  ) => {
    setError(null);
    try {
      const response = await axios.post(`${API_URL}/api/auth/signup`, {
        email,
        password,
        software_level: softwareLevel,
        hardware_level: hardwareLevel
      });

      const { access_token } = response.data;
      localStorage.setItem('auth_token', access_token);

      // Load user data
      await loadUser();
    } catch (err: any) {
      setError(err.response?.data?.detail || 'Sign up failed');
      throw err;
    }
  };

  const signout = () => {
    localStorage.removeItem('auth_token');
    setUser(null);
  };

  return (
    <AuthContext.Provider
      value={{
        user,
        loading,
        error,
        signin,
        signup,
        signout,
        loadUser
      }}
    >
      {children}
    </AuthContext.Provider>
  );
}

// Hook to use auth context
export function useAuth() {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}
