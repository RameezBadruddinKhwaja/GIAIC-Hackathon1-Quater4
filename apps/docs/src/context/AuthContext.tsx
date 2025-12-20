import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';

// Define the shape of our auth context
interface AuthContextType {
  isAuthenticated: boolean;
  user: any; // In a real app, you'd define a proper User type
  login: (email: string, password: string) => Promise<boolean>;
  logout: () => void;
  signup: (email: string, password: string, softwareBackground: string, hardwareBackground: string) => Promise<boolean>;
}

// Create the context with a default undefined value
const AuthContext = createContext<AuthContextType | undefined>(undefined);

// Provider component
interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [isAuthenticated, setIsAuthenticated] = useState<boolean>(false);
  const [user, setUser] = useState<any>(null);
  const [loading, setLoading] = useState<boolean>(true);

  // Check if user is authenticated on component mount
  useEffect(() => {
    const token = localStorage.getItem('auth_token');
    const userData = localStorage.getItem('user_data');

    if (token && userData) {
      setIsAuthenticated(true);
      setUser(JSON.parse(userData));
    }

    setLoading(false);
  }, []);

  const login = async (email: string, password: string): Promise<boolean> => {
    try {
      const response = await fetch('/api/auth/signin', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password }),
      });

      if (response.ok) {
        const data = await response.json();
        // Store token and user data in localStorage
        localStorage.setItem('auth_token', data.token);
        localStorage.setItem('user_data', JSON.stringify(data.user));

        setIsAuthenticated(true);
        setUser(data.user);
        return true;
      } else {
        return false;
      }
    } catch (error) {
      console.error('Login error:', error);
      return false;
    }
  };

  const signup = async (
    email: string,
    password: string,
    softwareBackground: string,
    hardwareBackground: string
  ): Promise<boolean> => {
    try {
      const response = await fetch('/api/auth/signup', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email,
          password,
          software_background: softwareBackground,
          hardware_background: hardwareBackground
        }),
      });

      if (response.ok) {
        const data = await response.json();
        // Store token and user data in localStorage
        localStorage.setItem('auth_token', data.token);
        localStorage.setItem('user_data', JSON.stringify(data.user));

        setIsAuthenticated(true);
        setUser(data.user);
        return true;
      } else {
        return false;
      }
    } catch (error) {
      console.error('Signup error:', error);
      return false;
    }
  };

  const logout = () => {
    // Remove token and user data from localStorage
    localStorage.removeItem('auth_token');
    localStorage.removeItem('user_data');

    setIsAuthenticated(false);
    setUser(null);
  };

  // If still loading, you might want to show a loading spinner
  if (loading) {
    return <div>Loading...</div>;
  }

  const value: AuthContextType = {
    isAuthenticated,
    user,
    login,
    logout,
    signup,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

// Custom hook to use the auth context
export const useAuth = (): AuthContextType => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

export default AuthContext;