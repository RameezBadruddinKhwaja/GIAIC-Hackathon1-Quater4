import React, { useState, useRef, useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useAuth } from '../AuthProvider';
import styles from './styles.module.css';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  citations?: Citation[];
  skills_loaded?: string[];
}

interface Citation {
  chapter_id: string;
  title: string;
  content_snippet: string;
  chapter_url: string;
}

interface ChatResponse {
  response_text: string;
  citations: Citation[];
  skills_loaded: string[];
}

export default function ChatWidget(): JSX.Element | null {
  const { siteConfig } = useDocusaurusContext();
  const { isAuthenticated } = useAuth();
  const API_URL = (siteConfig.customFields?.apiUrl as string) || 'https://giaic-hackathon1-quater4.vercel.app';
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // ChatWidget MUST NOT render if user is not authenticated
  if (!isAuthenticated) {
    return null;
  }

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessage = async () => {
    if (!input.trim() || isLoading) return;

    const userMessage: Message = {
      role: 'user',
      content: input.trim(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const response = await fetch(`${API_URL}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: userMessage.content,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data: ChatResponse = await response.json();

      const assistantMessage: Message = {
        role: 'assistant',
        content: data.response_text,
        citations: data.citations,
        skills_loaded: data.skills_loaded,
      };

      setMessages((prev) => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Chat error:', error);
      const errorMessage: Message = {
        role: 'assistant',
        content: `Sorry, I encountered an error. Please make sure the API server is running on ${API_URL}`,
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <>
      {/* Floating Button */}
      <button
        className={`${styles.floatingButton} ${isOpen ? styles.hidden : ''}`}
        onClick={() => setIsOpen(true)}
        aria-label="Open AI Chat"
      >
        <svg
          width="24"
          height="24"
          viewBox="0 0 24 24"
          fill="none"
          xmlns="http://www.w3.org/2000/svg"
        >
          <path
            d="M12 2C6.48 2 2 6.48 2 12C2 13.93 2.6 15.73 3.61 17.23L2 22L6.77 20.39C8.27 21.4 10.07 22 12 22C17.52 22 22 17.52 22 12C22 6.48 17.52 2 12 2Z"
            fill="currentColor"
          />
        </svg>
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div className={styles.headerContent}>
              <span className={styles.headerTitle}>Matrix AI Tutor</span>
              <span className={styles.headerSubtitle}>Physical AI & Robotics</span>
            </div>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              ✕
            </button>
          </div>

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {messages.length === 0 && (
              <div className={styles.emptyState}>
                <p>👋 Welcome to the Matrix AI Tutor!</p>
                <p>Ask me anything about Physical AI, ROS 2, NVIDIA Isaac, or Humanoid Robotics.</p>
              </div>
            )}
            {messages.map((message, index) => (
              <div
                key={index}
                className={`${styles.message} ${
                  message.role === 'user' ? styles.userMessage : styles.assistantMessage
                }`}
              >
                <div className={styles.messageContent}>
                  {message.content}
                </div>

                {/* Matrix Skills Loaded */}
                {message.skills_loaded && message.skills_loaded.length > 0 && (
                  <div className={styles.skillsLoaded}>
                    <span className={styles.matrixIcon}>⚡</span>
                    Matrix Skills: {message.skills_loaded.join(', ')}
                  </div>
                )}

                {/* Citations */}
                {message.citations && message.citations.length > 0 && (
                  <div className={styles.citations}>
                    <div className={styles.citationsLabel}>📚 Sources:</div>
                    {message.citations.map((citation, idx) => (
                      <a
                        key={idx}
                        href={citation.chapter_url}
                        className={styles.citation}
                        onClick={() => setIsOpen(false)}
                      >
                        <div className={styles.citationTitle}>{citation.title}</div>
                        <div className={styles.citationSnippet}>
                          {citation.content_snippet}
                        </div>
                      </a>
                    ))}
                  </div>
                )}
              </div>
            ))}
            {isLoading && (
              <div className={`${styles.message} ${styles.assistantMessage}`}>
                <div className={styles.loadingDots}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <div className={styles.inputContainer}>
            <textarea
              className={styles.input}
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask about ROS 2, SLAM, Jetson, Isaac Sim..."
              rows={2}
              disabled={isLoading}
            />
            <button
              className={styles.sendButton}
              onClick={sendMessage}
              disabled={!input.trim() || isLoading}
              aria-label="Send message"
            >
              <svg
                width="20"
                height="20"
                viewBox="0 0 24 24"
                fill="none"
                xmlns="http://www.w3.org/2000/svg"
              >
                <path
                  d="M2.01 21L23 12L2.01 3L2 10L17 12L2 14L2.01 21Z"
                  fill="currentColor"
                />
              </svg>
            </button>
          </div>
        </div>
      )}
    </>
  );
}
