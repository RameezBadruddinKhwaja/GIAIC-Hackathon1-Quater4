import React, { useState, useRef, useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useAuth } from '../../context/AuthContext';
import styles from './styles.module.css';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  citations?: Citation[];
  contextUsed?: number;
}

interface Citation {
  title: string;
  module: string;
  week: string;
  chapter_url: string;
  snippet: string;
}

interface ChatResponse {
  answer: string;
  citations: Citation[];
  model: string;
  context_used: number;
}

export default function ChatWidget(): JSX.Element | null {
  const { siteConfig } = useDocusaurusContext();
  const { user } = useAuth();
  const API_URL = (siteConfig.customFields?.apiUrl as string) || 'https://giaic-hackathon1-quater4.vercel.app';

  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

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
      const response = await fetch(`${API_URL}/api/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: userMessage.content,
          conversation_history: messages.map(m => ({
            role: m.role,
            content: m.content
          }))
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      const data: ChatResponse = await response.json();

      const assistantMessage: Message = {
        role: 'assistant',
        content: data.answer,
        citations: data.citations,
        contextUsed: data.context_used
      };

      setMessages((prev) => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Chat error:', error);

      // Provide user-friendly error messages
      let errorMsg = 'An unexpected error occurred.';
      if (error instanceof Error) {
        errorMsg = error.message;
      } else if (error.message) {
        errorMsg = error.message;
      }

      // Check for specific errors
      if (errorMsg.includes('Failed to fetch') || errorMsg.includes('NetworkError')) {
        errorMsg = 'Unable to connect to the AI server. Please check your internet connection and try again.';
      } else if (errorMsg.includes('OpenAI')) {
        errorMsg = 'The AI service is not configured. Please contact the administrator.';
      } else if (errorMsg.includes('Qdrant')) {
        errorMsg = 'The knowledge base is not configured. Please contact the administrator.';
      } else if (errorMsg.includes('content not yet ingested')) {
        errorMsg = 'The textbook content is still being processed. Please try again in a few minutes.';
      }

      const errorMessage: Message = {
        role: 'assistant',
        content: `❌ ${errorMsg}`,
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

  // Only render if user is authenticated
  if (!user) {
    return null;
  }

  return (
    <>
      {/* Floating Chat Button */}
      <button
        className={`${styles.floatingButton} ${isOpen ? styles.hidden : ''}`}
        onClick={() => setIsOpen(true)}
        aria-label="Open AI Chat Assistant"
        title="Ask AI about Physical AI & Robotics"
      >
        <svg
          width="28"
          height="28"
          viewBox="0 0 24 24"
          fill="none"
          xmlns="http://www.w3.org/2000/svg"
        >
          <path
            d="M12 2C6.48 2 2 6.48 2 12C2 13.93 2.6 15.73 3.61 17.23L2 22L6.77 20.39C8.27 21.4 10.07 22 12 22C17.52 22 22 17.52 22 12C22 6.48 17.52 2 12 2Z"
            fill="currentColor"
          />
          <circle cx="9" cy="12" r="1" fill="white"/>
          <circle cx="12" cy="12" r="1" fill="white"/>
          <circle cx="15" cy="12" r="1" fill="white"/>
        </svg>
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div className={styles.headerContent}>
              <span className={styles.headerTitle}>🤖 AI Tutor</span>
              <span className={styles.headerSubtitle}>Physical AI & Robotics</span>
            </div>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
              title="Close chat"
            >
              ✕
            </button>
          </div>

          {/* Messages Container */}
          <div className={styles.messagesContainer}>
            {messages.length === 0 && (
              <div className={styles.emptyState}>
                <div className={styles.emptyIcon}>💬</div>
                <p className={styles.emptyTitle}>Welcome to AI Tutor!</p>
                <p className={styles.emptyText}>
                  Ask me anything about Physical AI, ROS 2, NVIDIA Isaac, Humanoid Robotics, or any content from the textbook.
                </p>
                <div className={styles.exampleQuestions}>
                  <p className={styles.examplesLabel}>Try asking:</p>
                  <ul>
                    <li>"What is VSLAM?"</li>
                    <li>"Explain ROS 2 nodes and topics"</li>
                    <li>"How does Isaac Sim work?"</li>
                  </ul>
                </div>
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
                  <div className={styles.messageText}>{message.content}</div>

                  {/* Context Badge */}
                  {message.contextUsed !== undefined && message.contextUsed > 0 && (
                    <div className={styles.contextBadge}>
                      📚 {message.contextUsed} source{message.contextUsed > 1 ? 's' : ''} used
                    </div>
                  )}

                  {/* Citations */}
                  {message.citations && message.citations.length > 0 && (
                    <div className={styles.citations}>
                      <div className={styles.citationsLabel}>📖 Sources:</div>
                      <div className={styles.citationsList}>
                        {message.citations.map((citation, idx) => (
                          <a
                            key={idx}
                            href={citation.chapter_url}
                            className={styles.citation}
                            onClick={() => setIsOpen(false)}
                            title={`Go to ${citation.title}`}
                          >
                            <div className={styles.citationHeader}>
                              <span className={styles.citationTitle}>{citation.title}</span>
                              <span className={styles.citationMeta}>
                                {citation.module} › {citation.week}
                              </span>
                            </div>
                            <div className={styles.citationSnippet}>
                              {citation.snippet}
                            </div>
                          </a>
                        ))}
                      </div>
                    </div>
                  )}
                </div>
              </div>
            ))}

            {/* Loading Indicator */}
            {isLoading && (
              <div className={`${styles.message} ${styles.assistantMessage}`}>
                <div className={styles.loadingContainer}>
                  <div className={styles.loadingDots}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                  <span className={styles.loadingText}>Thinking...</span>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <div className={styles.inputContainer}>
            <textarea
              className={styles.input}
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask about ROS 2, Isaac Sim, VSLAM..."
              rows={2}
              disabled={isLoading}
              maxLength={500}
            />
            <button
              className={styles.sendButton}
              onClick={sendMessage}
              disabled={!input.trim() || isLoading}
              aria-label="Send message"
              title="Send message"
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
