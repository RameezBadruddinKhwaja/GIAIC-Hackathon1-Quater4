import React, { useState, useEffect, useRef } from 'react';
import { useAuth } from '../context/AuthContext'; // Assuming auth context exists

// Define TypeScript interfaces
interface Citation {
  chapter: string;
  section: string;
  url: string;
}

interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  citations?: Citation[];
  timestamp: Date;
}

interface ChatResponse {
  answer: string;
  citations: Citation[];
  session_id: string;
}

const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [showSelectionPrompt, setShowSelectionPrompt] = useState(false);
  const [selectionQuestion, setSelectionQuestion] = useState('');
  const messagesEndRef = useRef<null | HTMLDivElement>(null);
  const { isAuthenticated } = useAuth(); // Use auth context to check if user is logged in

  // Auto-scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection()?.toString().trim();
      if (selectedText && selectedText.length > 10) { // Only show for meaningful selections
        setSelectedText(selectedText);
        setShowSelectionPrompt(true);
        setSelectionQuestion('');
      } else {
        setShowSelectionPrompt(false);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message to chat
    const userMessage: ChatMessage = {
      id: Date.now().toString(),
      role: 'user',
      content: inputValue,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the backend API
      const response = await fetch('/api/chatbot/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question: inputValue,
          session_id: 'current-session-id', // In a real app, this would be managed properly
        }),
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data: ChatResponse = await response.json();

      // Add assistant response to chat
      const assistantMessage: ChatMessage = {
        id: Date.now().toString(),
        role: 'assistant',
        content: data.answer,
        citations: data.citations,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      // Add error message to chat
      const errorMessage: ChatMessage = {
        id: Date.now().toString(),
        role: 'assistant',
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleSelectionSubmit = async () => {
    if (!selectionQuestion.trim() || !selectedText || isLoading) return;

    // Add user message with selected text context
    const userMessage: ChatMessage = {
      id: Date.now().toString(),
      role: 'user',
      content: `Based on: "${selectedText}". ${selectionQuestion}`,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setSelectionQuestion('');
    setShowSelectionPrompt(false);
    setIsLoading(true);

    try {
      // Call the backend API for selection-based query
      const response = await fetch('/api/chatbot/query-selection', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          selected_text: selectedText,
          question: selectionQuestion,
          session_id: 'current-session-id', // In a real app, this would be managed properly
        }),
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data: ChatResponse = await response.json();

      // Add assistant response to chat
      const assistantMessage: ChatMessage = {
        id: Date.now().toString(),
        role: 'assistant',
        content: data.answer,
        citations: data.citations,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending selection-based query:', error);

      // Add error message to chat
      const errorMessage: ChatMessage = {
        id: Date.now().toString(),
        role: 'assistant',
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Function to render citations
  const renderCitations = (citations: Citation[] | undefined) => {
    if (!citations || citations.length === 0) return null;

    return (
      <div className="chat-citations">
        <p className="citation-title">Sources:</p>
        <ul className="citation-list">
          {citations.map((citation, index) => (
            <li key={index} className="citation-item">
              <a href={citation.url} target="_blank" rel="noopener noreferrer">
                {citation.chapter} - {citation.section}
              </a>
            </li>
          ))}
        </ul>
      </div>
    );
  };

  // Function to format timestamp
  const formatTime = (date: Date) => {
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  // If user is not authenticated, show a message
  if (!isAuthenticated) {
    return (
      <div className="chat-widget-unauthenticated">
        <button
          className="chat-toggle-btn"
          onClick={() => setIsOpen(!isOpen)}
          title="Chat with the textbook (requires login)"
        >
          💬 Chat (Login Required)
        </button>

        {isOpen && (
          <div className="chat-container">
            <div className="chat-header">
              <h3>Textbook Assistant</h3>
              <button
                className="chat-close-btn"
                onClick={() => setIsOpen(false)}
              >
                ×
              </button>
            </div>
            <div className="chat-messages">
              <div className="auth-prompt">
                <p>Please sign in to use the chatbot feature.</p>
              </div>
            </div>
          </div>
        )}
      </div>
    );
  }

  return (
    <div className="chat-widget">
      {/* Floating chat button */}
      <button
        className="chat-toggle-btn"
        onClick={() => setIsOpen(!isOpen)}
        title="Chat with the textbook"
      >
        💬 Chat
      </button>

      {/* Selection prompt that appears when text is selected */}
      {showSelectionPrompt && (
        <div className="selection-prompt">
          <div className="selection-content">
            <p>Ask about this selection:</p>
            <textarea
              value={selectionQuestion}
              onChange={(e) => setSelectionQuestion(e.target.value)}
              placeholder="What would you like to know about this text?"
              rows={2}
            />
            <div className="selection-actions">
              <button onClick={handleSelectionSubmit} disabled={isLoading}>
                {isLoading ? 'Asking...' : 'Ask'}
              </button>
              <button onClick={() => setShowSelectionPrompt(false)}>Cancel</button>
            </div>
          </div>
        </div>
      )}

      {/* Chat container */}
      {isOpen && (
        <div className="chat-container">
          <div className="chat-header">
            <h3>Textbook Assistant</h3>
            <button
              className="chat-close-btn"
              onClick={() => setIsOpen(false)}
            >
              ×
            </button>
          </div>

          <div className="chat-messages">
            {messages.length === 0 ? (
              <div className="welcome-message">
                <p>Hello! I'm your Physical AI & Humanoid Robotics textbook assistant.</p>
                <p>Ask me questions about the content, or select text and click "Ask about selection".</p>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`message ${message.role}`}
                >
                  <div className="message-content">
                    <p>{message.content}</p>
                    {message.citations && renderCitations(message.citations)}
                  </div>
                  <div className="message-timestamp">
                    {formatTime(message.timestamp)}
                  </div>
                </div>
              ))
            )}
            <div ref={messagesEndRef} />
          </div>

          <form className="chat-input-form" onSubmit={handleSubmit}>
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask a question about the textbook..."
              disabled={isLoading}
            />
            <button
              type="submit"
              disabled={!inputValue.trim() || isLoading}
            >
              {isLoading ? 'Sending...' : 'Send'}
            </button>
          </form>
        </div>
      )}

      <style jsx>{`
        .chat-widget {
          position: fixed;
          bottom: 20px;
          right: 20px;
          z-index: 1000;
        }

        .chat-toggle-btn {
          background-color: #2563eb;
          color: white;
          border: none;
          border-radius: 50%;
          width: 60px;
          height: 60px;
          font-size: 24px;
          cursor: pointer;
          box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
          display: flex;
          align-items: center;
          justify-content: center;
          transition: all 0.2s ease;
        }

        .chat-toggle-btn:hover {
          transform: scale(1.05);
          box-shadow: 0 6px 16px rgba(0, 0, 0, 0.2);
        }

        .chat-container {
          position: absolute;
          bottom: 80px;
          right: 0;
          width: 400px;
          height: 600px;
          background: white;
          border-radius: 12px;
          box-shadow: 0 10px 25px rgba(0, 0, 0, 0.2);
          display: flex;
          flex-direction: column;
          overflow: hidden;
        }

        .chat-header {
          background-color: #2563eb;
          color: white;
          padding: 16px;
          display: flex;
          justify-content: space-between;
          align-items: center;
        }

        .chat-header h3 {
          margin: 0;
          font-size: 16px;
        }

        .chat-close-btn {
          background: none;
          border: none;
          color: white;
          font-size: 24px;
          cursor: pointer;
          padding: 0;
          width: 30px;
          height: 30px;
          display: flex;
          align-items: center;
          justify-content: center;
        }

        .chat-messages {
          flex: 1;
          overflow-y: auto;
          padding: 16px;
          background-color: #f9fafb;
        }

        .welcome-message {
          color: #6b7280;
          text-align: center;
          padding: 20px 0;
        }

        .message {
          margin-bottom: 16px;
          max-width: 85%;
        }

        .message.user {
          margin-left: auto;
        }

        .message.assistant {
          margin-right: auto;
        }

        .message-content {
          padding: 12px 16px;
          border-radius: 18px;
          position: relative;
        }

        .message.user .message-content {
          background-color: #2563eb;
          color: white;
          border-bottom-right-radius: 4px;
        }

        .message.assistant .message-content {
          background-color: white;
          border: 1px solid #e5e7eb;
          border-bottom-left-radius: 4px;
        }

        .message-timestamp {
          font-size: 12px;
          color: #9ca3af;
          margin-top: 4px;
          text-align: right;
        }

        .chat-input-form {
          display: flex;
          padding: 12px;
          background: white;
          border-top: 1px solid #e5e7eb;
        }

        .chat-input-form input {
          flex: 1;
          padding: 12px 16px;
          border: 1px solid #d1d5db;
          border-radius: 24px;
          margin-right: 8px;
          font-size: 14px;
        }

        .chat-input-form input:focus {
          outline: none;
          border-color: #2563eb;
          box-shadow: 0 0 0 3px rgba(37, 99, 235, 0.1);
        }

        .chat-input-form button {
          background-color: #2563eb;
          color: white;
          border: none;
          border-radius: 24px;
          padding: 12px 20px;
          cursor: pointer;
          font-weight: 500;
        }

        .chat-input-form button:disabled {
          background-color: #9ca3af;
          cursor: not-allowed;
        }

        .selection-prompt {
          position: fixed;
          top: 50%;
          left: 50%;
          transform: translate(-50%, -50%);
          background: white;
          padding: 20px;
          border-radius: 8px;
          box-shadow: 0 10px 25px rgba(0, 0, 0, 0.2);
          z-index: 1001;
          min-width: 400px;
        }

        .selection-content textarea {
          width: 100%;
          margin: 10px 0;
          padding: 10px;
          border: 1px solid #d1d5db;
          border-radius: 4px;
          resize: vertical;
        }

        .selection-actions {
          display: flex;
          gap: 10px;
          justify-content: flex-end;
        }

        .selection-actions button {
          padding: 8px 16px;
          border: none;
          border-radius: 4px;
          cursor: pointer;
        }

        .selection-actions button:first-child {
          background-color: #2563eb;
          color: white;
        }

        .selection-actions button:last-child {
          background-color: #e5e7eb;
          color: #374151;
        }

        .chat-citations {
          margin-top: 8px;
          padding-top: 8px;
          border-top: 1px solid #e5e7eb;
        }

        .citation-title {
          font-weight: 600;
          margin-bottom: 4px;
          font-size: 13px;
          color: #4b5563;
        }

        .citation-list {
          list-style: none;
          padding: 0;
          margin: 0;
        }

        .citation-item {
          margin-bottom: 4px;
        }

        .citation-item a {
          color: #3b82f6;
          text-decoration: none;
          font-size: 13px;
        }

        .citation-item a:hover {
          text-decoration: underline;
        }

        .auth-prompt {
          text-align: center;
          padding: 20px;
          color: #6b7280;
        }

        @media (max-width: 768px) {
          .chat-container {
            width: 90vw;
            max-width: 90vw;
            height: 50vh;
            right: 5vw;
          }

          .selection-prompt {
            width: 90%;
            min-width: auto;
          }
        }
      `}</style>
    </div>
  );
};

export default ChatWidget;