import React, { useState, useEffect, useRef } from 'react';

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
        const errorData = await response.json().catch(() => ({}));
        throw new Error(`API request failed with status ${response.status}. ${errorData.detail || ''}`);
      }

      const data: ChatResponse = await response.json();

      // Add assistant response to chat
      const assistantMessage: ChatMessage = {
        id: Date.now().toString(),
        role: 'assistant',
        content: data.answer || 'No response received',
        citations: data.citations || [],
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      // Add error message to chat
      const errorMessage: ChatMessage = {
        id: Date.now().toString(),
        role: 'assistant',
        content: `Sorry, I encountered an error processing your request. ${error.message || 'Please try again.'}`,
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
        const errorData = await response.json().catch(() => ({}));
        throw new Error(`API request failed with status ${response.status}. ${errorData.detail || ''}`);
      }

      const data: ChatResponse = await response.json();

      // Add assistant response to chat
      const assistantMessage: ChatMessage = {
        id: Date.now().toString(),
        role: 'assistant',
        content: data.answer || 'No response received',
        citations: data.citations || [],
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending selection-based query:', error);

      // Add error message to chat
      const errorMessage: ChatMessage = {
        id: Date.now().toString(),
        role: 'assistant',
        content: `Sorry, I encountered an error processing your request. ${error.message || 'Please try again.'}`,
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

  return (
    <div className="chat-widget">
      {/* Floating chat button */}
      <button
        className="chat-toggle-btn"
        onClick={() => setIsOpen(!isOpen)}
        title="Chat with the textbook"
      >
        💬
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
        <div className="chat-container" data-theme="light">
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
                <p>Ask me questions about the content, or select text to ask about it.</p>
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
          bottom: 30px;
          right: 30px;
          z-index: 1000;
          font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, 'Open Sans', 'Helvetica Neue', sans-serif;
        }

        .chat-toggle-btn {
          background: linear-gradient(135deg, #4F46E5 0%, #7C3AED 100%);
          color: white;
          border: none;
          border-radius: 50%;
          width: 65px;
          height: 65px;
          font-size: 28px;
          cursor: pointer;
          box-shadow: 0 10px 25px rgba(79, 70, 229, 0.3);
          display: flex;
          align-items: center;
          justify-content: center;
          transition: all 0.3s cubic-bezier(0.25, 0.8, 0.25, 1);
          animation: float 3s ease-in-out infinite;
        }

        .chat-toggle-btn:hover {
          transform: scale(1.1) rotate(5deg);
          box-shadow: 0 15px 35px rgba(79, 70, 229, 0.4);
        }

        @keyframes float {
          0%, 100% { transform: translateY(0px); }
          50% { transform: translateY(-5px); }
        }

        .chat-container {
          position: absolute;
          bottom: 80px;
          right: 0;
          width: 400px;
          max-height: 600px;
          min-height: 400px;
          background: white;
          border-radius: 20px;
          box-shadow: 0 20px 50px rgba(0, 0, 0, 0.15);
          display: flex;
          flex-direction: column;
          overflow: hidden;
          transform: scale(1);
          opacity: 1;
          animation: slideUp 0.4s cubic-bezier(0.25, 0.8, 0.25, 1);
        }

        @keyframes slideUp {
          from {
            transform: translateY(20px) scale(0.95);
            opacity: 0;
          }
          to {
            transform: translateY(0) scale(1);
            opacity: 1;
          }
        }

        .chat-header {
          background: linear-gradient(135deg, #4F46E5 0%, #7C3AED 100%);
          color: white;
          padding: 20px;
          display: flex;
          justify-content: space-between;
          align-items: center;
          box-shadow: 0 4px 10px rgba(0, 0, 0, 0.05);
        }

        .chat-header h3 {
          margin: 0;
          font-size: 18px;
          font-weight: 600;
        }

        .chat-close-btn {
          background: rgba(255, 255, 255, 0.2);
          border: none;
          color: white;
          font-size: 20px;
          cursor: pointer;
          width: 36px;
          height: 36px;
          border-radius: 50%;
          display: flex;
          align-items: center;
          justify-content: center;
          transition: background 0.2s ease;
        }

        .chat-close-btn:hover {
          background: rgba(255, 255, 255, 0.3);
        }

        .chat-messages {
          flex: 1;
          overflow-y: auto;
          padding: 20px;
          background: #f8fafc;
          display: flex;
          flex-direction: column;
          gap: 16px;
        }

        .welcome-message {
          color: #64748b;
          text-align: center;
          padding: 30px 20px;
          font-size: 15px;
        }

        .message {
          max-width: 85%;
          animation: fadeIn 0.3s ease-out;
        }

        @keyframes fadeIn {
          from { opacity: 0; transform: translateY(10px); }
          to { opacity: 1; transform: translateY(0); }
        }

        .message.user {
          align-self: flex-end;
        }

        .message.assistant {
          align-self: flex-start;
        }

        .message-content {
          padding: 16px 20px;
          border-radius: 20px;
          position: relative;
          box-shadow: 0 2px 6px rgba(0, 0, 0, 0.05);
          word-wrap: break-word;
          min-width: 100px;
        }

        .message.user .message-content {
          background: linear-gradient(135deg, #4F46E5 0%, #6366F1 100%);
          color: white;
          border-bottom-right-radius: 5px;
        }

        .message.assistant .message-content {
          background: white;
          border: 1px solid #e2e8f0;
          border-bottom-left-radius: 5px;
          color: #334155;
        }

        .message-timestamp {
          font-size: 11px;
          color: #94a3b8;
          margin-top: 6px;
          text-align: right;
          padding-right: 5px;
        }

        .chat-input-form {
          display: flex;
          padding: 20px;
          background: white;
          border-top: 1px solid #e2e8f0;
          gap: 12px;
        }

        .chat-input-form input {
          flex: 1;
          padding: 16px 20px;
          border: 2px solid #e2e8f0;
          border-radius: 60px;
          font-size: 15px;
          transition: all 0.2s ease;
          outline: none;
        }

        .chat-input-form input:focus {
          border-color: #818cf8;
          box-shadow: 0 0 0 3px rgba(129, 140, 248, 0.2);
        }

        .chat-input-form button {
          background: linear-gradient(135deg, #4F46E5 0%, #6366F1 100%);
          color: white;
          border: none;
          border-radius: 60px;
          padding: 0 25px;
          cursor: pointer;
          font-weight: 500;
          font-size: 15px;
          transition: all 0.2s ease;
          display: flex;
          align-items: center;
          justify-content: center;
        }

        .chat-input-form button:disabled {
          background: #cbd5e1;
          cursor: not-allowed;
        }

        .chat-input-form button:not(:disabled):hover {
          transform: translateY(-2px);
          box-shadow: 0 5px 15px rgba(79, 70, 229, 0.3);
        }

        .selection-prompt {
          position: fixed;
          top: 50%;
          left: 50%;
          transform: translate(-50%, -50%);
          background: white;
          padding: 25px;
          border-radius: 16px;
          box-shadow: 0 25px 50px rgba(0, 0, 0, 0.2);
          z-index: 1001;
          min-width: 450px;
          max-width: 90vw;
          animation: modalAppear 0.3s ease-out;
        }

        @keyframes modalAppear {
          from { opacity: 0; transform: translate(-50%, -40%) scale(0.95); }
          to { opacity: 1; transform: translate(-50%, -50%) scale(1); }
        }

        .selection-content p {
          margin: 0 0 15px 0;
          font-weight: 500;
          color: #334155;
        }

        .selection-content textarea {
          width: 100%;
          margin: 10px 0 15px 0;
          padding: 14px;
          border: 2px solid #e2e8f0;
          border-radius: 12px;
          resize: vertical;
          min-height: 80px;
          font-family: inherit;
          font-size: 15px;
          transition: border-color 0.2s ease;
        }

        .selection-content textarea:focus {
          outline: none;
          border-color: #818cf8;
          box-shadow: 0 0 0 3px rgba(129, 140, 248, 0.2);
        }

        .selection-actions {
          display: flex;
          gap: 12px;
          justify-content: flex-end;
        }

        .selection-actions button {
          padding: 12px 24px;
          border: none;
          border-radius: 8px;
          cursor: pointer;
          font-weight: 500;
          font-size: 14px;
          transition: all 0.2s ease;
        }

        .selection-actions button:first-child {
          background: linear-gradient(135deg, #4F46E5 0%, #6366F1 100%);
          color: white;
        }

        .selection-actions button:last-child {
          background: #f1f5f9;
          color: #64748b;
        }

        .selection-actions button:hover {
          transform: translateY(-2px);
          box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
        }

        .chat-citations {
          margin-top: 12px;
          padding-top: 12px;
          border-top: 1px solid #e2e8f0;
        }

        .citation-title {
          font-weight: 600;
          margin-bottom: 6px;
          font-size: 13px;
          color: #475569;
        }

        .citation-list {
          list-style: none;
          padding: 0;
          margin: 0;
        }

        .citation-item {
          margin-bottom: 6px;
        }

        .citation-item a {
          color: #581c87;
          text-decoration: none;
          font-size: 13px;
          padding: 2px 0;
          display: inline-block;
          transition: color 0.2s ease;
        }

        .citation-item a:hover {
          color: #7e22ce;
          text-decoration: underline;
        }

        /* Dark mode support */
        .chat-container[data-theme="dark"] {
          background-color: #1e293b;
          color: #f1f5f9;
        }

        .chat-container[data-theme="dark"] .chat-messages {
          background-color: #0f172a;
          color: #f1f5f9;
        }

        .chat-container[data-theme="dark"] .message.assistant .message-content {
          background-color: #334155;
          color: #f1f5f9;
          border-color: #475569;
        }

        .chat-container[data-theme="dark"] .welcome-message {
          color: #94a3b8;
        }

        .chat-container[data-theme="dark"] .message-timestamp {
          color: #94a3b8;
        }

        .chat-container[data-theme="dark"] .chat-input-form {
          background-color: #1e293b;
          border-top-color: #334155;
        }

        .chat-container[data-theme="dark"] .chat-input-form input {
          background-color: #334155;
          color: #f1f5f9;
          border-color: #475569;
        }

        .chat-container[data-theme="dark"] .chat-input-form input::placeholder {
          color: #94a3b8;
        }

        .chat-container[data-theme="dark"] .chat-citations {
          border-top-color: #475569;
        }

        .chat-container[data-theme="dark"] .citation-title {
          color: #cbd5e1;
        }

        .chat-container[data-theme="dark"] .citation-item a {
          color: #818cf8;
        }

        .chat-container[data-theme="dark"] .citation-item a:hover {
          color: #60a5fa;
        }

        @media (max-width: 768px) {
          .chat-container {
            width: 95vw;
            max-width: 95vw;
            max-height: 70vh;
            min-height: 50vh;
            right: 2.5vw;
            bottom: 70px;
          }

          .chat-toggle-btn {
            width: 60px;
            height: 60px;
            font-size: 24px;
          }

          .selection-prompt {
            width: 95%;
            min-width: auto;
            padding: 20px;
          }

          .selection-content textarea {
            min-height: 60px;
          }

          .chat-header {
            padding: 16px;
          }

          .chat-header h3 {
            font-size: 16px;
          }

          .chat-messages {
            padding: 16px;
          }

          .chat-input-form {
            padding: 16px;
          }

          .chat-input-form input {
            padding: 14px 16px;
            font-size: 14px;
          }

          .chat-input-form button {
            padding: 0 20px;
          }
        }

        /* Better mobile responsiveness to prevent address bar overlap */
        @media (max-height: 700px) {
          .chat-container {
            max-height: 50vh;
            bottom: 100px;
          }
        }

        @media (max-height: 600px) {
          .chat-container {
            max-height: 40vh;
            bottom: 90px;
          }
        }

        @media (max-height: 500px) {
          .chat-container {
            max-height: 30vh;
            bottom: 80px;
          }
        }
      `}</style>
    </div>
  );
};

export default ChatWidget;