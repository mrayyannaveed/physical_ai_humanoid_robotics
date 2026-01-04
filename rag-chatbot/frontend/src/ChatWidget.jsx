import React, { useState, useEffect, useRef } from 'react';
import './ChatWidget.css';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [isUsingSelectedText, setIsUsingSelectedText] = useState(false);
  const messagesEndRef = useRef(null);
  const textareaRef = useRef(null);

  // Function to get selected text from the page
  const getSelectedText = () => {
    const selectedText = window.getSelection().toString().trim();
    return selectedText;
  };

  // Handle text selection on the page
  useEffect(() => {
    const handleSelection = () => {
      const text = getSelectedText();
      if (text) {
        setSelectedText(text);
        setIsUsingSelectedText(true);
      } else {
        setSelectedText('');
        setIsUsingSelectedText(false);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('selectionchange', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('selectionchange', handleSelection);
    };
  }, []);

  // Auto-scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Toggle chat widget open/close
  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  // Handle sending a message
  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = { id: Date.now(), text: inputValue, sender: 'user' };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Prepare the request payload
      const payload = {
        message: inputValue,
        selected_text_context: isUsingSelectedText ? selectedText : null,
        history: messages.map(msg => ({ text: msg.text, sender: msg.sender }))
      };

      // Call the backend API
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload)
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();

      const botMessage = {
        id: Date.now() + 1,
        text: data.response,
        sender: 'bot',
        sources: data.sources || []
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error. Please try again.',
        sender: 'bot'
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Handle key press (Enter to send)
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  // Clear selected text context
  const clearSelectedText = () => {
    setSelectedText('');
    setIsUsingSelectedText(false);
  };

  return (
    <>
      {/* Chat Widget Button */}
      {!isOpen && (
        <button
          className="chat-widget-button"
          onClick={toggleChat}
          aria-label="Open chat"
        >
          💬
        </button>
      )}

      {/* Chat Widget */}
      {isOpen && (
        <div className="chat-widget">
          <div className="chat-header">
            <div className="chat-title">Book Assistant</div>
            <button
              className="chat-close-button"
              onClick={toggleChat}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>

          {/* Selected Text Indicator */}
          {isUsingSelectedText && selectedText && (
            <div className="selected-text-indicator">
              <div className="indicator-content">
                <span className="indicator-label">Using selected text:</span>
                <span className="indicator-text">
                  "{selectedText.length > 100 ? selectedText.substring(0, 100) + '...' : selectedText}"
                </span>
                <button
                  className="clear-selection-button"
                  onClick={clearSelectedText}
                  title="Clear selection"
                >
                  ×
                </button>
              </div>
            </div>
          )}

          <div className="chat-messages">
            {messages.length === 0 ? (
              <div className="chat-welcome">
                <p>Hello! I'm your book assistant.</p>
                <p>Select text on the page to ask questions about it, or ask me anything about the book.</p>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`message ${message.sender}-message`}
                >
                  <div className="message-content">
                    {message.text}
                  </div>
                  {message.sources && message.sources.length > 0 && (
                    <div className="message-sources">
                      <details>
                        <summary>Sources</summary>
                        {message.sources.map((source, idx) => (
                          <div key={idx} className="source-item">
                            <div className="source-text">{source.text}</div>
                            <div className="source-meta">
                              Chapter: {source.chapter} | Score: {source.relevance_score.toFixed(2)}
                            </div>
                          </div>
                        ))}
                      </details>
                    </div>
                  )}
                </div>
              ))
            )}
            {isLoading && (
              <div className="message bot-message">
                <div className="message-content">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chat-input-area">
            <textarea
              ref={textareaRef}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder={isUsingSelectedText
                ? "Ask about the selected text..."
                : "Ask me anything about the book..."}
              className="chat-input"
              rows="1"
            />
            <button
              onClick={handleSendMessage}
              disabled={!inputValue.trim() || isLoading}
              className="chat-send-button"
            >
              {isLoading ? 'Sending...' : 'Send'}
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default ChatWidget;