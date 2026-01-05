import React, { useState, useEffect, useRef } from 'react';
import './ChatWidget.css';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [isContextActive, setIsContextActive] = useState(false);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Function to get selected text from the page
  const getSelectedText = () => {
    const selection = window.getSelection();
    if (selection && selection.toString().trim()) {
      return selection.toString().trim();
    }
    return '';
  };

  // Handle text selection on the page
  useEffect(() => {
    const handleSelection = () => {
      const text = getSelectedText();
      if (text) {
        setSelectedText(text);
        setIsContextActive(true);
      } else {
        setSelectedText('');
        setIsContextActive(false);
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
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Toggle chat widget open/close
  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      setTimeout(() => {
        inputRef.current?.focus();
      }, 100);
    }
  };

  // Handle sending a message
  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now().toString(),
      text: inputValue,
      sender: 'user'
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Prepare the request payload
      const payload = {
        message: inputValue,
        selected_text: isContextActive ? selectedText : null
      };

      // Call the backend API
      const response = await fetch('http://127.0.0.1:8000/chat', {
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
        id: (Date.now() + 1).toString(),
        text: data.response,
        sender: 'bot'
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: (Date.now() + 1).toString(),
        text: 'Sorry, I encountered an error. Please try again.',
        sender: 'bot'
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Handle key press (Enter to send, Shift+Enter for new line)
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  // Clear selected text context
  const clearSelectedText = () => {
    setSelectedText('');
    setIsContextActive(false);
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

          {/* Context Indicator */}
          {isContextActive && selectedText && (
            <div className="context-indicator">
              <div className="indicator-content">
                <span className="indicator-label">Context: Selection Active</span>
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
                <p>ask anything about the book.</p>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`message ${message.sender}-message`}
                >
                  <div className="message-content">{message.text}</div>
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
              ref={inputRef}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder={isContextActive
                ? "Ask about the selected text..."
                : "Ask anything about the book..."}
              className="chat-input"
              rows={1}
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