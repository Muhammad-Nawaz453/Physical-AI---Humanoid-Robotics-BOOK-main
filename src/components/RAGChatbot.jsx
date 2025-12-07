import { useState, useRef, useEffect } from 'react';
import { MessageSquare, Send, X, Sparkles } from 'lucide-react';
import API_URL from '../config';
export default function RAGChatbot() {
    const [isOpen, setIsOpen] = useState(false);
    const [messages, setMessages] = useState([
        { role: 'assistant', content: 'Hi! I can help you with questions about Physical AI & Humanoid Robotics. Ask me anything!' }
    ]);
    const [input, setInput] = useState('');
    const [loading, setLoading] = useState(false);
    const [selectedText, setSelectedText] = useState('');
    const messagesEndRef = useRef(null);

    useEffect(() => {
        messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    }, [messages]);

    useEffect(() => {
        const handleSelection = () => {
            const selection = window.getSelection()?.toString().trim();
            if (selection && selection.length > 10) {
                setSelectedText(selection);
            }
        };

        document.addEventListener('mouseup', handleSelection);
        return () => document.removeEventListener('mouseup', handleSelection);
    }, []);

    const sendMessage = async () => {
        if (!input.trim() && !selectedText) return;

        const userMessage = input.trim() || `Explain this: "${selectedText}"`;
        setMessages(prev => [...prev, { role: 'user', content: userMessage }]);
        setInput('');
        setLoading(true);

        try {
            const response = await fetch(`${API_URL}/api/chatbot`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    question: userMessage,
                    selectedText: selectedText
                })
            });

            const data = await response.json();
            setMessages(prev => [...prev, {
                role: 'assistant',
                content: data.answer || 'Sorry, I could not process that question.'
            }]);
        } catch (error) {
            console.error('Chatbot error:', error);
            setMessages(prev => [...prev, {
                role: 'assistant',
                content: 'Error connecting to the chatbot. Make sure the backend is running on port 8000.'
            }]);
        } finally {
            setLoading(false);
            setSelectedText('');
        }
    };

    const handleKeyPress = (e) => {
        if (e.key === 'Enter' && !e.shiftKey) {
            e.preventDefault();
            sendMessage();
        }
    };

    return (
        <>
            {/* Floating Chat Button */}
            <button
                onClick={() => setIsOpen(!isOpen)}
                style={{
                    position: 'fixed',
                    bottom: '24px',
                    right: '24px',
                    backgroundColor: '#2563eb',
                    color: 'white',
                    padding: '16px',
                    borderRadius: '9999px',
                    boxShadow: '0 10px 15px rgba(0,0,0,0.3)',
                    border: 'none',
                    cursor: 'pointer',
                    zIndex: 50,
                    transition: 'all 0.3s',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center'
                }}
                onMouseOver={(e) => {
                    e.currentTarget.style.backgroundColor = '#1d4ed8';
                    e.currentTarget.style.transform = 'scale(1.1)';
                }}
                onMouseOut={(e) => {
                    e.currentTarget.style.backgroundColor = '#2563eb';
                    e.currentTarget.style.transform = 'scale(1)';
                }}
            >
                {isOpen ? <X size={24} /> : <MessageSquare size={24} />}
            </button>

            {/* Selected Text Popup */}
            {selectedText && !isOpen && (
                <div style={{
                    position: 'fixed',
                    bottom: '96px',
                    right: '24px',
                    backgroundColor: 'white',
                    borderRadius: '12px',
                    boxShadow: '0 10px 25px rgba(0,0,0,0.2)',
                    padding: '12px',
                    zIndex: 50,
                    maxWidth: '300px'
                }}>
                    <p style={{
                        fontSize: '0.875rem',
                        color: '#666',
                        marginBottom: '8px'
                    }}>
                        Ask about selected text?
                    </p>
                    <button
                        onClick={() => {
                            setIsOpen(true);
                            setInput(`Explain this: "${selectedText.substring(0, 100)}..."`);
                        }}
                        style={{
                            width: '100%',
                            backgroundColor: '#2563eb',
                            color: 'white',
                            padding: '8px 12px',
                            borderRadius: '6px',
                            fontSize: '0.875rem',
                            border: 'none',
                            cursor: 'pointer',
                            display: 'flex',
                            alignItems: 'center',
                            justifyContent: 'center',
                            gap: '8px'
                        }}
                        onMouseOver={(e) => e.currentTarget.style.backgroundColor = '#1d4ed8'}
                        onMouseOut={(e) => e.currentTarget.style.backgroundColor = '#2563eb'}
                    >
                        <Sparkles size={16} />
                        Ask AI
                    </button>
                </div>
            )}

            {/* Chat Window */}
            {isOpen && (
                <div style={{
                    position: 'fixed',
                    bottom: '96px',
                    right: '24px',
                    width: '384px',
                    height: '600px',
                    backgroundColor: 'white',
                    borderRadius: '16px',
                    boxShadow: '0 20px 60px rgba(0,0,0,0.3)',
                    display: 'flex',
                    flexDirection: 'column',
                    zIndex: 50,
                    border: '1px solid #e5e7eb'
                }}>
                    {/* Header */}
                    <div style={{
                        background: 'linear-gradient(135deg, #2563eb 0%, #7c3aed 100%)',
                        color: 'white',
                        padding: '16px',
                        borderTopLeftRadius: '16px',
                        borderTopRightRadius: '16px'
                    }}>
                        <h3 style={{
                            fontWeight: 'bold',
                            fontSize: '1.125rem',
                            margin: 0
                        }}>
                            Physical AI Assistant
                        </h3>
                        <p style={{
                            fontSize: '0.75rem',
                            opacity: 0.9,
                            margin: '4px 0 0 0'
                        }}>
                            Powered by RAG + Google Gemini
                        </p>
                    </div>

                    {/* Messages */}
                    <div style={{
                        flex: 1,
                        overflowY: 'auto',
                        padding: '16px',
                        display: 'flex',
                        flexDirection: 'column',
                        gap: '12px'
                    }}>
                        {messages.map((msg, idx) => (
                            <div
                                key={idx}
                                style={{
                                    display: 'flex',
                                    justifyContent: msg.role === 'user' ? 'flex-end' : 'flex-start'
                                }}
                            >
                                <div
                                    style={{
                                        maxWidth: '80%',
                                        padding: '12px',
                                        borderRadius: '12px',
                                        backgroundColor: msg.role === 'user' ? '#2563eb' : '#f3f4f6',
                                        color: msg.role === 'user' ? 'white' : '#1f2937',
                                        borderBottomRightRadius: msg.role === 'user' ? '4px' : '12px',
                                        borderBottomLeftRadius: msg.role === 'user' ? '12px' : '4px'
                                    }}
                                >
                                    <p style={{
                                        fontSize: '0.875rem',
                                        margin: 0,
                                        whiteSpace: 'pre-wrap',
                                        wordBreak: 'break-word'
                                    }}>
                                        {msg.content}
                                    </p>
                                </div>
                            </div>
                        ))}
                        {loading && (
                            <div style={{ display: 'flex', justifyContent: 'flex-start' }}>
                                <div style={{
                                    backgroundColor: '#f3f4f6',
                                    padding: '12px',
                                    borderRadius: '12px'
                                }}>
                                    <div style={{ display: 'flex', gap: '8px' }}>
                                        {[0, 0.2, 0.4].map((delay, i) => (
                                            <div
                                                key={i}
                                                style={{
                                                    width: '8px',
                                                    height: '8px',
                                                    backgroundColor: '#9ca3af',
                                                    borderRadius: '50%',
                                                    animation: 'bounce 1s infinite',
                                                    animationDelay: `${delay}s`
                                                }}
                                            />
                                        ))}
                                    </div>
                                    <style>{`
                                        @keyframes bounce {
                                            0%, 100% { transform: translateY(0); }
                                            50% { transform: translateY(-10px); }
                                        }
                                    `}</style>
                                </div>
                            </div>
                        )}
                        <div ref={messagesEndRef} />
                    </div>

                    {/* Input */}
                    <div style={{
                        borderTop: '1px solid #e5e7eb',
                        padding: '16px'
                    }}>
                        <div style={{ display: 'flex', gap: '8px' }}>
                            <textarea
                                value={input}
                                onChange={(e) => setInput(e.target.value)}
                                onKeyPress={handleKeyPress}
                                placeholder="Ask about the textbook..."
                                disabled={loading}
                                style={{
                                    flex: 1,
                                    padding: '12px',
                                    border: '1px solid #d1d5db',
                                    borderRadius: '8px',
                                    resize: 'none',
                                    outline: 'none',
                                    fontSize: '0.875rem',
                                    fontFamily: 'inherit'
                                }}
                                rows={2}
                                onFocus={(e) => e.target.style.borderColor = '#2563eb'}
                                onBlur={(e) => e.target.style.borderColor = '#d1d5db'}
                            />
                            <button
                                onClick={sendMessage}
                                disabled={loading || (!input.trim() && !selectedText)}
                                style={{
                                    backgroundColor: (loading || (!input.trim() && !selectedText)) ? '#9ca3af' : '#2563eb',
                                    color: 'white',
                                    padding: '12px 16px',
                                    borderRadius: '8px',
                                    border: 'none',
                                    cursor: (loading || (!input.trim() && !selectedText)) ? 'not-allowed' : 'pointer',
                                    transition: 'background 0.3s',
                                    display: 'flex',
                                    alignItems: 'center',
                                    justifyContent: 'center'
                                }}
                                onMouseOver={(e) => {
                                    if (!loading && (input.trim() || selectedText)) {
                                        e.currentTarget.style.backgroundColor = '#1d4ed8';
                                    }
                                }}
                                onMouseOut={(e) => {
                                    if (!loading && (input.trim() || selectedText)) {
                                        e.currentTarget.style.backgroundColor = '#2563eb';
                                    }
                                }}
                            >
                                <Send size={20} />
                            </button>
                        </div>
                    </div>
                </div>
            )}
        </>
    );
}