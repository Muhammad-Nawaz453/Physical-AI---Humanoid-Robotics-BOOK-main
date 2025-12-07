import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import useBaseUrl from '@docusaurus/useBaseUrl';
import API_URL from '../config';
export default function Login() {
    const [email, setEmail] = useState('');
    const [password, setPassword] = useState('');
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState('');
    const [debugInfo, setDebugInfo] = useState('');

    const signupUrl = useBaseUrl('/signup');
    const homeUrl = useBaseUrl('/');

    // Redirect if already logged in
    useEffect(() => {
        const sessionToken = localStorage.getItem('session_token');
        if (sessionToken) {
            console.log('Already logged in, redirecting...');
            window.location.href = homeUrl;
        }
    }, [homeUrl]);

    const handleLogin = async () => {
        setError('');
        setDebugInfo('');

        if (!email || !password) {
            setError('Please fill in all fields');
            return;
        }

        setLoading(true);
        setDebugInfo('Sending login request...');
        console.log('Login attempt:', { email });

        try {
            const response = await fetch(`${API_URL}/api/auth/login`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ email, password })
            });

            console.log('Response status:', response.status);
            setDebugInfo(`Response: ${response.status}`);

            const data = await response.json();
            console.log('Response data:', data);

            if (response.ok) {
                if (!data.session_token) {
                    setError('Login succeeded but no session token received!');
                    console.error('Missing session_token in response:', data);
                    return;
                }

                // Store session token and user data
                localStorage.setItem('session_token', data.session_token);
                localStorage.setItem('user', JSON.stringify(data.user));

                console.log('Login successful! Session stored.');
                console.log('Session token:', data.session_token.substring(0, 10) + '...');

                setDebugInfo('Login successful! Redirecting...');

                // Small delay to show success message
                setTimeout(() => {
                    window.location.href = homeUrl;
                }, 500);
            } else {
                const errorMessage = data.detail || 'Invalid email or password';
                setError(errorMessage);
                console.error('Login failed:', errorMessage);

                if (errorMessage.includes('Invalid credentials')) {
                    setError('Invalid email or password. Please check and try again.');
                }
            }
        } catch (error) {
            console.error('Login error:', error);
            setError(`Connection error: ${error.message}. Make sure the backend is running on port 8000!`);
            setDebugInfo(`Error: ${error.message}`);
        } finally {
            setLoading(false);
        }
    };

    const handleKeyPress = (e) => {
        if (e.key === 'Enter' && !loading) {
            handleLogin();
        }
    };

    return (
        <Layout title="Login">
            <div style={{
                minHeight: '80vh',
                background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                padding: '2rem'
            }}>
                <div style={{
                    backgroundColor: 'white',
                    borderRadius: '16px',
                    boxShadow: '0 20px 60px rgba(0,0,0,0.3)',
                    padding: '2.5rem',
                    width: '100%',
                    maxWidth: '420px'
                }}>
                    <h2 style={{
                        fontSize: '2rem',
                        fontWeight: 'bold',
                        color: '#333',
                        marginBottom: '0.5rem',
                        textAlign: 'center'
                    }}>
                        🤖 Welcome Back
                    </h2>
                    <p style={{
                        textAlign: 'center',
                        color: '#666',
                        marginBottom: '2rem'
                    }}>
                        Login to continue your Physical AI journey
                    </p>

                    {/* Error Message */}
                    {error && (
                        <div style={{
                            backgroundColor: '#fee',
                            border: '1px solid #fcc',
                            color: '#c33',
                            padding: '0.75rem',
                            borderRadius: '8px',
                            marginBottom: '1rem',
                            fontSize: '0.9rem'
                        }}>
                            ⚠️ {error}
                        </div>
                    )}

                    {/* Debug Info */}
                    {debugInfo && (
                        <div style={{
                            backgroundColor: '#e3f2fd',
                            border: '1px solid #90caf9',
                            color: '#1565c0',
                            padding: '0.5rem',
                            borderRadius: '4px',
                            marginBottom: '1rem',
                            fontSize: '0.8rem'
                        }}>
                            🔍 {debugInfo}
                        </div>
                    )}

                    <div style={{ marginBottom: '1rem' }}>
                        <label style={{
                            display: 'block',
                            marginBottom: '0.5rem',
                            fontWeight: '500',
                            color: '#333'
                        }}>
                            Email
                        </label>
                        <input
                            type="email"
                            placeholder="john@example.com"
                            value={email}
                            onChange={(e) => setEmail(e.target.value)}
                            onKeyPress={handleKeyPress}
                            disabled={loading}
                            autoComplete="email"
                            style={{
                                width: '100%',
                                padding: '0.75rem',
                                border: '2px solid #e0e0e0',
                                borderRadius: '8px',
                                fontSize: '1rem',
                                outline: 'none',
                                transition: 'border 0.3s'
                            }}
                            onFocus={(e) => e.target.style.borderColor = '#667eea'}
                            onBlur={(e) => e.target.style.borderColor = '#e0e0e0'}
                        />
                    </div>

                    <div style={{ marginBottom: '1.5rem' }}>
                        <label style={{
                            display: 'block',
                            marginBottom: '0.5rem',
                            fontWeight: '500',
                            color: '#333'
                        }}>
                            Password
                        </label>
                        <input
                            type="password"
                            placeholder="Enter your password"
                            value={password}
                            onChange={(e) => setPassword(e.target.value)}
                            onKeyPress={handleKeyPress}
                            disabled={loading}
                            autoComplete="current-password"
                            style={{
                                width: '100%',
                                padding: '0.75rem',
                                border: '2px solid #e0e0e0',
                                borderRadius: '8px',
                                fontSize: '1rem',
                                outline: 'none',
                                transition: 'border 0.3s'
                            }}
                            onFocus={(e) => e.target.style.borderColor = '#667eea'}
                            onBlur={(e) => e.target.style.borderColor = '#e0e0e0'}
                        />
                    </div>

                    <button
                        onClick={handleLogin}
                        disabled={loading}
                        style={{
                            width: '100%',
                            padding: '0.75rem',
                            backgroundColor: loading ? '#ccc' : '#667eea',
                            color: 'white',
                            border: 'none',
                            borderRadius: '8px',
                            fontSize: '1rem',
                            fontWeight: '600',
                            cursor: loading ? 'not-allowed' : 'pointer',
                            transition: 'background 0.3s'
                        }}
                        onMouseOver={(e) => !loading && (e.target.style.backgroundColor = '#5568d3')}
                        onMouseOut={(e) => !loading && (e.target.style.backgroundColor = '#667eea')}
                    >
                        {loading ? '⏳ Logging in...' : 'Login →'}
                    </button>

                    <p style={{
                        marginTop: '1.5rem',
                        textAlign: 'center',
                        color: '#666'
                    }}>
                        Don't have an account?{' '}
                        <a
                            href={signupUrl}
                            style={{
                                color: '#667eea',
                                fontWeight: '600',
                                textDecoration: 'none'
                            }}
                        >
                            Sign up
                        </a>
                    </p>

                    {/* Test Backend Connection */}
                    <button
                        onClick={async () => {
                            try {
                                const response = await fetch(`${API_URL}/health`);
                                const data = await response.json();
                                alert(`Backend Status: ${data.status}\nDocuments: ${data.documents}\nDatabase: ${data.database_configured ? '✅' : '❌'}`);
                            } catch (error) {
                                alert(`Cannot reach backend! Error: ${error.message}`);
                            }
                        }}
                        style={{
                            marginTop: '1rem',
                            width: '100%',
                            padding: '0.5rem',
                            backgroundColor: '#f0f0f0',
                            color: '#666',
                            border: '1px solid #ddd',
                            borderRadius: '4px',
                            fontSize: '0.85rem',
                            cursor: 'pointer'
                        }}
                    >
                        🔍 Test Backend Connection
                    </button>
                </div>
            </div>
        </Layout>
    );
}