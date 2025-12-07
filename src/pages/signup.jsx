import React, { useState } from 'react';
import Layout from '@theme/Layout';
import useBaseUrl from '@docusaurus/useBaseUrl';
import API_URL from '../config';

export default function Signup() {
    const [step, setStep] = useState(1);
    const [formData, setFormData] = useState({
        email: '',
        password: '',
        name: '',
        softwareBackground: '',
        hardwareBackground: '',
        experienceLevel: 'beginner'
    });
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState('');
    const [debugInfo, setDebugInfo] = useState('');

    const loginUrl = useBaseUrl('/login');
    const homeUrl = useBaseUrl('/');

    const handleSubmit = async () => {
        setLoading(true);
        setError('');
        setDebugInfo('Sending request to backend...');

        try {
            console.log('Sending signup request:', {
                email: formData.email,
                name: formData.name,
                // Don't log password
            });

            const response = await fetch(`${API_URL}/api/auth/signup`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(formData)
            });

            console.log('Response status:', response.status);
            setDebugInfo(`Response received: ${response.status}`);

            const data = await response.json();
            console.log('Response data:', data);

            if (response.ok) {
                // IMPORTANT: Login endpoint returns session_token, signup doesn't
                // So we need to login after signup
                setDebugInfo('Signup successful! Now logging in...');

                // Auto-login after successful signup
                const loginResponse = await fetch(`${API_URL}/api/auth/login`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        email: formData.email,
                        password: formData.password
                    })
                });

                if (loginResponse.ok) {
                    const loginData = await loginResponse.json();
                    localStorage.setItem('session_token', loginData.session_token);
                    localStorage.setItem('user', JSON.stringify(loginData.user));

                    console.log('Auto-login successful!');
                    alert('Signup successful! Redirecting...');
                    window.location.href = homeUrl;
                } else {
                    // Signup worked but auto-login failed - just redirect to login
                    alert('Signup successful! Please login.');
                    window.location.href = loginUrl;
                }
            } else {
                // Signup failed
                const errorMessage = data.detail || 'Signup failed!';
                setError(errorMessage);
                console.error('Signup error:', errorMessage);

                if (errorMessage.includes('already registered') || errorMessage.includes('already exist')) {
                    setError('This email is already registered. Try logging in or use a different email.');
                }
            }
        } catch (error) {
            console.error('Network error:', error);
            setError(`Connection error: ${error.message}. Make sure backend is running on port 8000!`);
            setDebugInfo(`Error: ${error.message}`);
        } finally {
            setLoading(false);
        }
    };

    return (
        <Layout title="Sign Up">
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
                    padding: '2rem',
                    width: '100%',
                    maxWidth: '500px'
                }}>
                    <h2 style={{
                        fontSize: '2rem',
                        fontWeight: 'bold',
                        color: '#333',
                        marginBottom: '1.5rem',
                        textAlign: 'center'
                    }}>
                        {step === 1 ? '🤖 Create Account' : '📚 Tell Us About Yourself'}
                    </h2>

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

                    {/* Debug Info (only during development) */}
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

                    {/* Progress Bar */}
                    <div style={{ display: 'flex', marginBottom: '2rem', gap: '0.5rem' }}>
                        <div style={{
                            flex: 1,
                            height: '8px',
                            borderRadius: '4px',
                            backgroundColor: step >= 1 ? '#667eea' : '#e0e0e0'
                        }} />
                        <div style={{
                            flex: 1,
                            height: '8px',
                            borderRadius: '4px',
                            backgroundColor: step >= 2 ? '#667eea' : '#e0e0e0'
                        }} />
                    </div>

                    {step === 1 ? (
                        <>
                            <div style={{ marginBottom: '1rem' }}>
                                <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
                                    Full Name
                                </label>
                                <input
                                    type="text"
                                    placeholder="John Doe"
                                    value={formData.name}
                                    onChange={(e) => setFormData({ ...formData, name: e.target.value })}
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

                            <div style={{ marginBottom: '1rem' }}>
                                <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
                                    Email
                                </label>
                                <input
                                    type="email"
                                    placeholder="john@example.com"
                                    value={formData.email}
                                    onChange={(e) => setFormData({ ...formData, email: e.target.value })}
                                    style={{
                                        width: '100%',
                                        padding: '0.75rem',
                                        border: '2px solid #e0e0e0',
                                        borderRadius: '8px',
                                        fontSize: '1rem',
                                        outline: 'none'
                                    }}
                                    onFocus={(e) => e.target.style.borderColor = '#667eea'}
                                    onBlur={(e) => e.target.style.borderColor = '#e0e0e0'}
                                />
                            </div>

                            <div style={{ marginBottom: '1.5rem' }}>
                                <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
                                    Password
                                </label>
                                <input
                                    type="password"
                                    placeholder="Min 6 characters"
                                    value={formData.password}
                                    onChange={(e) => setFormData({ ...formData, password: e.target.value })}
                                    style={{
                                        width: '100%',
                                        padding: '0.75rem',
                                        border: '2px solid #e0e0e0',
                                        borderRadius: '8px',
                                        fontSize: '1rem',
                                        outline: 'none'
                                    }}
                                    onFocus={(e) => e.target.style.borderColor = '#667eea'}
                                    onBlur={(e) => e.target.style.borderColor = '#e0e0e0'}
                                />
                            </div>

                            <button
                                onClick={() => {
                                    if (!formData.name || !formData.email || !formData.password) {
                                        setError('Please fill all fields!');
                                        return;
                                    }
                                    if (formData.password.length < 6) {
                                        setError('Password must be at least 6 characters!');
                                        return;
                                    }
                                    setError('');
                                    setStep(2);
                                }}
                                style={{
                                    width: '100%',
                                    padding: '0.75rem',
                                    backgroundColor: '#667eea',
                                    color: 'white',
                                    border: 'none',
                                    borderRadius: '8px',
                                    fontSize: '1rem',
                                    fontWeight: '600',
                                    cursor: 'pointer',
                                    transition: 'background 0.3s'
                                }}
                                onMouseOver={(e) => e.target.style.backgroundColor = '#5568d3'}
                                onMouseOut={(e) => e.target.style.backgroundColor = '#667eea'}
                            >
                                Next Step →
                            </button>
                        </>
                    ) : (
                        <>
                            <div style={{ marginBottom: '1rem' }}>
                                <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
                                    Experience Level
                                </label>
                                <select
                                    value={formData.experienceLevel}
                                    onChange={(e) => setFormData({ ...formData, experienceLevel: e.target.value })}
                                    style={{
                                        width: '100%',
                                        padding: '0.75rem',
                                        border: '2px solid #e0e0e0',
                                        borderRadius: '8px',
                                        fontSize: '1rem',
                                        outline: 'none'
                                    }}
                                >
                                    <option value="beginner">🌱 Beginner - New to robotics</option>
                                    <option value="intermediate">📈 Intermediate - Some experience</option>
                                    <option value="advanced">🚀 Advanced - Expert level</option>
                                </select>
                            </div>

                            <div style={{ marginBottom: '1rem' }}>
                                <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
                                    Software Background
                                </label>
                                <textarea
                                    placeholder="e.g., Python, ROS, Machine Learning, TensorFlow..."
                                    value={formData.softwareBackground}
                                    onChange={(e) => setFormData({ ...formData, softwareBackground: e.target.value })}
                                    style={{
                                        width: '100%',
                                        padding: '0.75rem',
                                        border: '2px solid #e0e0e0',
                                        borderRadius: '8px',
                                        fontSize: '1rem',
                                        height: '100px',
                                        resize: 'vertical',
                                        outline: 'none',
                                        fontFamily: 'inherit'
                                    }}
                                    onFocus={(e) => e.target.style.borderColor = '#667eea'}
                                    onBlur={(e) => e.target.style.borderColor = '#e0e0e0'}
                                />
                            </div>

                            <div style={{ marginBottom: '1.5rem' }}>
                                <label style={{ display: 'block', marginBottom: '0.5rem', fontWeight: '500' }}>
                                    Hardware Background
                                </label>
                                <textarea
                                    placeholder="e.g., Arduino, Raspberry Pi, Robotics kits, 3D printing..."
                                    value={formData.hardwareBackground}
                                    onChange={(e) => setFormData({ ...formData, hardwareBackground: e.target.value })}
                                    style={{
                                        width: '100%',
                                        padding: '0.75rem',
                                        border: '2px solid #e0e0e0',
                                        borderRadius: '8px',
                                        fontSize: '1rem',
                                        height: '100px',
                                        resize: 'vertical',
                                        outline: 'none',
                                        fontFamily: 'inherit'
                                    }}
                                    onFocus={(e) => e.target.style.borderColor = '#667eea'}
                                    onBlur={(e) => e.target.style.borderColor = '#e0e0e0'}
                                />
                            </div>

                            <div style={{ display: 'flex', gap: '1rem' }}>
                                <button
                                    onClick={() => setStep(1)}
                                    disabled={loading}
                                    style={{
                                        flex: 1,
                                        padding: '0.75rem',
                                        backgroundColor: loading ? '#f0f0f0' : '#e0e0e0',
                                        color: '#333',
                                        border: 'none',
                                        borderRadius: '8px',
                                        fontSize: '1rem',
                                        fontWeight: '600',
                                        cursor: loading ? 'not-allowed' : 'pointer'
                                    }}
                                >
                                    ← Back
                                </button>
                                <button
                                    onClick={handleSubmit}
                                    disabled={loading}
                                    style={{
                                        flex: 2,
                                        padding: '0.75rem',
                                        backgroundColor: loading ? '#ccc' : '#667eea',
                                        color: 'white',
                                        border: 'none',
                                        borderRadius: '8px',
                                        fontSize: '1rem',
                                        fontWeight: '600',
                                        cursor: loading ? 'not-allowed' : 'pointer'
                                    }}
                                >
                                    {loading ? '⏳ Creating Account...' : 'Complete Signup ✓'}
                                </button>
                            </div>
                        </>
                    )}

                    <p style={{
                        marginTop: '1.5rem',
                        textAlign: 'center',
                        color: '#666'
                    }}>
                        Already have an account? <a href={loginUrl} style={{ color: '#667eea', fontWeight: '600' }}>Login</a>
                    </p>
                </div>
            </div>
        </Layout>
    );
}