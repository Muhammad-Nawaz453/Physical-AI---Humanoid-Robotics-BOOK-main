import React, { useState } from 'react';

export default function ChapterControls({ chapterContent, userBackground }) {
    const [content, setContent] = useState(chapterContent);
    const [isPersonalized, setIsPersonalized] = useState(false);
    const [isUrdu, setIsUrdu] = useState(false);
    const [loading, setLoading] = useState(false);

    const personalizeContent = () => {
        setLoading(true);

        setTimeout(() => {
            // Get user data
            const level = userBackground?.experienceLevel || 'beginner';
            const softwareSkills = userBackground?.softwareBackground || '';
            const hardwareSkills = userBackground?.hardwareBackground || '';

            let personalizedText = chapterContent;

            // Add personalized header based on experience level
            if (level === 'beginner') {
                personalizedText = `
## 🌱 Beginner-Friendly Version

*This content has been tailored for beginners. Take your time and practice each concept!*

---

${chapterContent}

---

### 💡 Tips for Beginners:
- Don't rush through the material
- Try coding examples yourself
- Use Google when stuck
- Join robotics communities for help
        `;
            } else if (level === 'intermediate') {
                personalizedText = `
## 📈 Intermediate Track

*You have some experience - this version includes additional challenges!*

---

${chapterContent}

---

### 🎯 Challenge Yourself:
- Implement optimizations
- Try different approaches
- Build your own variations
- Share your projects with others
        `;
            } else if (level === 'advanced') {
                personalizedText = `
## 🚀 Advanced Deep Dive

*Expert content with cutting-edge concepts and research directions.*

---

${chapterContent}

---

### ⚡ Advanced Topics:
- Latest research papers
- Performance optimization techniques
- Edge cases and error handling
- Contributing to open source projects
        `;
            }

            // Add relevant background tips
            if (softwareSkills.toLowerCase().includes('python')) {
                personalizedText += '\n\n### 🐍 Python Developer Tip:\nYou can leverage your Python skills here! Most robotics frameworks have excellent Python bindings.';
            }

            if (hardwareSkills.toLowerCase().includes('arduino') || hardwareSkills.toLowerCase().includes('raspberry')) {
                personalizedText += '\n\n### 🔧 Hardware Experience Bonus:\nYour hardware background gives you an advantage! You already understand embedded systems.';
            }

            setContent(personalizedText);
            setIsPersonalized(true);
            setLoading(false);
        }, 500);
    };

    const translateToUrdu = () => {
        setLoading(true);

        setTimeout(() => {
            // Common robotics terms with Urdu translations
            const translations = {
                'Robot': 'روبوٹ (Robot)',
                'robot': 'روبوٹ (robot)',
                'Sensor': 'سینسر (Sensor)',
                'sensor': 'سینسر (sensor)',
                'Control': 'کنٹرول (Control)',
                'control': 'کنٹرول (control)',
                'Simulation': 'نقالی (Simulation)',
                'simulation': 'نقالی (simulation)',
                'Artificial Intelligence': 'مصنوعی ذہانت (AI)',
                'artificial intelligence': 'مصنوعی ذہانت (AI)',
                'AI': 'اے آئی (AI)',
                'Physical AI': 'فزیکل اے آئی (Physical AI)',
                'Humanoid': 'انسان نما (Humanoid)',
                'humanoid': 'انسان نما (humanoid)',
                'Introduction': 'تعارف (Introduction)',
                'Module': 'ماڈیول (Module)',
                'module': 'ماڈیول (module)',
                'Chapter': 'باب (Chapter)',
                'chapter': 'باب (chapter)',
                'Learning': 'سیکھنا (Learning)',
                'learning': 'سیکھنا (learning)',
                'Programming': 'پروگرامنگ (Programming)',
                'programming': 'پروگرامنگ (programming)',
                'Code': 'کوڈ (Code)',
                'code': 'کوڈ (code)',
                'System': 'نظام (System)',
                'system': 'نظام (system)',
                'Data': 'ڈیٹا (Data)',
                'data': 'ڈیٹا (data)',
                'Algorithm': 'الگورتھم (Algorithm)',
                'algorithm': 'الگورتھم (algorithm)',
                'Model': 'ماڈل (Model)',
                'model': 'ماڈل (model)',
                'Training': 'تربیت (Training)',
                'training': 'تربیت (training)',
                'Neural Network': 'عصبی نیٹ ورک (Neural Network)',
                'Computer Vision': 'کمپیوٹر وژن (Computer Vision)',
                'Machine Learning': 'مشین لرننگ (Machine Learning)'
            };

            let urduContent = chapterContent;

            // Replace technical terms
            Object.entries(translations).forEach(([eng, urdu]) => {
                const regex = new RegExp(`\\b${eng}\\b`, 'g');
                urduContent = urduContent.replace(regex, urdu);
            });

            // Add Urdu header
            urduContent = `
# 🇵🇰 اردو ترجمہ / Urdu Translation

*نوٹ: یہ خودکار ترجمہ ہے۔ تکنیکی اصطلاحات انگریزی میں بھی شامل ہیں۔*

*Note: This is an automatic translation. Technical terms are included in English for clarity.*

---

${urduContent}

---

### اضافی معلومات / Additional Info:
- تکنیکی الفاظ دونوں زبانوں میں دیے گئے ہیں
- Technical terms are provided in both languages
- مزید سوالات کے لیے استاد سے رابطہ کریں
- Contact your instructor for more questions
      `;

            setContent(urduContent);
            setIsUrdu(true);
            setLoading(false);
        }, 500);
    };

    const resetContent = () => {
        setContent(chapterContent);
        setIsPersonalized(false);
        setIsUrdu(false);
    };

    return (
        <div style={{
            backgroundColor: '#f8f9fa',
            borderRadius: '12px',
            padding: '1.5rem',
            marginBottom: '2rem',
            border: '2px solid #e0e0e0'
        }}>
            {/* Control Buttons */}
            <div style={{
                display: 'flex',
                flexWrap: 'wrap',
                gap: '0.75rem',
                marginBottom: '1.5rem'
            }}>
                <button
                    onClick={personalizeContent}
                    disabled={loading || isPersonalized}
                    style={{
                        display: 'flex',
                        alignItems: 'center',
                        gap: '0.5rem',
                        backgroundColor: isPersonalized ? '#28a745' : '#9b59b6',
                        color: 'white',
                        padding: '0.75rem 1.25rem',
                        border: 'none',
                        borderRadius: '8px',
                        fontSize: '0.95rem',
                        fontWeight: '600',
                        cursor: loading || isPersonalized ? 'not-allowed' : 'pointer',
                        opacity: loading || isPersonalized ? 0.6 : 1,
                        transition: 'all 0.3s'
                    }}
                    onMouseOver={(e) => {
                        if (!loading && !isPersonalized) {
                            e.target.style.transform = 'scale(1.05)';
                        }
                    }}
                    onMouseOut={(e) => {
                        e.target.style.transform = 'scale(1)';
                    }}
                >
                    <span>✨</span>
                    {isPersonalized ? 'Personalized ✓' : 'Personalize Content'}
                </button>

                <button
                    onClick={translateToUrdu}
                    disabled={loading || isUrdu}
                    style={{
                        display: 'flex',
                        alignItems: 'center',
                        gap: '0.5rem',
                        backgroundColor: isUrdu ? '#28a745' : '#27ae60',
                        color: 'white',
                        padding: '0.75rem 1.25rem',
                        border: 'none',
                        borderRadius: '8px',
                        fontSize: '0.95rem',
                        fontWeight: '600',
                        cursor: loading || isUrdu ? 'not-allowed' : 'pointer',
                        opacity: loading || isUrdu ? 0.6 : 1,
                        transition: 'all 0.3s'
                    }}
                    onMouseOver={(e) => {
                        if (!loading && !isUrdu) {
                            e.target.style.transform = 'scale(1.05)';
                        }
                    }}
                    onMouseOut={(e) => {
                        e.target.style.transform = 'scale(1)';
                    }}
                >
                    <span>🇵🇰</span>
                    {isUrdu ? 'اردو میں ✓' : 'Translate to Urdu'}
                </button>

                {(isPersonalized || isUrdu) && (
                    <button
                        onClick={resetContent}
                        style={{
                            display: 'flex',
                            alignItems: 'center',
                            gap: '0.5rem',
                            backgroundColor: '#6c757d',
                            color: 'white',
                            padding: '0.75rem 1.25rem',
                            border: 'none',
                            borderRadius: '8px',
                            fontSize: '0.95rem',
                            fontWeight: '600',
                            cursor: 'pointer',
                            transition: 'all 0.3s'
                        }}
                        onMouseOver={(e) => {
                            e.target.style.transform = 'scale(1.05)';
                        }}
                        onMouseOut={(e) => {
                            e.target.style.transform = 'scale(1)';
                        }}
                    >
                        <span>↺</span>
                        Reset to Original
                    </button>
                )}
            </div>

            {/* Content Display */}
            <div style={{
                backgroundColor: 'white',
                borderRadius: '8px',
                padding: '1.5rem',
                border: '1px solid #dee2e6',
                minHeight: '200px'
            }}>
                {loading ? (
                    <div style={{
                        display: 'flex',
                        alignItems: 'center',
                        justifyContent: 'center',
                        padding: '3rem'
                    }}>
                        <div style={{
                            width: '40px',
                            height: '40px',
                            border: '4px solid #f3f3f3',
                            borderTop: '4px solid #667eea',
                            borderRadius: '50%',
                            animation: 'spin 1s linear infinite'
                        }} />
                    </div>
                ) : (
                    <div style={{
                        whiteSpace: 'pre-wrap',
                        lineHeight: '1.6',
                        color: '#333'
                    }}>
                        {content}
                    </div>
                )}
            </div>

            {/* Status Badges */}
            {!loading && (
                <div style={{
                    marginTop: '1rem',
                    display: 'flex',
                    flexWrap: 'wrap',
                    gap: '0.5rem'
                }}>
                    {isPersonalized && (
                        <span style={{
                            backgroundColor: '#e7d4f7',
                            color: '#6f42c1',
                            padding: '0.4rem 0.8rem',
                            borderRadius: '20px',
                            fontSize: '0.85rem',
                            fontWeight: '600'
                        }}>
                            🎯 Tailored to {userBackground?.experienceLevel || 'your'} level
                        </span>
                    )}
                    {isUrdu && (
                        <span style={{
                            backgroundColor: '#d4edda',
                            color: '#155724',
                            padding: '0.4rem 0.8rem',
                            borderRadius: '20px',
                            fontSize: '0.85rem',
                            fontWeight: '600'
                        }}>
                            🇵🇰 اردو Translation Active
                        </span>
                    )}
                </div>
            )}

            <style>{`
        @keyframes spin {
          0% { transform: rotate(0deg); }
          100% { transform: rotate(360deg); }
        }
      `}</style>
        </div>
    );
}