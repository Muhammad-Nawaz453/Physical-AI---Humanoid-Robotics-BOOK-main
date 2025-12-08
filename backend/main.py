from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import os
from dotenv import load_dotenv
import google.generativeai as genai
from typing import Optional
import psycopg2
import hashlib
import secrets
from datetime import datetime, timedelta
from contextlib import asynccontextmanager

# Load environment variables FIRST
load_dotenv()

# Simple in-memory storage for textbook content
TEXTBOOK_CONTENT = {}


def load_textbook_content():
    """Load all markdown files into memory"""
    import glob

    # Try multiple possible paths
    possible_paths = [
        "./docs/**/*.md",
        "./docs/**/*.mdx",
        "../docs/**/*.md",
        "../docs/**/*.mdx",
        "D:/physical-ai-textbook/my-book/docs/**/*.md",
        "D:/physical-ai-textbook/my-book/docs/**/*.mdx"
    ]

    all_files = []
    for path in possible_paths:
        found = glob.glob(path, recursive=True)
        all_files.extend(found)
        if found:
            print(f"Found {len(found)} files in: {path}")

    # Remove duplicates
    all_files = list(set(all_files))

    if not all_files:
        print("WARNING: No markdown files found! Check your docs folder path.")
        print("Current directory:", os.getcwd())
        return

    print(f"\nTotal unique files found: {len(all_files)}")
    print("Loading files...")

    loaded_count = 0
    for file_path in all_files:
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
                if content.strip():  # Only add non-empty files
                    filename = os.path.basename(file_path)
                    TEXTBOOK_CONTENT[filename] = content
                    loaded_count += 1
                    print(f"  ✓ Loaded: {filename}")
        except Exception as e:
            print(f"  ✗ Error loading {file_path}: {e}")

    print(f"\n✅ Successfully loaded {loaded_count} documents into memory!\n")


def init_database():
    """Create sessions table if it doesn't exist"""
    try:
        conn = get_db()
        cursor = conn.cursor()

        # Create sessions table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS sessions (
                id SERIAL PRIMARY KEY,
                user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,
                session_token VARCHAR(255) UNIQUE NOT NULL,
                expires_at TIMESTAMP NOT NULL,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)

        conn.commit()
        cursor.close()
        conn.close()
        print("✅ Database tables initialized!")
    except Exception as e:
        print(f"⚠️  Database init error: {e}")


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    print("🚀 Starting Physical AI Textbook API...")
    load_textbook_content()
    init_database()
    print("✅ Server ready!")
    yield
    # Shutdown
    print("👋 Shutting down...")


app = FastAPI(lifespan=lifespan)

# CORS - Fixed to allow all origins during development
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://127.0.0.1:3000",
        "https://github.com/Muhammad-Nawaz453/",  # Add your GitHub Pages URL
        "https://*.vercel.app",  # Keep this for Vercel previews
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Configure Gemini (FREE!)
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
model = genai.GenerativeModel('gemini-2.5-flash-lite')

# Database connection


def get_db():
    return psycopg2.connect(os.getenv("DATABASE_URL"))

# Models


class ChatRequest(BaseModel):
    question: str
    selectedText: Optional[str] = None


class SignupRequest(BaseModel):
    email: str
    password: str
    name: str
    softwareBackground: str
    hardwareBackground: str
    experienceLevel: str


class LoginRequest(BaseModel):
    email: str
    password: str


class SessionValidateRequest(BaseModel):
    session_token: str


def search_content(query: str):
    """Simple keyword search in textbook content"""
    results = []
    query_lower = query.lower()

    for filename, content in TEXTBOOK_CONTENT.items():
        # Simple relevance: check if query words are in content
        if any(word in content.lower() for word in query_lower.split()):
            # Get a relevant snippet
            lines = content.split('\n')
            relevant_lines = [line for line in lines if any(
                word in line.lower() for word in query_lower.split())]
            snippet = '\n'.join(relevant_lines[:5])  # First 5 relevant lines
            results.append(snippet)

    return results[:3]  # Return top 3 results

# ============================================
# FIXED: Chatbot endpoint with proper POST
# ============================================


@app.post("/api/chatbot")
async def chatbot(request: ChatRequest):
    try:
        print(f"📨 Chatbot request received: {request.question[:50]}...")

        # Build context
        context_parts = []

        if request.selectedText:
            context_parts.append(
                f"User selected this text: {request.selectedText}")

        # Search textbook content
        if TEXTBOOK_CONTENT:
            search_results = search_content(request.question)
            if search_results:
                context_parts.append("Relevant textbook content:")
                context_parts.extend(search_results)
        else:
            print("⚠️  Warning: No textbook content loaded!")

        context = "\n\n".join(
            context_parts) if context_parts else "No specific context available."

        # Create prompt for Gemini
        prompt = f"""You are a helpful assistant for a Physical AI & Humanoid Robotics textbook.

Context from the textbook:
{context}

User question: {request.question}

Please provide a clear, helpful answer based on the context above. If the context doesn't contain the answer, provide general knowledge about robotics and AI."""

        # Get response from Gemini (FREE!)
        response = model.generate_content(prompt)

        print(f"✅ Response generated successfully!")
        return {"answer": response.text}

    except Exception as e:
        print(f"❌ Chatbot Error: {e}")
        return {"answer": f"Sorry, I encountered an error: {str(e)}. Please try again."}

# ============================================
# FIXED: Signup with better error handling
# ============================================


@app.post("/api/auth/signup")
async def signup(request: SignupRequest):
    try:
        print(f"📝 Signup request for: {request.email}")

        conn = get_db()
        cursor = conn.cursor()

        # Check if user already exists
        cursor.execute("SELECT id FROM users WHERE email = %s",
                       (request.email,))
        if cursor.fetchone():
            cursor.close()
            conn.close()
            raise HTTPException(
                status_code=400, detail="Email already registered")

        # Hash password
        password_hash = hashlib.sha256(request.password.encode()).hexdigest()

        cursor.execute("""
            INSERT INTO users (email, password_hash, name, software_background, hardware_background, experience_level)
            VALUES (%s, %s, %s, %s, %s, %s)
            RETURNING id
        """, (
            request.email,
            password_hash,
            request.name,
            request.softwareBackground,
            request.hardwareBackground,
            request.experienceLevel
        ))

        user_id = cursor.fetchone()[0]
        conn.commit()

        print(f"✅ User created with ID: {user_id}")

        cursor.close()
        conn.close()

        return {
            "message": "User created successfully",
            "user": {
                "id": user_id,
                "email": request.email,
                "name": request.name,
                "softwareBackground": request.softwareBackground,
                "hardwareBackground": request.hardwareBackground,
                "experienceLevel": request.experienceLevel
            }
        }

    except HTTPException:
        raise
    except Exception as e:
        print(f"❌ Signup error: {e}")
        raise HTTPException(status_code=400, detail=str(e))

# ============================================
# FIXED: Login with session token generation
# ============================================


@app.post("/api/auth/login")
async def login(request: LoginRequest):
    try:
        print(f"🔐 Login attempt for: {request.email}")

        conn = get_db()
        cursor = conn.cursor()

        # Hash password
        password_hash = hashlib.sha256(request.password.encode()).hexdigest()

        cursor.execute("""
            SELECT id, email, name, software_background, hardware_background, experience_level
            FROM users
            WHERE email = %s AND password_hash = %s
        """, (request.email, password_hash))

        user = cursor.fetchone()

        if not user:
            cursor.close()
            conn.close()
            print(f"❌ Invalid credentials for: {request.email}")
            raise HTTPException(status_code=401, detail="Invalid credentials")

        # Create session token
        session_token = secrets.token_urlsafe(32)
        expires_at = datetime.now() + timedelta(days=7)  # 7-day session

        cursor.execute("""
            INSERT INTO sessions (user_id, session_token, expires_at)
            VALUES (%s, %s, %s)
        """, (user[0], session_token, expires_at))

        conn.commit()
        cursor.close()
        conn.close()

        print(f"✅ Login successful for user ID: {user[0]}")

        return {
            "session_token": session_token,
            "user": {
                "id": user[0],
                "email": user[1],
                "name": user[2],
                "softwareBackground": user[3],
                "hardwareBackground": user[4],
                "experienceLevel": user[5]
            }
        }

    except HTTPException:
        raise
    except Exception as e:
        print(f"❌ Login error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

# ============================================
# NEW: Session validation endpoint
# ============================================


@app.get("/api/auth/validate")
async def validate_session(session_token: str):
    try:
        print(f"🔍 Validating session: {session_token[:10]}...")

        conn = get_db()
        cursor = conn.cursor()

        cursor.execute("""
            SELECT u.id, u.email, u.name, u.software_background, u.hardware_background, u.experience_level
            FROM users u
            JOIN sessions s ON u.id = s.user_id
            WHERE s.session_token = %s AND s.expires_at > NOW()
        """, (session_token,))

        user = cursor.fetchone()
        cursor.close()
        conn.close()

        if not user:
            print(f"❌ Invalid or expired session")
            raise HTTPException(
                status_code=401, detail="Invalid or expired session")

        print(f"✅ Session valid for user: {user[1]}")

        return {
            "id": user[0],
            "email": user[1],
            "name": user[2],
            "softwareBackground": user[3],
            "hardwareBackground": user[4],
            "experienceLevel": user[5]
        }

    except HTTPException:
        raise
    except Exception as e:
        print(f"❌ Session validation error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

# ============================================
# NEW: Logout endpoint
# ============================================


@app.post("/api/auth/logout")
async def logout(request: SessionValidateRequest):
    try:
        conn = get_db()
        cursor = conn.cursor()

        cursor.execute("""
            DELETE FROM sessions WHERE session_token = %s
        """, (request.session_token,))

        conn.commit()
        cursor.close()
        conn.close()

        return {"message": "Logged out successfully"}

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# ============================================
# Health check endpoints
# ============================================


@app.get("/")
async def root():
    return {
        "message": "Physical AI Textbook API - FREE Version with Session Management!",
        "documents_loaded": len(TEXTBOOK_CONTENT),
        "status": "running",
        "endpoints": {
            "chatbot": "POST /api/chatbot",
            "signup": "POST /api/auth/signup",
            "login": "POST /api/auth/login",
            "validate": "GET /api/auth/validate?session_token=xxx",
            "logout": "POST /api/auth/logout"
        }
    }


@app.get("/health")
async def health():
    return {
        "status": "healthy",
        "documents": len(TEXTBOOK_CONTENT),
        "gemini_configured": bool(os.getenv("GEMINI_API_KEY")),
        "database_configured": bool(os.getenv("DATABASE_URL"))
    }


if __name__ == "__main__":
    import uvicorn
    from dotenv import load_dotenv

    # Load .env file
    load_dotenv()

    print("="*60)
    print("🤖 Physical AI Textbook API Server")
    print("="*60)

    # Debug: Check if environment variables are loaded
    db_url = os.getenv("DATABASE_URL")
    gemini_key = os.getenv("GEMINI_API_KEY")

    print(f"📊 Environment Check:")
    print(f"   Gemini API Key: {'✅ Found' if gemini_key else '❌ Missing'}")
    print(f"   Database URL: {'✅ Found' if db_url else '❌ Missing'}")

    if db_url:
        # Print first 50 chars to verify it's Neon (not localhost)
        db_preview = db_url[:50] + "..." if len(db_url) > 50 else db_url
        print(f"   DB Preview: {db_preview}")

        if "localhost" in db_url or "127.0.0.1" in db_url:
            print("   ⚠️  WARNING: Database URL points to localhost!")
            print("   ⚠️  Make sure .env has your Neon connection string!")
    else:
        print("   ⚠️  DATABASE_URL not found in environment!")
        print("   ⚠️  Check if .env file exists in backend/ folder")

    print("="*60)

    uvicorn.run(app, host="0.0.0.0", port=8000)


app = app
