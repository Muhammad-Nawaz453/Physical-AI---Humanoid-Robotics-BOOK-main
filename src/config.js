/**
 * API Configuration for Production
 */

// Your Vercel backend URL
const BACKEND_URL = 'https://physical-ai-humanoid-robotics-nmkiaqiyx-muhammadnawazs-projects.vercel.app/';  // UPDATE THIS!

// Determine environment
const isProduction = typeof window !== 'undefined' 
  ? window.location.hostname !== 'localhost'
  : false;

// Export API URL
export const API_URL = isProduction 
  ? BACKEND_URL 
  : 'http://localhost:8000';

// API Endpoints
export const API_ENDPOINTS = {
  CHATBOT: `${API_URL}/api/chatbot`,
  SIGNUP: `${API_URL}/api/auth/signup`,
  LOGIN: `${API_URL}/api/auth/login`,
  VALIDATE: `${API_URL}/api/auth/validate`,
  HEALTH: `${API_URL}/health`,
};

export default API_URL;