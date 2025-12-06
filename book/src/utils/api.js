// book/src/utils/api.js

const API_BASE_URL = 'http://localhost:8000'; // Assuming FastAPI backend runs on port 8000

// Helper function to make authenticated API requests
async function makeAuthenticatedRequest(endpoint, method = 'GET', body = null) {
  const token = localStorage.getItem('access_token'); // Assuming token is stored in localStorage
  const headers = {
    'Content-Type': 'application/json',
    ...(token && { 'Authorization': `Bearer ${token}` }),
  };

  const config = {
    method,
    headers,
    ...(body && { body: JSON.stringify(body) }),
  };

  try {
    const response = await fetch(`${API_BASE_URL}${endpoint}`, config);
    const data = await response.json();
    if (!response.ok) {
      throw new Error(data.detail || 'API request failed');
    }
    return data;
  } catch (error) {
    console.error('API Error:', error);
    throw error;
  }
}

// --- Authentication API Calls (from auth.md) ---

export async function loginUser(username, password) {
  return makeAuthenticatedRequest('/auth/login', 'POST', { username, password });
}

export async function registerUser(username, email, password) {
  return makeAuthenticatedRequest('/auth/register', 'POST', { username, email, password });
}

export async function fetchUserProfile() {
  return makeAuthenticatedRequest('/users/me');
}

export async function updateUserPreferences(language_preference, theme_preference) {
  return makeAuthenticatedRequest('/users/me/preferences', 'PUT', { language_preference, theme_preference });
}

// --- Other potential API calls ---
// export async function fetchRobotStatus() {
//   return makeAuthenticatedRequest('/robot/status');
// }

// export async function sendRobotCommand(command_data) {
//   return makeAuthenticatedRequest('/robot/command', 'POST', command_data);
// }
