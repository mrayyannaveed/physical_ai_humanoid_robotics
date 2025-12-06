import React, { useState } from 'react';
import Layout from '@theme/Layout';

function AuthPage() {
  const [isLogin, setIsLogin] = useState(true);
  const [username, setUsername] = useState('');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [message, setMessage] = useState('');

  const handleSubmit = async (event) => {
    event.preventDefault();
    setMessage('');
    const endpoint = isLogin ? '/auth/login' : '/auth/register';
    const body = isLogin ? { username, password } : { username, email, password };

    try {
      const response = await fetch(`http://localhost:8000${endpoint}`, { // Assuming FastAPI runs on 8000
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(body),
      });

      const data = await response.json();

      if (response.ok) {
        setMessage(data.message || (isLogin ? 'Login successful!' : 'Registration successful!'));
        // TODO: Handle token storage on login (e.g., localStorage)
        if (isLogin && data.access_token) {
          console.log('Access Token:', data.access_token);
        }
      } else {
        setMessage(data.detail || 'An error occurred.');
      }
    } catch (error) {
      setMessage('Network error or server unavailable.');
      console.error('Auth API call failed:', error);
    }
  };

  return (
    <Layout title="Authentication" description="Login or Signup to the Humanoid Robot UI">
      <main className="container margin-vert--md">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1>{isLogin ? 'Login' : 'Signup'}</h1>
            <form onSubmit={handleSubmit}>
              <div className="margin-bottom--md">
                <input
                  type="text"
                  placeholder="Username"
                  className="input input--lg"
                  value={username}
                  onChange={(e) => setUsername(e.target.value)}
                  required
                />
              </div>
              {!isLogin && (
                <div className="margin-bottom--md">
                  <input
                    type="email"
                    placeholder="Email"
                    className="input input--lg"
                    value={email}
                    onChange={(e) => setEmail(e.target.value)}
                    required
                  />
                </div>
              )}
              <div className="margin-bottom--md">
                <input
                  type="password"
                  placeholder="Password"
                  className="input input--lg"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  required
                />
              </div>
              <button type="submit" className="button button--primary button--block button--lg">
                {isLogin ? 'Login' : 'Signup'}
              </button>
            </form>
            <div className="margin-top--md text--center">
              <button
                className="button button--link"
                onClick={() => setIsLogin(!isLogin)}
              >
                {isLogin ? 'Need an account? Signup' : 'Already have an account? Login'}
              </button>
            </div>
            {message && (
              <div className={`alert ${message.includes('successful') ? 'alert--success' : 'alert--danger'} margin-top--md`}>
                {message}
              </div>
            )}
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default AuthPage;
