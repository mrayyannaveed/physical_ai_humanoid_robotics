import React from 'react';
import ChatWidget from './ChatWidget';
import './App.css';

function App() {
  return (
    <div className="App">
      <header className="App-header">
        <h1>Physical AI & Humanoid Robotics Book</h1>
        <p>Welcome to the interactive book experience with AI assistance!</p>
      </header>

      <main>
        <section className="book-content">
          <h2>Chapter 1: Introduction to Physical AI</h2>
          <p>
            Physical AI represents the convergence of artificial intelligence and physical systems.
            It involves the development of intelligent agents that can interact with the physical world,
            perceive their environment, and make decisions based on sensory input. This field encompasses
            robotics, computer vision, machine learning, and control systems.
          </p>

          <h3>Key Concepts</h3>
          <p>
            The core concepts of Physical AI include perception, planning, control, and learning.
            Perception involves understanding the environment through sensors, while planning focuses
            on determining appropriate actions. Control ensures the execution of these actions,
            and learning allows systems to improve over time.
          </p>

          <h2>Chapter 2: Humanoid Robotics</h2>
          <p>
            Humanoid robots are designed to resemble and mimic human behavior and appearance.
            They represent a significant challenge in robotics, requiring advanced control systems,
            sophisticated actuators, and complex algorithms to achieve human-like movement and interaction.
          </p>

          <h3>Applications</h3>
          <p>
            Humanoid robots have potential applications in healthcare, education, entertainment,
            and assistance. They can serve as companions for the elderly, educational tools for children,
            or assistants in various service industries.
          </p>
        </section>
      </main>

      {/* The chat widget will be rendered here */}
      <ChatWidget />
    </div>
  );
}

export default App;