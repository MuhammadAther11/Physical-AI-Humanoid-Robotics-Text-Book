import React from 'react';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import Chatbot from '@site/src/components/Chatbot/Chatbot';

export default function ChatbotPage(): JSX.Element {
  return (
    <Layout
      title="AI Assistant - Physical AI & Humanoid Robotics"
      description="RAG-powered AI Assistant for the Physical AI & Humanoid Robotics textbook">
      <div className="container" style={{ padding: '2rem 1rem', maxWidth: '1200px' }}>
        <Heading as="h1" style={{ textAlign: 'center', marginBottom: '1rem' }}>
          AI Assistant
        </Heading>
        <p style={{ textAlign: 'center', marginBottom: '2rem', color: 'var(--ifm-color-emphasis-600)' }}>
          Ask me anything about humanoid robotics, AI, or related topics from the textbook!
        </p>
        <Chatbot />
      </div>
    </Layout>
  );
}
