import React from 'react';
import Layout from '@theme/Layout';
import styles from './chat.module.css';

export default function Chat(): JSX.Element {
  return (
    <Layout
      title="AI Chat"
      description="Chat with the Matrix AI Tutor about Physical AI and Robotics">
      <div className={styles.chatPage}>
        <div className={styles.container}>
          <h1>Matrix AI Tutor</h1>
          <p className={styles.description}>
            Click the chat button in the bottom-right corner to start chatting with the AI tutor.
          </p>
          <p className={styles.hint}>
            Ask questions about ROS 2, NVIDIA Isaac, Humanoid Robotics, SLAM, or any topic from the textbook.
          </p>
        </div>
      </div>
    </Layout>
  );
}
