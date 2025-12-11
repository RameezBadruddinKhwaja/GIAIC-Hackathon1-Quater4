import React, { useState, useRef } from 'react';
import Editor from '@monaco-editor/react';
import styles from './ROS2Playground.module.css';

interface ROS2PlaygroundProps {
  apiUrl?: string;
  initialLanguage?: 'python' | 'cpp';
}

export function ROS2Playground({
  apiUrl = 'https://giaic-hackathon1-quater4.vercel.app',
  initialLanguage = 'python',
}: ROS2PlaygroundProps) {
  const [description, setDescription] = useState('');
  const [language, setLanguage] = useState<'python' | 'cpp'>(initialLanguage);
  const [generatedCode, setGeneratedCode] = useState('# Enter a description and click Generate');
  const [explanation, setExplanation] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const editorRef = useRef<any>(null);

  const handleEditorMount = (editor: any) => {
    editorRef.current = editor;
  };

  const generateCode = async () => {
    if (!description.trim()) {
      setError('Please enter a description');
      return;
    }

    setLoading(true);
    setError(null);

    try {
      const response = await fetch(`${apiUrl}/api/codegen/`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          description: description.trim(),
          language,
        }),
      });

      if (response.ok) {
        const data = await response.json();
        setGeneratedCode(data.code);
        setExplanation(data.explanation);
      } else {
        const errorData = await response.json();
        setError(errorData.detail || 'Failed to generate code');
      }
    } catch (err) {
      setError('Network error. Please check your connection.');
      console.error('Error generating code:', err);
    } finally {
      setLoading(false);
    }
  };

  const copyCode = () => {
    navigator.clipboard.writeText(generatedCode);
    alert('Code copied to clipboard!');
  };

  const downloadCode = () => {
    const extension = language === 'python' ? 'py' : 'cpp';
    const blob = new Blob([generatedCode], { type: 'text/plain' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `ros2_node.${extension}`;
    a.click();
    URL.revokeObjectURL(url);
  };

  const examplePrompts = [
    'Create a publisher that publishes Twist messages to cmd_vel topic at 10Hz',
    'Create a subscriber that listens to /scan topic and prints laser ranges',
    'Create a service server that adds two integers',
    'Create an action server for a Fibonacci sequence',
  ];

  const fillExample = (example: string) => {
    setDescription(example);
  };

  return (
    <div className={styles.playground}>
      <div className={styles.header}>
        <h3>🤖 ROS 2 Code Generator</h3>
        <p>Generate ROS 2 nodes from natural language descriptions</p>
      </div>

      <div className={styles.controls}>
        <div className={styles.inputGroup}>
          <label htmlFor="description">Describe your ROS 2 node:</label>
          <textarea
            id="description"
            value={description}
            onChange={(e) => setDescription(e.target.value)}
            placeholder="Example: Create a publisher that sends velocity commands..."
            rows={3}
            className={styles.textarea}
          />
        </div>

        <div className={styles.examples}>
          <span className={styles.examplesLabel}>Examples:</span>
          {examplePrompts.map((example, idx) => (
            <button
              key={idx}
              onClick={() => fillExample(example)}
              className={styles.exampleButton}
            >
              {example.substring(0, 40)}...
            </button>
          ))}
        </div>

        <div className={styles.optionsRow}>
          <div className={styles.languageSelector}>
            <label>Language:</label>
            <select
              value={language}
              onChange={(e) => setLanguage(e.target.value as 'python' | 'cpp')}
              className={styles.select}
            >
              <option value="python">Python</option>
              <option value="cpp">C++</option>
            </select>
          </div>

          <button
            onClick={generateCode}
            disabled={loading || !description.trim()}
            className={styles.generateButton}
          >
            {loading ? '⏳ Generating...' : '✨ Generate Code'}
          </button>
        </div>

        {error && (
          <div className={styles.error}>
            ⚠️ {error}
          </div>
        )}
      </div>

      <div className={styles.editorContainer}>
        <div className={styles.editorHeader}>
          <span className={styles.editorTitle}>
            Generated Code ({language === 'python' ? 'Python' : 'C++'})
          </span>
          <div className={styles.editorActions}>
            <button onClick={copyCode} className={styles.actionButton} title="Copy code">
              📋 Copy
            </button>
            <button onClick={downloadCode} className={styles.actionButton} title="Download code">
              💾 Download
            </button>
          </div>
        </div>

        <Editor
          height="400px"
          language={language === 'python' ? 'python' : 'cpp'}
          value={generatedCode}
          onChange={(value) => setGeneratedCode(value || '')}
          onMount={handleEditorMount}
          theme="vs-dark"
          options={{
            minimap: { enabled: false },
            fontSize: 14,
            lineNumbers: 'on',
            scrollBeyondLastLine: false,
            automaticLayout: true,
            tabSize: language === 'python' ? 4 : 2,
          }}
        />
      </div>

      {explanation && (
        <div className={styles.explanation}>
          <h4>📝 Explanation:</h4>
          <p>{explanation}</p>
        </div>
      )}

      <div className={styles.tips}>
        <h4>💡 Tips:</h4>
        <ul>
          <li>Be specific about message types (e.g., Twist, LaserScan, Int32)</li>
          <li>Mention topic names for publishers/subscribers</li>
          <li>Specify frequencies for periodic operations</li>
          <li>Edit the generated code directly in the editor above</li>
        </ul>
      </div>
    </div>
  );
}
