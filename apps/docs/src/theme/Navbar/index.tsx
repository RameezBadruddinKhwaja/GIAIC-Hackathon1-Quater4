import React from 'react';
import OriginalNavbar from '@theme-original/Navbar';
import AuthButton from '@site/src/components/Auth/AuthButton';

export default function Navbar(props): JSX.Element {
  return (
    <div style={{ position: 'relative' }}>
      <OriginalNavbar {...props} />
      <div
        style={{
          position: 'absolute',
          top: '0',
          right: '16px',
          height: '100%',
          display: 'flex',
          alignItems: 'center',
          pointerEvents: 'none',
        }}
      >
        <div style={{ pointerEvents: 'auto' }}>
          <AuthButton />
        </div>
      </div>
    </div>
  );
}
