import React from 'react';
import OriginalContent from '@theme-original/DocItem/Content';
import TranslateButton from '@site/src/components/TranslateButton';
import PersonalizeButton from '@site/src/components/PersonalizeButton';

export default function Content(props): JSX.Element {
  return (
    <>
      <div style={{
        display: 'flex',
        gap: '12px',
        marginBottom: '24px',
        flexWrap: 'wrap',
        alignItems: 'center'
      }}>
        <TranslateButton />
        <PersonalizeButton />
      </div>
      <OriginalContent {...props} />
    </>
  );
}
