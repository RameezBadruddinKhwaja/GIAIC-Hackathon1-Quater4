import React from 'react';
import OriginalContent from '@theme-original/DocItem/Content';
import PersonalizeButton from '@site/src/components/PersonalizeButton';
import TranslateButton from '@site/src/components/TranslateButton';

export default function Content(props): JSX.Element {
  return (
    <>
      <OriginalContent {...props} />
      <div style={{ marginTop: '2rem', display: 'flex', gap: '1rem', flexWrap: 'wrap', justifyContent: 'flex-start' }}>
        <PersonalizeButton />
        <TranslateButton />
      </div>
    </>
  );
}
