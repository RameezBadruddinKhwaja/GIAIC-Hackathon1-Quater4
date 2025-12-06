import React from 'react';
import OriginalContent from '@theme-original/DocItem/Content';
import PersonalizeButton from '@site/src/components/PersonalizeButton';
import TranslateButton from '@site/src/components/TranslateButton';

export default function Content(props): JSX.Element {
  return (
    <>
      <PersonalizeButton />
      <TranslateButton />
      <OriginalContent {...props} />
    </>
  );
}
