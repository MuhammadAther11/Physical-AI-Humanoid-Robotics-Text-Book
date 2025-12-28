import React from 'react';
import OriginalLayout from '@theme-original/Layout';

function Layout(props) {
  return (
    <OriginalLayout {...props}>
      {props.children}
    </OriginalLayout>
  );
}

export default Layout;