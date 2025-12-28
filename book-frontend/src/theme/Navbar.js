import React from 'react';
import OriginalNavbar from '@theme-original/Navbar';
import styles from './Navbar.module.css';

function Navbar(props) {
  return (
    <OriginalNavbar {...props} />
  );
}

export default Navbar;