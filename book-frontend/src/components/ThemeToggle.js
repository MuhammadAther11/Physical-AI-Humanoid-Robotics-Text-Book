import React, {useState, useEffect} from 'react';
import clsx from 'clsx';
import styles from './ThemeToggle.module.css';

function ThemeToggle() {
  const [theme, setTheme] = useState('system'); // 'light', 'dark', or 'system'
  
  useEffect(() => {
    // Check user preference or system preference
    const savedTheme = localStorage.getItem('theme');
    const systemPrefersDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
    
    if (savedTheme) {
      setTheme(savedTheme);
      applyTheme(savedTheme);
    } else if (systemPrefersDark) {
      setTheme('dark');
      applyTheme('dark');
    } else {
      setTheme('light');
      applyTheme('light');
    }
  }, []);

  const applyTheme = (theme) => {
    if (theme === 'system') {
      const systemPrefersDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
      document.documentElement.setAttribute('data-theme', systemPrefersDark ? 'dark' : 'light');
    } else {
      document.documentElement.setAttribute('data-theme', theme);
    }
  };

  const toggleTheme = () => {
    const newTheme = theme === 'light' ? 'dark' : 
                     theme === 'dark' ? 'system' : 'light';
    
    setTheme(newTheme);
    localStorage.setItem('theme', newTheme);
    applyTheme(newTheme);
  };

  return (
    <div className={clsx('theme-toggle', styles.themeToggle)}>
      <button
        onClick={toggleTheme}
        className={clsx('theme-toggle__button', styles.themeToggleButton)}
        aria-label={`Switch to ${theme === 'light' ? 'dark' : theme === 'dark' ? 'system' : 'light'} theme`}
        title={`Current theme: ${theme}`}>
        {theme === 'light' ? (
          <span className={clsx('theme-toggle__icon', styles.themeToggleIcon)}>☀️ Light</span>
        ) : theme === 'dark' ? (
          <span className={clsx('theme-toggle__icon', styles.themeToggleIcon)}>🌙 Dark</span>
        ) : (
          <span className={clsx('theme-toggle__icon', styles.themeToggleIcon)}>💻 System</span>
        )}
      </button>
    </div>
  );
}

export default ThemeToggle;