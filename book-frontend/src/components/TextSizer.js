import React, {useState, useEffect} from 'react';
import clsx from 'clsx';
import styles from './TextSizer.module.css';

function TextSizer() {
  const [textSize, setTextSize] = useState('normal'); // 'small', 'normal', 'large', 'larger'
  
  useEffect(() => {
    // Check for saved text size preference
    const savedTextSize = localStorage.getItem('textSize');
    if (savedTextSize) {
      setTextSize(savedTextSize);
      applyTextSize(savedTextSize);
    }
  }, []);

  const applyTextSize = (size) => {
    document.documentElement.style.setProperty('--ifm-font-size-base', 
      size === 'small' ? '0.875rem' : 
      size === 'large' ? '1.125rem' : 
      size === 'larger' ? '1.25rem' : '1rem');
      
    document.documentElement.style.setProperty('--ifm-h1-font-size', 
      size === 'small' ? '2rem' : 
      size === 'large' ? '2.5rem' : 
      size === 'larger' ? '3rem' : '2.25rem');
      
    document.documentElement.style.setProperty('--ifm-h2-font-size', 
      size === 'small' ? '1.5rem' : 
      size === 'large' ? '2rem' : 
      size === 'larger' ? '2.25rem' : '1.75rem');
  };

  const changeTextSize = (newSize) => {
    setTextSize(newSize);
    localStorage.setItem('textSize', newSize);
    applyTextSize(newSize);
  };

  return (
    <div className={clsx('text-sizer', styles.textSizer)}>
      <div className={clsx('text-sizer__controls', styles.textSizerControls)}>
        <button
          onClick={() => changeTextSize('small')}
          className={clsx('text-sizer__button', styles.textSizerButton, {
            [styles.textSizerButtonActive]: textSize === 'small'
          })}
          aria-label="Small text size"
          title="Small text">
          A
        </button>
        <button
          onClick={() => changeTextSize('normal')}
          className={clsx('text-sizer__button', styles.textSizerButton, {
            [styles.textSizerButtonActive]: textSize === 'normal'
          })}
          aria-label="Normal text size"
          title="Normal text">
          A
        </button>
        <button
          onClick={() => changeTextSize('large')}
          className={clsx('text-sizer__button', styles.textSizerButton, {
            [styles.textSizerButtonActive]: textSize === 'large'
          })}
          aria-label="Large text size"
          title="Large text">
          A
        </button>
        <button
          onClick={() => changeTextSize('larger')}
          className={clsx('text-sizer__button', styles.textSizerButton, {
            [styles.textSizerButtonActive]: textSize === 'larger'
          })}
          aria-label="Larger text size"
          title="Larger text">
          A
        </button>
      </div>
      <div className={clsx('text-sizer__label', styles.textSizerLabel)}>
        Text Size
      </div>
    </div>
  );
}

export default TextSizer;