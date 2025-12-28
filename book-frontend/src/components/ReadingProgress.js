import React, {useState, useEffect} from 'react';
import clsx from 'clsx';
import styles from './ReadingProgress.module.css';

function ReadingProgress() {
  const [progress, setProgress] = useState(0);

  useEffect(() => {
    const handleScroll = () => {
      const scrollTop = window.pageYOffset;
      const docHeight = document.documentElement.scrollHeight - window.innerHeight;
      const newProgress = (scrollTop / docHeight) * 100;
      setProgress(newProgress);
    };

    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  return (
    <div className={clsx('reading-progress', styles.readingProgress)}>
      <div 
        className={clsx('reading-progress__bar', styles.readingProgressBar)}
        style={{ width: `${progress}%` }}
      />
      <div className={clsx('reading-progress__text', styles.readingProgressText)}>
        {Math.round(progress)}% read
      </div>
    </div>
  );
}

export default ReadingProgress;