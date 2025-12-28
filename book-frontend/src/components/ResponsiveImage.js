import React from 'react';
import clsx from 'clsx';
import styles from './ResponsiveImage.module.css';

function ResponsiveImage({src, alt, caption, className, ...props}) {
  return (
    <figure className={clsx('responsive-image', styles.responsiveImage, className)}>
      <img 
        src={src} 
        alt={alt} 
        className={clsx('responsive-image__img', styles.responsiveImageImg)}
        {...props}
      />
      {caption && (
        <figcaption 
          className={clsx('responsive-image__caption', styles.responsiveImageCaption)}>
          {caption}
        </figcaption>
      )}
    </figure>
  );
}

export default ResponsiveImage;