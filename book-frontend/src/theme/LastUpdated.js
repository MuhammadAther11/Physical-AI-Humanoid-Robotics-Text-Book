import React from 'react';
import clsx from 'clsx';
import styles from './LastUpdated.module.css';

function LastUpdated() {
  const relatedContent = [
    { title: 'Getting Started', url: '/docs/getting-started' },
    { title: 'Configuration', url: '/docs/configuration' },
    { title: 'API Reference', url: '/docs/api' },
    { title: 'Troubleshooting', url: '/docs/troubleshooting' },
  ];

  return (
    <div className={clsx('last-updated', styles.lastUpdated)}>
      <div className={clsx('related-content', styles.relatedContent)}>
        <h3 className={clsx('related-content__title', styles.relatedTitle)}>Related Content</h3>
        <ul className={clsx('related-content__list', styles.relatedList)}>
          {relatedContent.map((item, index) => (
            <li key={index} className={clsx('related-content__item', styles.relatedItem)}>
              <a href={item.url} className={clsx('related-content__link', styles.relatedLink)}>
                {item.title}
              </a>
            </li>
          ))}
        </ul>
      </div>
      <div className={clsx('last-updated__info', styles.lastUpdatedInfo)}>
        <p className={clsx('last-updated__text', styles.lastUpdatedText)}>
          Last updated: {new Date().toLocaleDateString()}
        </p>
      </div>
    </div>
  );
}

export default LastUpdated;