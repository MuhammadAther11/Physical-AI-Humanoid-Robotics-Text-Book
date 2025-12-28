import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import {useLocation} from '@docusaurus/router';
import styles from './Toc.module.css';

function Toc() {
  const location = useLocation();
  
  // This is a simplified implementation - in a real scenario, 
  // this would extract headings from the current page
  const mockTocItems = [
    {id: 'introduction', text: 'Introduction', level: 2},
    {id: 'setup', text: 'Setup', level: 2},
    {id: 'configuration', text: 'Configuration', level: 2},
    {id: 'usage', text: 'Usage', level: 2},
    {id: 'examples', text: 'Examples', level: 2},
    {id: 'troubleshooting', text: 'Troubleshooting', level: 2},
    {id: 'faq', text: 'FAQ', level: 2},
  ];

  if (mockTocItems.length === 0) {
    return null;
  }

  return (
    <div className={clsx('toc', styles.toc)}>
      <h3 className={clsx('toc__title', styles.tocTitle)}>On this page</h3>
      <ul className={clsx('toc__list', styles.tocList)}>
        {mockTocItems.map((item, index) => (
          <li 
            key={index} 
            className={clsx('toc__item', styles.tocItem, {
              [`toc__level-${item.level}`]: true,
            })}>
            <Link 
              href={`#${item.id}`} 
              className={clsx('toc__link', styles.tocLink, {
                [`toc__link--active`]: location.hash === `#${item.id}`
              })}>
              {item.text}
            </Link>
          </li>
        ))}
      </ul>
    </div>
  );
}

export default Toc;