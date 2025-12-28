import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import {useLocation} from '@docusaurus/router';
import {usePluralForm} from '@docusaurus/theme-common';
import styles from './Breadcrumb.module.css';

function Breadcrumb() {
  const location = useLocation();
  const {translate} = usePluralForm();
  
  // Simple breadcrumb implementation based on URL path
  const pathSegments = location.pathname
    .split('/')
    .filter(segment => segment !== '')
    .map(segment => {
      // Convert kebab-case to Title Case
      const title = segment
        .split('-')
        .map(word => word.charAt(0).toUpperCase() + word.slice(1))
        .join(' ');
      
      return {
        title,
        path: `/${segment}`,
        url: `#${segment}`
      };
    });

  if (pathSegments.length === 0) {
    return null;
  }

  return (
    <nav
      className={clsx('breadcrumb-nav', styles.breadcrumbNav)}
      aria-label={translate({message: 'Breadcrumb', id: 'breadcrumb.nav.aria.label'})}>
      <ul className={clsx('breadcrumb-nav__list', styles.breadcrumbList)}>
        <li className={clsx('breadcrumb-nav__item', styles.breadcrumbItem)}>
          <Link to="/" className={clsx('breadcrumb-nav__link', styles.breadcrumbLink)}>
            Home
          </Link>
        </li>
        {pathSegments.map((segment, index) => (
          <li 
            key={index} 
            className={clsx('breadcrumb-nav__item', styles.breadcrumbItem, {
              'breadcrumb-nav__item--active': index === pathSegments.length - 1
            })}
            aria-current={index === pathSegments.length - 1 ? 'page' : undefined}>
            <span className={clsx('breadcrumb-nav__divider', styles.breadcrumbDivider)}>›</span>
            {index === pathSegments.length - 1 ? (
              <span className={clsx('breadcrumb-nav__link', styles.breadcrumbLink, styles.breadcrumbLinkActive)}>
                {segment.title}
              </span>
            ) : (
              <Link 
                to={segment.path} 
                className={clsx('breadcrumb-nav__link', styles.breadcrumbLink)}>
                {segment.title}
              </Link>
            )}
          </li>
        ))}
      </ul>
    </nav>
  );
}

export default Breadcrumb;