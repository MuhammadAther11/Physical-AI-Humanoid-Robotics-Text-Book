import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import {useThemeConfig} from '@docusaurus/theme-common';
import styles from './MobileNav.module.css';

function MobileNav() {
  const {navbar: {items}} = useThemeConfig();

  return (
    <nav 
      className={clsx('mobile-nav', styles.mobileNav)} 
      aria-label="Mobile navigation">
      <div className={clsx('mobile-nav__container', styles.mobileNavContainer)}>
        <ul className={clsx('mobile-nav__list', styles.mobileNavList)}>
          {items?.map((item, idx) => (
            <li key={idx} className={clsx('mobile-nav__item', styles.mobileNavItem)}>
              <Link
                to={item.href || item.to}
                className={clsx('mobile-nav__link', styles.mobileNavLink)}>
                {item.label}
              </Link>
            </li>
          ))}
        </ul>
      </div>
    </nav>
  );
}

export default MobileNav;