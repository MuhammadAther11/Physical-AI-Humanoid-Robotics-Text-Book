import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './TouchNav.module.css';

function TouchNav() {
  return (
    <div className={clsx('touch-nav', styles.touchNav)}>
      <div className={clsx('touch-nav__container', styles.touchNavContainer)}>
        <div className={clsx('touch-nav__section', styles.touchNavSection)}>
          <h3 className={clsx('touch-nav__section-title', styles.touchNavSectionTitle)}>Quick Access</h3>
          <div className={clsx('touch-nav__grid', styles.touchNavGrid)}>
            <Link 
              to="/docs/category/getting-started" 
              className={clsx('touch-nav__item', styles.touchNavItem)}>
              <div className={clsx('touch-nav__icon', styles.touchNavIcon)}>🚀</div>
              <span className={clsx('touch-nav__label', styles.touchNavLabel)}>Start</span>
            </Link>
            <Link 
              to="/docs/category/tutorials" 
              className={clsx('touch-nav__item', styles.touchNavItem)}>
              <div className={clsx('touch-nav__icon', styles.touchNavIcon)}>📚</div>
              <span className={clsx('touch-nav__label', styles.touchNavLabel)}>Tutorials</span>
            </Link>
            <Link 
              to="/docs/category/api-reference" 
              className={clsx('touch-nav__item', styles.touchNavItem)}>
              <div className={clsx('touch-nav__icon', styles.touchNavIcon)}>⚙️</div>
              <span className={clsx('touch-nav__label', styles.touchNavLabel)}>API</span>
            </Link>
            <Link 
              to="/docs/category/troubleshooting" 
              className={clsx('touch-nav__item', styles.touchNavItem)}>
              <div className={clsx('touch-nav__icon', styles.touchNavIcon)}>🔧</div>
              <span className={clsx('touch-nav__label', styles.touchNavLabel)}>Help</span>
            </Link>
          </div>
        </div>
        
        <div className={clsx('touch-nav__section', styles.touchNavSection)}>
          <h3 className={clsx('touch-nav__section-title', styles.touchNavSectionTitle)}>Popular Topics</h3>
          <div className={clsx('touch-nav__list', styles.touchNavList)}>
            <Link 
              to="/docs/module-1-ros2-nervous-system/01-ros2-fundamentals" 
              className={clsx('touch-nav__list-item', styles.touchNavListItem)}>
              ROS 2 Fundamentals
            </Link>
            <Link 
              to="/docs/module-3-ai-robot-brain/01-isaac-overview" 
              className={clsx('touch-nav__list-item', styles.touchNavListItem)}>
              Isaac Overview
            </Link>
            <Link 
              to="/docs/module-4-vla/vla-overview" 
              className={clsx('touch-nav__list-item', styles.touchNavListItem)}>
              VLA Pipeline
            </Link>
          </div>
        </div>
      </div>
    </div>
  );
}

export default TouchNav;