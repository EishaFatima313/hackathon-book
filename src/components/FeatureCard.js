/**
 * Feature Card Component
 * Reusable card component for showcasing features
 */

import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './FeatureCard.module.css';

export function FeatureCard({icon, title, description, link, color, className}) {
  const card = (
    <div className={clsx('card', styles.featureCard, className)}>
      {icon && (
        <div className={styles.featureCardIcon} style={{backgroundColor: color}}>
          {icon}
        </div>
      )}
      <div className={styles.featureCardBody}>
        {title && <h3 className={styles.featureCardTitle}>{title}</h3>}
        {description && (
          <p className={styles.featureCardText}>{description}</p>
        )}
      </div>
    </div>
  );

  if (link) {
    return (
      <Link to={link} className={styles.featureCardLink}>
        {card}
      </Link>
    );
  }

  return card;
}

/**
 * Feature Grid Component
 * Container for multiple feature cards
 */
export function FeatureGrid({children, className}) {
  return (
    <div className={clsx('row', styles.featureGrid, className)}>
      {children}
    </div>
  );
}

/**
 * Feature Item Component
 * Wrapper for individual feature cards in a grid
 */
export function FeatureItem({children, className, col = 4}) {
  const colClasses = {
    3: 'col col--3',
    4: 'col col--4',
    6: 'col col--6',
    12: 'col col--12',
  };

  return (
    <div className={clsx(colClasses[col] || colClasses[4], styles.featureItem, className)}>
      {children}
    </div>
  );
}

export default FeatureCard;
