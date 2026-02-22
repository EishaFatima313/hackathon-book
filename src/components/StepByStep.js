/**
 * Step by Step Component
 * For tutorials and procedural documentation
 */

import React from 'react';
import clsx from 'clsx';
import styles from './StepByStep.module.css';

export function Step({number, title, children}) {
  return (
    <div className={styles.step}>
      <div className={styles.stepHeader}>
        <div className={styles.stepNumber}>{number}</div>
        {title && <h3 className={styles.stepTitle}>{title}</h3>}
      </div>
      <div className={styles.stepContent}>
        {children}
      </div>
    </div>
  );
}

export function StepGroup({children, className}) {
  return (
    <div className={clsx(styles.stepGroup, className)}>
      {children}
    </div>
  );
}

export default Step;
