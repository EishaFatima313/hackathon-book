/**
 * Custom Callout Components for Documentation
 * Enhanced admonitions with modern styling
 */

import React from 'react';
import clsx from 'clsx';
import styles from './Callout.module.css';

const icons = {
  note: (
    <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
      <circle cx="10" cy="10" r="8" stroke="currentColor" strokeWidth="2"/>
      <path d="M10 7V10" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
      <circle cx="10" cy="13" r="1" fill="currentColor"/>
    </svg>
  ),
  tip: (
    <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
      <path d="M10 2L12 8L18 10L12 12L10 18L8 12L2 10L8 8L10 2Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    </svg>
  ),
  info: (
    <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
      <circle cx="10" cy="10" r="8" stroke="currentColor" strokeWidth="2"/>
      <path d="M10 9V13" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
      <circle cx="10" cy="7" r="1" fill="currentColor"/>
    </svg>
  ),
  warning: (
    <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
      <path d="M10 3L18 17H2L10 3Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
      <path d="M10 9V12" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
      <circle cx="10" cy="14" r="1" fill="currentColor"/>
    </svg>
  ),
  danger: (
    <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
      <circle cx="10" cy="10" r="8" stroke="currentColor" strokeWidth="2"/>
      <path d="M7 7L13 13" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
      <path d="M13 7L7 13" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
    </svg>
  ),
  success: (
    <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
      <circle cx="10" cy="10" r="8" stroke="currentColor" strokeWidth="2"/>
      <path d="M7 10L9 12L13 8" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    </svg>
  ),
};

function Callout({type = 'note', title, children}) {
  const calloutTypes = {
    note: styles.note,
    tip: styles.tip,
    info: styles.info,
    warning: styles.warning,
    danger: styles.danger,
    success: styles.success,
  };

  const calloutTitles = {
    note: 'Note',
    tip: 'Tip',
    info: 'Info',
    warning: 'Warning',
    danger: 'Danger',
    success: 'Success',
  };

  return (
    <div className={clsx(styles.callout, calloutTypes[type])}>
      <div className={styles.calloutHeader}>
        <div className={styles.calloutIcon}>
          {icons[type]}
        </div>
        <span className={styles.calloutTitle}>{title || calloutTitles[type]}</span>
      </div>
      <div className={styles.calloutContent}>
        {children}
      </div>
    </div>
  );
}

// Pre-configured Callout Variants
export function Note({title, children}) {
  return <Callout type="note" title={title}>{children}</Callout>;
}

export function Tip({title, children}) {
  return <Callout type="tip" title={title}>{children}</Callout>;
}

export function Info({title, children}) {
  return <Callout type="info" title={title}>{children}</Callout>;
}

export function Warning({title, children}) {
  return <Callout type="warning" title={title}>{children}</Callout>;
}

export function Danger({title, children}) {
  return <Callout type="danger" title={title}>{children}</Callout>;
}

export function Success({title, children}) {
  return <Callout type="success" title={title}>{children}</Callout>;
}

// Default export
export default Callout;
