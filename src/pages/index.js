import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/ros2-basics/introduction">
            Get Started with ROS 2
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into the meta tag">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <h3>ROS 2 Fundamentals</h3>
                <p>Learn the core concepts of ROS 2 including nodes, topics, services, and messages.</p>
              </div>
              <div className="col col--4">
                <h3>AI Integration</h3>
                <p>Connect AI agents to robot controllers and enable intelligent robot behaviors.</p>
              </div>
              <div className="col col--4">
                <h3>Humanoid Robots</h3>
                <p>Design and simulate humanoid robots using URDF and advanced kinematics.</p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}