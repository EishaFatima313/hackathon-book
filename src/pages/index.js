import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <h1 className={clsx('hero__title', styles.heroTitle)}>
            {siteConfig.title}
          </h1>
          <p className={clsx('hero__subtitle', styles.heroSubtitle)}>
            {siteConfig.tagline}
          </p>
          <p className={styles.heroDescription}>
            Master modern robotics with ROS 2, AI integration, and digital twin simulations. 
            From beginner to advanced — your journey starts here.
          </p>
          <div className={styles.heroButtons}>
            <Link
              className={clsx('button', 'button--primary', 'button--lg', styles.heroButton)}
              to="/docs/ros2-basics/introduction">
              <span>Start Learning</span>
              <svg width="20" height="20" viewBox="0 0 20 20" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M7.5 15L12.5 10L7.5 5" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
              </svg>
            </Link>
            <Link
              className={clsx('button', 'button--outline', 'button--lg', styles.heroButtonSecondary)}
              to="/docs/digital-twin/">
              Explore Digital Twin
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

function FeatureCard({icon, title, description, link, color}) {
  return (
    <div className={clsx('col', 'col--4', styles.featureCol)}>
      <Link to={link} className={styles.featureLink}>
        <div className={clsx('card', styles.featureCard)}>
          <div className={styles.featureCardIcon} style={{backgroundColor: color}}>
            {icon}
          </div>
          <div className={styles.featureCardBody}>
            <h3 className={styles.featureCardTitle}>{title}</h3>
            <p className={styles.featureCardText}>{description}</p>
            <div className={styles.featureCardLink}>
              Learn more
              <svg width="16" height="16" viewBox="0 0 16 16" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M6 12L10 8L6 4" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
              </svg>
            </div>
          </div>
        </div>
      </Link>
    </div>
  );
}

function HomepageFeatures() {
  const features = [
    {
      icon: (
        <svg width="32" height="32" viewBox="0 0 32 32" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M16 4L4 10V22L16 28L28 22V10L16 4Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          <path d="M16 4V16" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          <path d="M16 16L28 10" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          <path d="M16 16L4 10" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
        </svg>
      ),
      title: 'ROS 2 Fundamentals',
      description: 'Master the core concepts of ROS 2 including nodes, topics, services, actions, and custom messages with hands-on tutorials.',
      link: '/docs/ros2-basics/introduction',
      color: 'var(--ifm-color-primary)',
    },
    {
      icon: (
        <svg width="32" height="32" viewBox="0 0 32 32" fill="none" xmlns="http://www.w3.org/2000/svg">
          <circle cx="16" cy="16" r="12" stroke="currentColor" strokeWidth="2"/>
          <path d="M16 8V16L20 20" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
        </svg>
      ),
      title: 'Digital Twin Simulation',
      description: 'Build realistic robot simulations in Gazebo and Unity. Test your robots in virtual environments before deploying to hardware.',
      link: '/docs/digital-twin/',
      color: 'var(--ifm-color-success)',
    },
    {
      icon: (
        <svg width="32" height="32" viewBox="0 0 32 32" fill="none" xmlns="http://www.w3.org/2000/svg">
          <rect x="6" y="6" width="20" height="20" rx="4" stroke="currentColor" strokeWidth="2"/>
          <path d="M12 12L20 20" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          <path d="M20 12L12 20" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
        </svg>
      ),
      title: 'AI-Robot Integration',
      description: 'Connect AI agents to robot controllers using NVIDIA Isaac. Enable intelligent behaviors and autonomous decision-making.',
      link: '/docs/ai-robot-brain/',
      color: 'var(--ifm-color-info)',
    },
    {
      icon: (
        <svg width="32" height="32" viewBox="0 0 32 32" fill="none" xmlns="http://www.w3.org/2000/svg">
          <circle cx="16" cy="10" r="4" stroke="currentColor" strokeWidth="2"/>
          <path d="M8 26C8 26 10 18 16 18C22 18 24 26 24 26" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
        </svg>
      ),
      title: 'Vision-Language-Action',
      description: 'Explore cutting-edge VLA models that enable robots to understand natural language commands and visual contexts.',
      link: '/docs/vision-language-action/',
      color: 'var(--ifm-color-warning)',
    },
    {
      icon: (
        <svg width="32" height="32" viewBox="0 0 32 32" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M16 4V28" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          <path d="M4 16H28" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          <circle cx="16" cy="16" r="6" stroke="currentColor" strokeWidth="2"/>
        </svg>
      ),
      title: 'Humanoid Robots',
      description: 'Design and simulate humanoid robots using URDF. Learn advanced kinematics, dynamics, and motion planning.',
      link: '/docs/ros2-basics/setup',
      color: 'var(--ifm-color-danger)',
    },
    {
      icon: (
        <svg width="32" height="32" viewBox="0 0 32 32" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M16 4L28 10L16 16L4 10L16 4Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          <path d="M28 10V22L16 28V16" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          <path d="M4 10V22L16 28V16" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
        </svg>
      ),
      title: 'Hands-on Projects',
      description: 'Build real-world robotics projects with step-by-step guidance. From simple publishers to complex multi-robot systems.',
      link: '/docs/ros2-basics/examples/publisher-tutorial',
      color: 'var(--ifm-color-primary-light)',
    },
  ];

  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.featuresHeader}>
          <h2 className={styles.featuresTitle}>Learning Paths</h2>
          <p className={styles.featuresSubtitle}>
            Structured modules designed to take you from ROS 2 basics to advanced AI-powered robotics
          </p>
        </div>
        <div className="row">
          {features.map((props, idx) => (
            <FeatureCard key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

function QuickStartSection() {
  return (
    <section className={styles.quickStart}>
      <div className="container">
        <div className={styles.quickStartContent}>
          <div className={styles.quickStartText}>
            <h2 className={styles.quickStartTitle}>Ready to Get Started?</h2>
            <p className={styles.quickStartDescription}>
              Begin your robotics journey with our comprehensive introduction to ROS 2. 
              No prior experience required — we'll guide you through every step.
            </p>
            <div className={styles.quickStartButtons}>
              <Link
                className={clsx('button', 'button--primary', 'button--lg')}
                to="/docs/ros2-basics/introduction">
                Start with ROS 2 Basics
              </Link>
              <Link
                className={clsx('button', 'button--secondary', 'button--lg')}
                to="/docs/ros2-basics/setup">
                Setup Guide
              </Link>
            </div>
          </div>
          <div className={styles.quickStartVisual}>
            <div className={styles.codePreview}>
              <div className={styles.codePreviewHeader}>
                <div className={styles.codePreviewDots}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
              <pre className={styles.codePreviewBody}>
                <code>{`#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String

class RobotNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('robot_node')
        self.publisher_ = self.create_publisher(
            String, 'robot_commands', 10
        )
        
    def send_command(self, cmd):
        msg = String()
        msg.data = cmd
        self.publisher_.publish(msg)

if __name__ == '__main__':
    rclpy.init()
    node = RobotNode()
    node.send_command("move_forward")`}</code>
              </pre>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Learn Robotics with ROS 2 and AI Integration - Comprehensive tutorials and guides for modern robotics development">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <QuickStartSection />
      </main>
    </Layout>
  );
}
