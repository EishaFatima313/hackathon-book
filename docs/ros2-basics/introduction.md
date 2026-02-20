---
sidebar_position: 1
---

# Introduction to ROS 2

Welcome to **Module 1: ROS 2 Basics**!

In this module, you will learn the fundamentals of **ROS 2** (Robot Operating System 2), the framework that powers modern robotics.

---

## What is ROS 2?

**ROS 2** is a set of software libraries and tools for building robot applications.

Think of ROS 2 as the **"Android for Robots"** – it provides a standard way to write robot software, just like Android provides a standard way to write phone apps.

```
┌─────────────────────────────────────────────────────────┐
│                    ROS 2 Platform                        │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  Perception  │  │  Planning    │  │  Control     │  │
│  │  (Sensors)   │  │  (Decision)  │  │  (Motors)    │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
│                                                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  Navigation  │  │  Manipulation│  │  Simulation  │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

### Why ROS 2?

ROS 2 solves common robotics challenges:

| Problem | ROS 2 Solution |
|---------|----------------|
| **Hardware diversity** | Works with any robot (wheeled, humanoid, drone, etc.) |
| **Software reuse** | Share code across projects |
| **Real-time control** | Built-in real-time support |
| **Distributed systems** | Multiple computers work together |
| **Production ready** | Designed for real products, not just research |

---

## ROS 2 vs ROS 1

ROS 2 is the **next generation** of ROS:

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| **Communication** | Custom middleware | DDS (standard) |
| **Real-time** | Limited | Full support |
| **Security** | None | Built-in security |
| **Network** | Single network | Multi-network |
| **Status** | Legacy (ended 2025) | Active development |

> **Note:** ROS 1 reached end-of-life in 2025. All new projects should use ROS 2.

---

## ROS 2 Architecture Overview

ROS 2 is built on a **distributed architecture** where multiple programs (called **nodes**) communicate with each other.

### The Basic Building Blocks

```
┌─────────────────────────────────────────────────────────────────┐
│                     ROS 2 System                                │
│                                                                  │
│   ┌─────────────┐         ┌─────────────┐                      │
│   │   Node 1    │         │   Node 2    │                      │
│   │  (Sensor)   │ ──────→ │  (Control)  │                      │
│   │             │  Topic  │             │                      │
│   └─────────────┘         └─────────────┘                      │
│                                                                  │
│   ┌─────────────┐         ┌─────────────┐                      │
│   │   Node 3    │         │   Node 4    │                      │
│   │  (Service)  │ ←─────→ │  (Client)   │                      │
│   │             │  Request│             │                      │
│   └─────────────┘         └─────────────┘                      │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Key Concepts

| Concept | Description | Real-World Analogy |
|---------|-------------|-------------------|
| **Node** | A program that does one job | Like an app on your phone |
| **Topic** | A channel for sending messages | Like a YouTube channel (publish/subscribe) |
| **Message** | Data sent between nodes | Like an email or text message |
| **Service** | Request/response communication | Like calling a function on another computer |
| **Action** | Long-running tasks with feedback | Like ordering food (order → wait → receive) |

---

## Communication Patterns

ROS 2 has three main ways for nodes to talk to each other:

### 1. Topics (Publisher/Subscriber)

**One-to-many** communication – like a radio station broadcasting to listeners.

```
         ┌──────────────┐
         │  Publisher   │
         │   (Node A)   │
         └──────┬───────┘
                │
                │ Topic: /camera/image
                │
        ┌───────┴───────┬───────────────┐
        ↓               ↓               ↓
┌──────────────┐ ┌──────────────┐ ┌──────────────┐
│ Subscriber 1 │ │ Subscriber 2 │ │ Subscriber 3 │
│   (Node B)   │ │   (Node C)   │ │   (Node D)   │
└──────────────┘ └──────────────┘ └──────────────┘
```

**Use when:** You need to broadcast data to multiple listeners (e.g., sensor data).

### 2. Services (Client/Server)

**One-to-one** request/response – like calling a function.

```
┌──────────────┐         ┌──────────────┐
│    Client    │ ──────→ │    Server    │
│   (Node A)   │ Request │   (Node B)   │
│              │ ←────── │              │
│              │ Response│              │
└──────────────┘         └──────────────┘
```

**Use when:** You need a specific answer (e.g., "turn on LED" → "OK").

### 3. Actions

**Long-running tasks** with progress updates – like ordering delivery.

```
┌──────────────┐         ┌──────────────┐
│    Client    │ ──────→ │    Server    │
│   (Node A)   │  Goal   │   (Node B)   │
│              │ ←────── │              │
│              │ Feedback│              │
│              │ ←────── │              │
│              │ Result  │              │
└──────────────┘         └──────────────┘
```

**Use when:** A task takes time (e.g., "navigate to kitchen" → progress updates → "arrived").

---

## A Simple ROS 2 System Example

Imagine a robot that avoids obstacles:

```
┌─────────────────────────────────────────────────────────────────┐
│              Obstacle Avoidance Robot System                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   ┌─────────────┐                                               │
│   │   Camera    │ ──→ [Publishes images to /camera]            │
│   │    Node     │                                               │
│   └─────────────┘                                               │
│          ↓                                                       │
│   ┌─────────────┐                                               │
│   │   Vision    │ ← [Subscribes to /camera]                    │
│   │   Node      │   [Detects obstacles]                        │
│   │             │ → [Publishes to /obstacles]                  │
│   └─────────────┘                                               │
│          ↓                                                       │
│   ┌─────────────┐                                               │
│   │  Navigator  │ ← [Subscribes to /obstacles]                 │
│   │    Node     │   [Plans safe path]                          │
│   │             │ → [Publishes to /cmd_vel]                    │
│   └─────────────┘                                               │
│          ↓                                                       │
│   ┌─────────────┐                                               │
│   │   Motor     │ ← [Subscribes to /cmd_vel]                   │
│   │  Controller │   [Moves the robot]                          │
│   └─────────────┘                                               │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

Each node does **one job** and communicates through **topics**.

---

## Why This Matters for AI + Humanoid Robots

ROS 2 is the **bridge** between AI agents and robot hardware:

```
┌─────────────────────────────────────────────────────────────────┐
│              AI Agent → Humanoid Robot Pipeline                  │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   ┌─────────────┐                                               │
│   │  AI Agent   │ "Pick up the cup"                            │
│   │  (LLM/VLA)  │                                               │
│   └──────┬──────┘                                               │
│          │                                                       │
│          ↓ ROS 2 Messages                                        │
│   ┌─────────────┐                                               │
│   │  Command    │ Convert to joint angles                      │
│   │  Interpreter│                                               │
│   └──────┬──────┘                                               │
│          │                                                       │
│          ↓ /joint_commands                                       │
│   ┌─────────────┐                                               │
│   │  Humanoid   │ Execute movement                             │
│   │  Robot      │                                               │
│   └─────────────┘                                               │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

Understanding ROS 2 is **essential** for connecting AI to robots!

---

## What's Next?

In the following sections, you will:

1. **Core Concepts** – Deep dive into nodes, topics, services, and more
2. **Setup** – Install ROS 2 and create your first workspace
3. **Examples** – Build working publishers, subscribers, and services
4. **Exercises** – Practice what you've learned

Let's continue to [Core Concepts](./core-concepts.md)!

---

## Quick Reference

| Term | Definition |
|------|------------|
| **ROS 2** | Robot Operating System 2 |
| **Node** | A single program in ROS 2 |
| **Topic** | A broadcast communication channel |
| **Service** | A request/response communication |
| **Action** | A long-running task with feedback |
| **rclpy** | ROS 2 client library for Python |
| **rclcpp** | ROS 2 client library for C++ |
| **DDS** | Data Distribution Service (ROS 2 middleware) |

---

## Resources

- [Official ROS 2 Documentation](https://docs.ros.org/)
- [ROS 2 Tutorials](https://docs.ros.org/en/rolling/Tutorials.html)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
