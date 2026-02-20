# Robotics Education Platform

This project contains educational materials for learning ROS 2 (Robot Operating System 2) with a focus on connecting AI agents to humanoid robot control.

## Getting Started

### Prerequisites

- Node.js (version 16.14 or higher)
- npm or yarn package manager

### Installation

1. Clone or download this repository
2. Navigate to the project directory
3. Install dependencies:

```bash
npm install
```

### Running the Documentation Locally

To start the development server and view the documentation in your browser:

```bash
npm start
```

This will start a local development server and open the documentation in your default browser at `http://localhost:3000`.

### Building for Production

To build the static website for deployment:

```bash
npm run build
```

## Course Modules

The course is organized into three main chapters:

### 1. ROS 2 Basics
- Introduction to ROS 2 concepts (nodes, topics, services, messages)
- ROS 2 architecture overview
- Simple rclpy examples

### 2. AI Agents → Robot Control
- Using rclpy to publish/subscribe to ROS topics
- Bridging Python AI agents to ROS controllers
- Example implementations of command interpretation

### 3. Humanoid Structure with URDF
- Understanding URDF (Unified Robot Description Format)
- Creating humanoid robot models
- Visualization techniques using RViz and Gazebo

## Code Examples

Sample code for each chapter is located in the `docs/code-examples/` directory:

- `publisher_example.py` and `subscriber_example.py` - Basic ROS 2 publisher/subscriber
- `ai_navigation_agent.py` - AI agent implementation for robot navigation
- `simple_humanoid.urdf` - Basic humanoid robot model in URDF format

## Contributing

Feel free to submit issues and enhancement requests. For major changes, please open an issue first to discuss what you would like to change.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details."# hackathon-book" 
