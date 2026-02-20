---
sidebar_position: 3
---

# ROS 2 Setup Guide

This guide will help you install ROS 2 and create your first workspace.

---

## Prerequisites

Before installing ROS 2, ensure you have:

- **Operating System**: Ubuntu 22.04 (recommended), Ubuntu 24.04, Windows 10/11, or macOS
- **Disk Space**: At least 5 GB free
- **RAM**: 4 GB minimum, 8 GB recommended
- **Internet**: For downloading packages

---

## Installation on Ubuntu

### Step 1: Configure Locale

ROS 2 requires UTF-8 locale support:

```bash
locale  # Check if you have UTF-8

# If not, configure it:
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Step 2: Setup Sources

```bash
# Install required packages
sudo apt install software-properties-common

# Add the ROS 2 GPG key
sudo apt update && sudo apt install curl gnupg
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS 2

```bash
# Update package index
sudo apt update

# Install ROS 2 Humble (LTS version)
sudo apt install ros-humble-desktop

# Or install the full version (includes more tools)
sudo apt install ros-humble-desktop-full
```

### Step 4: Setup Environment

```bash
# Source the setup file (for current terminal)
source /opt/ros/humble/setup.bash

# Make it permanent - add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 5: Install Development Tools

```bash
# Install colcon (build tool) and other utilities
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update
```

---

## Installation on Windows

### Step 1: Install Prerequisites

1. Install **Visual Studio 2022** with "Desktop development with C++"
2. Install **Python 3.10+** from python.org
3. Install **CMake** from cmake.org

### Step 2: Download ROS 2

Download the ROS 2 Humble installer from:
https://github.com/ros2/ros2/releases

### Step 3: Run Installer

1. Run the downloaded `.exe` file
2. Follow the installation wizard
3. Choose installation location (default: `C:\dev\ros2_humble`)

### Step 4: Setup Environment

```cmd
# In Command Prompt
call C:\dev\ros2_humble\local_setup.bat

# For PowerShell
& C:\dev\ros2_humble\local_setup.ps1
```

### Step 5: Verify Installation

```cmd
# Run turtlesim
ros2 run turtlesim turtlesim_node
```

---

## Installation on macOS

### Step 1: Install Dependencies

```bash
# Install Xcode command line tools
xcode-select --install

# Install Homebrew (if not installed)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install dependencies
brew install python3 cmake openssl opencv eigen
```

### Step 2: Install ROS 2 via Homebrew

```bash
# Add ROS 2 tap
brew tap osrf/simulation

# Install ROS 2 Humble
brew install ros-humble
```

### Step 3: Setup Environment

```bash
# Source the setup file
source /opt/ros/humble/setup.bash

# Add to ~/.zshrc for persistence
echo "source /opt/ros/humble/setup.bash" >> ~/.zshrc
```

---

## Verify Your Installation

### Test 1: Run Turtlesim

```bash
# Terminal 1 - Start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2 - Send commands
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

You should see a turtle moving in a window!

### Test 2: Check ROS 2 Version

```bash
ros2 --version
```

Expected output: `ros2 0.19.x`

### Test 3: List Available Packages

```bash
ros2 pkg list | head -20
```

---

## Create Your First ROS 2 Workspace

### Step 1: Create Workspace Structure

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### Step 2: Create a Package

```bash
# Navigate to src directory
cd src

# Create a Python package
ros2 pkg create --build-type ament_python my_first_package

# Or create a C++ package
ros2 pkg create --build-type ament_cmake my_first_package_cpp
```

### Step 3: Explore the Package Structure

```
my_first_package/
├── package.xml              # Package metadata
├── setup.py                 # Python setup file
├── setup.cfg                # Configuration
└── my_first_package/        # Source code directory
    └── __init__.py
```

### Step 4: Build the Workspace

```bash
# Go back to workspace root
cd ~/ros2_ws

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

### Step 5: Run Your Package

```bash
# List executables in your package
ros2 pkg executables my_first_package

# Run an executable (if you created one)
ros2 run my_first_package my_node
```

---

## Configure Your Development Environment

### VS Code Setup (Recommended)

1. Install **VS Code** from code.visualstudio.com
2. Install extensions:
   - **ROS** (by Microsoft)
   - **Python** (by Microsoft)
   - **C/C++** (by Microsoft)

### VS Code Settings

Create `.vscode/settings.json` in your workspace:

```json
{
    "python.autoComplete.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages"
    ],
    "python.analysis.extraPaths": [
        "/opt/ros/humble/lib/python3.10/site-packages"
    ],
    "C_Cpp.default.includePath": [
        "/opt/ros/humble/include"
    ]
}
```

---

## Common Issues and Solutions

### Issue 1: "Command not found: ros2"

**Solution:** Source the setup file:
```bash
source /opt/ros/humble/setup.bash
```

### Issue 2: "Unable to locate package ros-humble-desktop"

**Solution:** Make sure you added the repository correctly:
```bash
sudo apt update
sudo apt install software-properties-common
# Then re-add the repository (see Step 2 above)
```

### Issue 3: Build fails with "ModuleNotFoundError"

**Solution:** Source your workspace:
```bash
cd ~/ros2_ws
source install/setup.bash
```

### Issue 4: Permission denied when running rosdep

**Solution:** Use sudo:
```bash
sudo rosdep init
rosdep update
```

---

## Essential ROS 2 Environment Variables

| Variable | Description | Example |
|----------|-------------|---------|
| `ROS_DOMAIN_ID` | Isolate ROS 2 networks | `export ROS_DOMAIN_ID=5` |
| `RMW_IMPLEMENTATION` | Choose middleware | `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` |
| `ROS_LOG_DIR` | Log file location | `export ROS_LOG_DIR=~/ros2_logs` |

---

## Next Steps

Now that ROS 2 is installed, let's create some working examples!

Continue to [Publisher Tutorial](./examples/publisher-tutorial.md).

---

## Quick Reference

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Create workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws

# Create package
ros2 pkg create --build-type ament_python my_package

# Build
colcon build
source install/setup.bash

# Run node
ros2 run my_package my_node

# Check installation
ros2 run turtlesim turtlesim_node
```
