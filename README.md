# 🐢 ROS2 Turtlesim Controller & Monitor


A ROS2 package that implements a **multi-node control system** for `turtlesim`. This project demonstrates **process management**, **custom service calls**, **subscriber/publisher patterns**, and **safety monitoring** within the ROS2 ecosystem.

---

## 📑 Table of Contents
- [Overview](#-overview)
- [Project Architecture](#-project-architecture)
- [Installation & Build](#-installation--build)
- [Usage](#-usage)
- [Nodes Description](#-nodes-description)
- [Safety Features](#-safety-features)

---

## 📖 Overview
This package allows users to interactively control **two turtles** in a simulation while a background monitoring process ensures safety. It features:

- ✅ **Automatic Setup**: Spawns a second turtle immediately upon launch.
- ✅ **Interactive UI**: A dedicated terminal interface for sending velocity commands.
- ✅ **Safety Monitor**: A high-priority node that overrides user commands to prevent collisions or boundary breaches.

---

## 📂 Project Architecture
```plaintext
assignment1_rt/
├── launch/
│   └── assignment1.launch.py      # Main launch file (Orchestrator)
├── assignment1_rt/
│   ├── __init__.py
│   ├── ui_node.py                 # User Interface (Input)
│   ├── distance_monitor.py        # Safety Controller (Logic)
│   ├── distance_checks.py         # Helper Class (Calculations)
│   └── turtle_spawn.py            # Service Client (Setup)
├── package.xml
├── setup.py
└── setup.cfg
```

---

## ⚙️ Installation & Build
Assuming **ROS2 Humble** and its dependencies are already installed:

1. Navigate to your ROS2 workspace `src` folder:
   ```bash
   cd ~/ros2_ws/src
   ```
2. Clone the repository from GitHub:
   ```bash
   git clone https://github.com/Belal-Elmikkawy/Assignment1_RT1.git
   ```
3. Go back to the workspace root and install dependencies:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```
4. Build the package:
   ```bash
   colcon build --packages-select assignment1_rt
   ```
5. Source the setup file:
   ```bash
   source install/setup.bash
   ```

---

## 🚀 Usage
Run the entire project with:
```bash
ros2 launch assignment1_rt assignment1.launch.py
```

### The Interface
A new terminal window titled **`ui_node`** will appear. Use it to control the robots:

- **Select Robot**: `turtle1` or `turtle2`
- **Linear X**: Speed forward (+) or backward (-)
- **Angular Z**: Rotation speed (Clockwise/Counter-Clockwise)

> **Note:** The turtle moves for **1.0 second** and then stops automatically.

---

## 📦 Nodes Description

### 1. `ui_node.py` – User Interface Node
- **Purpose**: Provides an interactive terminal interface for the user.
- **Functionality**:
  - Captures user input for selecting the turtle (`turtle1` or `turtle2`).
  - Accepts velocity commands: Linear X and Angular Z.
  - Publishes `geometry_msgs/Twist` messages to the corresponding topic (`/turtleX/cmd_vel`).
- **Interaction**:
  - Works independently but sends commands that can be overridden by the safety monitor.

### 2. `distance_monitor.py` – Safety Controller Node
- **Purpose**: Ensures safe operation of both turtles.
- **Functionality**:
  - Subscribes to `/turtle1/pose` and `/turtle2/pose` topics.
  - Calculates the Euclidean distance between turtles using helper functions.
  - Checks for collisions and boundary violations.
  - Publishes zero velocity to turtles if safety conditions are breached.
- **Interaction**:
  - Overrides commands from `ui_node` when safety limits are exceeded.

### 3. `distance_checks.py` – Helper Class
- **Purpose**: Provides reusable functions for distance and boundary calculations.
- **Functionality**:
  - Computes Euclidean distance between two points.
  - Validates if a turtle is within safe boundaries.
- **Interaction**:
  - Used internally by `distance_monitor.py` for logic processing.

### 4. `turtle_spawn.py` – Service Client Node
- **Purpose**: Spawns the second turtle in the simulation.
- **Functionality**:
  - Calls the `/spawn` service provided by `turtlesim`.
  - Places `turtle2` at coordinates `(7.0, 7.0)` with a default orientation.
- **Interaction**:
  - Runs once at launch to set up the environment.

### 5. `turtlesim_node` – Simulator Node
- **Purpose**: The standard ROS2 turtlesim node.
- **Functionality**:
  - Provides the simulation environment for the turtles.
  - Offers topics like `/turtleX/cmd_vel` and `/turtleX/pose` for control and monitoring.
- **Interaction**:
  - Acts as the base system that other nodes interact with.

---

## 🛡 Safety Features
The `distance_monitor` node runs at **10Hz** and performs:

- **Collision Detection**:
  - **Condition**: Distance between `turtle1` and `turtle2` < `2.0`.
  - **Action**: Publishes `0.0` velocity to both turtles immediately.
- **Boundary Constraints**:
  - **Condition**: Any turtle gets closer than `1.0` unit to the wall (`0` or `11`).
  - **Action**: Stops the specific turtle approaching the wall.

When a safety breach occurs, a warning is logged:
```plaintext
[WARN] [distance_monitor]: Turtles too close! Dist: 1.45
```

---
