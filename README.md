# 🐢 ROS2 Turtlesim Controller & Safety Monitor

A ROS2 package that implements a multi-node control system for **turtlesim**, demonstrating process orchestration, custom service calls, subscriber/publisher patterns, and real-time safety monitoring within the ROS2 ecosystem.

---

## 📑 Table of Contents

- [Overview](#-overview)
- [Project Architecture](#-project-architecture)
- [Prerequisites](#-prerequisites)
- [Installation](#-installation)
- [Usage](#-usage)
- [Nodes Description](#-nodes-description)
- [Safety Features](#-safety-features)
- [Troubleshooting](#-troubleshooting)

---

## 📖 Overview

This package enables users to **control two turtles interactively** in turtlesim while a **background monitoring node** enforces collision avoidance and boundary safety.

### Features:
- 🛠 **Automatic Setup:** A second turtle is spawned automatically on launch.  
- 🎮 **Interactive UI:** A dedicated terminal interface for sending velocity commands to each turtle.  
- 🛡 **Safety Monitor:** High-priority logic node that prevents collisions and wall hits by overriding unsafe commands.

---

## 📂 Project Architecture
assignment1_rt/
├── launch/
│ └── assignment1.launch.py # Main launch file (Orchestrator)
├── assignment1_rt/
│ ├── init.py
│ ├── ui_node.py # User Interface (Input)
│ ├── distance_monitor.py # Safety Controller (Logic)
│ ├── distance_checks.py # Helper Class (Calculations)
│ └── turtle_spawn.py # Service Client (Setup)
├── package.xml
├── setup.py
└── setup.cfg


---

## 🛠 Prerequisites

Ensure the following are installed:

### ROS2  
(Humble Hawksbill or newer)

### Turtlesim  
```bash
sudo apt install ros-humble-turtlesim

