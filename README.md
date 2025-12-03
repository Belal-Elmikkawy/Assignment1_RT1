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


