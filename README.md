# 🧭 ROS Multi-Agent Mission Coordination

### Autonomous Navigation of Three Robots in ROS1 & Gazebo

This repository contains our implementation of the **Mission Coordination Lab** for the  
**M2 Smart Aerospace & Autonomous Systems (SAAS)** program at Université Paris-Saclay.

The project focuses on moving **three differential-drive robots** from their start positions  
to their corresponding **goal flags** (robot1 → flag1, robot2 → flag2, robot3 → flag3) using  
**ROS1, Gazebo, and Python**, while avoiding collisions and demonstrating  
multi-agent coordination strategies.

# authors

### nguyenvietkhanh.hn@gmail.com

### diyari.m.salih@gmail.com

### jeremi91750@gmail.com

---

## 📌 **Objectives**

- Navigate three robots autonomously to their assigned flags.
- Avoid collisions between robots at intersections.
- Use ROS1 topics, messages, and launch files effectively.
- Implement sensing-based strategies using ultrasonic range data.
- Build **robust, modular strategies** that adapt to obstacles or moved flags.
- Develop and compare **multiple coordination strategies** (timing, PID approach, etc.).

---

## 🚀 **Key Features**

### **✔ ROS1 Node Architecture**

- Custom agent node (`agent.py`) for robot control
- Subscribes to:
  - `/robot_i/odom` — robot pose
  - `/robot_i/sonar` — front ultrasonic distance
- Publishes to:
  - `/robot_i/cmd_vel` — linear & angular velocity commands

### **✔ Multi-Agent Navigation**

- Moves three robots concurrently in Gazebo
- Ensures collision-free trajectories
- Includes **timing strategy** for safe multi-robot execution

### **✔ Sensing & Control**

- Ultrasonic sensing (5 m range)
- Odometry-based pose tracking
- Differential-drive kinematics
- PID-inspired deceleration near goal

### **✔ Strategies Implemented**

| Strategy                       | Description                                                    |
| ------------------------------ | -------------------------------------------------------------- |
| **Basic Motion**               | Direct movement toward goal using pose error                   |
| **PID-like Goal Approach**     | High speed far from goal, slow near goal                       |
| **Timing Strategy**            | Each robot starts at a delayed time offset to avoid collisions |
| **(Optional) Robust Strategy** | Handles moved flags + custom obstacles                         |

---

## 🛠️ **Project Structure**
