# Security-Robot-Using-Turtlebot3
An autonomous security patrol robot simulation using TurtleBot3, ROS Noetic, and Gazebo. The robot performs automated patrols and detects door status (open/closed) using yaw orientation in a simulated office environment.

---

## 📽️ Demo Video
Watch the full simulation on YouTube:  
📺 **[Click here to view demo](https://www.youtube.com/YOUR-DEMO-LINK)** *(Insert your video link here)*

---

## 📦 Features
- ✅ **Autonomous multi-point patrol** around office environment.
- 🚪 **Door status detection** using yaw orientation via `/gazebo/get_model_state`.
- 🧠 Optional **image-based door detection** using OpenCV.
- 📍 Integrated with **SLAM** and **navigation stack** (`move_base`, RViz).
- 🧪 Fully tested in simulation (no physical hardware required).

- ## 🧰 Requirements

- Ubuntu 20.04
- ROS Noetic
- TurtleBot3 packages
- Gazebo (default with ROS Noetic)
- OpenCV (for image detection)
- RViz

---

## ⚙️ Setup Instructions

### 1. Clone the Repository
```bash
cd ~/catkin_ws/src
git clone https://github.com/faradaysalad/Security-Robot-Using-Turtlebot3.git
cd ..
catkin_make
source devel/setup.bash
```bash

### 2. Setup TurtleBot3 Environment
```bash
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
source ~/.bashrc
```bash

