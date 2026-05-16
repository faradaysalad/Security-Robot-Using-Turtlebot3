# Security-Robot-Using-Turtlebot3

An autonomous office security patrol robot simulation using TurtleBot3, ROS Noetic, and Gazebo. The robot performs automated patrols and detects door status (opened/closed) using door yaw orientation in a simulated office environment.

---

## 📌 Project Overview

This project was developed using ROS Noetic and TurtleBot3 Waffle Pi in a fully simulated Gazebo environment.

The system is capable of:
- Performing autonomous patrols around an office environment
- Navigating between checkpoints using ROS navigation stack
- Detecting whether doors are opened or closed
- Identifying potential intrusions when doors are unexpectedly opened

---

## 📽️ Demo Video

Watch the full simulation on YouTube:  
📺 **[Click here to view demo](https://youtu.be/JmPvoCIatec?si=J4V4utAkfll-7T1P)

---

## 📦 Features

- ✅ **Autonomous multi-point patrol** around office environment
- 🚪 **Door status detection** using yaw orientation via `/gazebo/get_model_state`
- 🧭 **Autonomous navigation** using `move_base`
- 📍 Integrated with **SLAM** and **RViz**
- 🧠 Optional **image-based door detection** using OpenCV
- 🖥️ Fully tested in simulation (no physical hardware required)

---

## 🧰 Requirements

- Ubuntu 20.04
- ROS Noetic
- TurtleBot3 packages
- Gazebo
- RViz
- Python 3
- OpenCV

---

## ⚙️ Setup Instructions

### 1. Clone the Repository

```bash
git clone https://github.com/faradaysalad/Security-Robot-Using-Turtlebot3.git
cd Security-Robot-Using-Turtlebot3
```

---

### 2. Create Catkin Workspace (If Needed)

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

Move the project folder into the `src` directory.

---

### 3. Build the Workspace

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## ▶️ Running the Project

### Launch Gazebo Simulation

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

---

### Run SLAM Mapping

```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch
```

---

### Launch Navigation Stack

```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/path/to/map_office.yaml
```

---

### Start Autonomous Patrol

```bash
rosrun your_package patrol_doors.py
```

---

### Run Door Yaw Detection

```bash
rosrun your_package door_yaw_checker.py
```

---

### Run Full Patrol + Door Detection System

```bash
rosrun your_package door_patrol_yaw_checker.py
```

---

### Optional: Run OpenCV Door Detection

```bash
rosrun your_package door_image_detector.py
```

---

## 🚪 Door Detection Method

The robot checks whether doors are opened or closed by reading the yaw orientation of door models in Gazebo.

The system:
1. Retrieves door model states using `/gazebo/get_model_state`
2. Converts quaternion orientation into yaw angle
3. Compares yaw values with predefined thresholds
4. Detects abnormal door states as potential intrusions

Example output:

```bash
🚪 Meeting Room Door is CLOSED
🚪 Office Door is OPENED 🚨
```

---

## 🧪 Simulating Opened Doors

You can manually rotate doors in Gazebo using:

```bash
rosservice call /gazebo/set_model_state "model_state:
  model_name: 'door_6'
  pose:
    position: {x: 5.3, y: 5.1, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.6816, w: 0.7317}
  reference_frame: 'world'"
```

---

## 📂 Project Structure

```bash
Security-Robot-Using-Turtlebot3/
│
├── scripts/
│   ├── patrol_doors.py
│   ├── door_yaw_checker.py
│   ├── door_patrol_yaw_checker.py
│   └── door_image_detector.py
│
├── worlds/
│   └── office.world
│
├── maps/
│   ├── map_office.pgm
│   └── map_office.yaml
│
├── launch/
│   └── *.launch
│
└── README.md
```

---


## 🔮 Future Improvements

- AI-based visual door detection
- Telegram/Discord alert integration
- External security log storage
- Real-world TurtleBot3 deployment

---

## 👨‍💻 Authors

- Farah Dania Binti Imam Nawawi

---

## 📚 References

- TurtleBot3 Simulation Documentation  
  https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/

- Dataset of Gazebo Worlds, Models and Maps  
  https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps

- Gazebo Models and Worlds Collection  
  https://github.com/leonhartyao/gazebo_models_worlds_collection

---
