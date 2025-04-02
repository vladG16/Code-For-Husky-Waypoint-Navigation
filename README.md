# Husky Robot Waypoint Navigation (with OptiTrack Integration)

## Overview
This guide walks you through setting up and executing waypoint navigation for the Clearpath Husky robot using ROS Noetic and OptiTrack for external pose tracking. It includes the use of a custom ROS node for sending waypoints and recording travel distance.

---

## Prerequisites
Ensure the following have been installed on your Ubuntu system:

- ROS Noetic
- Clearpath Husky packages
- OptiTrack Motive (on external Windows machine)
- `natnet_ros_cpp` ROS package (for streaming pose from OptiTrack)

---

## Workspace Setup

### 1. Create and Build Catkin Workspace
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## Setup Waypoint Navigation

### 2. Place Scripts in Package
Clone or create a custom package (e.g., `husky_waypoints`) and place your Python scripts in the `scripts/` directory:

- `simple_waypoint_nav.py`: Sends fixed waypoints to `/move_base_simple/goal`
- `waypoint_with_distance.py`: Same as above, also logs and calculates distance traveled

Make the scripts executable:
```bash
chmod +x waypoint_with_distance.py
```

### 3. Create Launch File
Create `husky_waypoint_full.launch` inside the `launch/` directory of your package:
```xml
<launch>
  <include file="$(find husky_gazebo)/launch/husky_playpen.launch" />
  <include file="$(find husky_navigation)/launch/move_base_mapless_demo.launch" />
  <node pkg="husky_waypoints" type="waypoint_with_distance.py" name="waypoint_nav" output="screen" />
</launch>
```

---

## Real Robot (No Simulation)

### 4. Transfer to Husky
- Use `scp` or a USB drive to transfer your package (`husky_waypoints`) to the Husky’s onboard computer.
- Place it inside the existing `catkin_ws/src` directory.

### 5. Build on the Husky
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 6. Launch on the Husky
Run the launch file:
```bash
roslaunch husky_waypoints husky_waypoint_full.launch
```
This starts move_base, loads the robot in Gazebo (or real environment), and runs your waypoint node.

---

## OptiTrack Integration

### 7. Clone and Configure OptiTrack Bridge
```bash
cd ~/catkin_ws/src
git clone https://github.com/L2S-lab/natnet_ros_cpp.git
cd ..
catkin_make
source devel/setup.bash
```

### 8. Update Network Info
Edit `natnet_ros.launch`:
```bash
nano ~/catkin_ws/src/natnet_ros_cpp/launch/natnet_ros.launch
```
Set your client IP (Ubuntu), server IP (OptiTrack Motive PC), and model name.

### 9. Launch OptiTrack Bridge
```bash
roslaunch natnet_ros_cpp natnet_ros.launch
```
Confirm `/optitrack/Husky/pose` topic is streaming.

---

## Plotting Distance (Optional)
Run the plotting script after navigation ends:
```bash
python3 waypoint_distance.py
```
This displays a graph of distance vs time using data saved to `husky_distance.csv`.

---

## Troubleshooting
- Make sure all IP addresses are correct for OptiTrack
- Use `rostopic echo /move_base_simple/goal` to verify goal publishing
- Use `rqt_graph` or `rosnode list` to check node connectivity

---

## Summary
- Launch from your host Ubuntu PC
- Transfer ROS node to the Husky
- Run `natnet_ros_cpp` to stream OptiTrack pose
- Launch waypoint node to drive to goals

This workflow combines high-level autonomy with precise motion tracking using OptiTrack.

