# TurtleBot3 SLAM using ROS1 in a custom environment

This project demonstrates real-time Simultaneous Localization and Mapping (SLAM) using TurtleBot3 in the Robot Operating System (ROS1) framework.

We implement 2D SLAM using LiDAR-based scan matching to build an occupancy grid map of an unknown indoor environment. The system estimates the robot’s pose while incrementally constructing the map.

🔹 Key Features

Real-time 2D mapping using LiDAR data

Odometry and TF frame integration (odom → base_link → map)

Launch-based modular ROS architecture

# Map saving and reloading capability

Visualization in RViz

🔹 SLAM Algorithm

This project uses Gmapping, a Rao-Blackwellized Particle Filter-based SLAM algorithm, to generate accurate occupancy grid maps.

🔹 System Workflow

LiDAR publishes /scan data

Odometry publishes /odom

SLAM node fuses sensor data

Map is published to /map

Visualization through RViz

🔹 Applications

Autonomous navigation

Indoor mapping
