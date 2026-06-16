# 2D-LiDAR-SLAM-MentorPi-Gazebo

This project focuses on testing, in simulation, different localization and mapping approaches for a mobile robot.

The methods explored include:

Differential odometry alone, with 3D point accumulation for map building
Differential odometry used as input to a SLAM algorithm
Kalman filtering for sensor fusion between differential odometry and IMU measurements, with 3D point accumulation for map building
Kalman filtering for sensor fusion between differential odometry and IMU measurements, used as input to a SLAM algorithm

The robot used in this project is the Hiwonder MentorPi Mecanum, configured and used as a differential-drive robot. The simulation environment is Gazebo.

## How to Run

```bash
ros2 launch mentorpi_simulation display.launch.py use_rviz:=True
```

## Results

