# Sensor Fusion Projects Overview

Welcome to the Sensor Fusion Course repository! 
This repo contains multiple projects focused on perception algorithms used in autonomous vehicles, covering LiDAR, camera, and radar sensor data processing.

## Projects Included

### 1. LiDAR Obstacle Detection  
- Uses LiDAR point clouds to detect and segment obstacles.  
- Key algorithms:  
  - Ground plane segmentation using **RANSAC**  
  - Object clustering using **Euclidean clustering** with **KD-Trees**  
  - Bounding box fitting (axis-aligned and oriented)
  - ![OUP1](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Lidar_Obstacle_Detection/Lidar2Gif.gif) 

### 2. 2D Camera Feature Tracking  
- Detects and tracks keypoints across image sequences.  
- Key algorithms:  
  - Keypoint detectors: FAST, BRISK, ORB, AKAZE, SIFT, Harris, Shi-Tomasi  
  - Descriptors: BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT  
  - Descriptor matching with Brute Force and FLANN matchers  
  - Evaluation of detector-descriptor combinations for optimal feature tracking
  - <img src="https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_2D_Feature_Matching/Results/FAST_BRIEF.gif" width="1800" height="250" />

### 3. 3D Object Tracking  
- Combines 2D camera detection with LiDAR data to track objects in 3D.  
- Key steps:  
  - Object detection using YOLO deep learning framework  
  - Associating bounding boxes between frames via keypoint matches  
  - Computing Time-To-Collision (TTC) from camera and LiDAR data
  - <img src="https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_3D_Object_Tracking/WriteupImages/3DTrackingLidCame.gif" width="1200" height="300" />

### 4. Radar Target Generation and Detection  
- Simulates FMCW radar waveform and target motion.  
- Key algorithms:  
  - FMCW waveform design and beat signal generation  
  - Range FFT and Doppler FFT for range and velocity estimation  
  - 2D CFAR (Constant False Alarm Rate) for target detection in Range-Doppler maps
  - <img src="https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Radar_Target_Generation_And_Detection/Images/2DFFT.png" width="700" height="450" />

### 5. Unscented Kalman Filter (UKF)  
- Implements the UKF for sensor fusion and object state estimation.  
- Key features:  
  - Nonlinear state estimation using CTRV(Constant Turn Rate Velocity Model) and sigma points approximation
  - Fusion of measurements from multiple sensors (e.g., radar and lidar)  
  - Tracking multiple object position, velocity, and orientation
  - ![OUP1](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Lidar_Obstacle_Detection/Lidar2Gif.gif) 

---

Each project demonstrates foundational perception and sensor fusion techniques critical to autonomous driving applications.

Feel free to explore the individual folders for detailed implementations, data, and results!

