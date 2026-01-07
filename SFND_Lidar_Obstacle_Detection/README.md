# LiDAR Obstacle Detection:
### What is Lidar?
LiDAR (Light Detection and Ranging) is a remote sensing technology that measures distance by firing rapid laser pulses and timing how long they take to reflect back from objects. By collecting millions of these points per second, LiDAR builds a precise 3D map of the environment.
It’s widely used in autonomous vehicles for detecting obstacles, road surfaces, and surrounding objects.
<Image Hesai>
<Image OSI>

### What is a PointCloud?
A point cloud is a collection of 3D data points that represent the shape and location of objects in the environment.
Each point stores coordinates (x, y, z), and sometimes intensity or color, captured by sensors like LiDAR.
When combined, these points form a detailed digital model of the real world that algorithms can process to detect obstacles, surfaces, and objects.
<Image PCD>

### The PCL Library
The [Point Cloud Library (PCL)](https://pointclouds.org/) is an open-source C++ framework designed for working with 2D/3D point cloud data.
It provides ready-to-use tools for filtering, segmentation, registration, clustering, feature extraction, and visualization of LiDAR data.
PCL is widely used in robotics, autonomous driving, and perception systems because of its speed, flexibility, and large algorithm collection.

### Why LiDAR fro Autonomous Vehicles
LiDAR provides high-resolution 3D perception, allowing self-driving cars to precisely measure distance to objects and understand the shape of the environment.
It works reliably in day or night and is widely used alongside cameras and radar for safe navigation.
-  Advantages
    - Provides **high-resolution 3D distance measurements**
    - Works well **day or night**, independent of lighting conditions
    - Detects obstacles, road edges, and shapes **with great accuracy**
- Disadvantages
    - **More expensive** than camera-based systems
    - Can struggle in **rain, fog, or snow**
    - Generates large data → **requires high computing power**
- Can these disadvantages be reduced?
    - Regarding cost, there is a clear downward trend in price of LiDARs as below
    <Image LidarCost>
    - Yes. Many limitations of LiDAR can be reduced through **sensor fusion** — combining LiDAR with cameras, radar, GPS/IMU, and maps.
    - Cameras provide color, texture, and sign/lane understanding
    - Radar performs better in rain, fog, and long range
    - Fusion creates a **more reliable and complete perception** than any single sensor alone
 
## 🧩 Point Cloud Segmentation in Autonomous Vehicles

### Why Segment Point Clouds?
Segmentation helps split the 3D point cloud into **meaningful groups**, such as:
- **Ground vs obstacles**
- Free space where the car can drive

Without segmentation, all points are just raw data.  
With segmentation, perception systems can classify what is **drivable**, what must be **avoided**, and what objects should be **tracked**.

---

### RANSAC for Ground Plane Segmentation

#### What is RANSAC?
RANSAC stands for Random Sample Consensus, and is a method for detecting outliers in data. RANSAC runs for a max number of iterations, and returns the model with the best fit. Each iteration randomly picks a subsample of the data and fits a model through it, such as a line or a plane. Then the iteration with the highest number of inliers or the lowest noise is used as the best model. Here the inliers refers to our points of interest, example the Lidar points on ground while noise or the ones that are non-ground.
In LiDAR perception,it is often used to estimate the **ground plane** and separate it from obstacles.

Here is a simple GIF on 2D Ransac algorithm working. The red coloured points are the subsets choosen in every iteration and Green Line is the best fit model for that subset choosen. Here our target is to choose the line/model that fits the most points along the diagonal.
< Image ransac-line-ani>

<Image Ransac overview>
In LiDAR perception, we need to deal with planes instead of lines as our point cloud dimension is 3D.

### Why RANSAC is Useful
- Robust against **noise and outliers**
- Works well even if ground is not perfectly flat, may be afew ups and downs
- Efficient enough for real-time perception
RANSAC is a foundational step before object clustering, tracking, and path planning.


### Object Clustering in Point Clouds
Once the ground is removed, the remaining LiDAR points usually belong to **objects**
such as cars, pedestrians, barriers, and poles.  
Object clustering groups nearby points into **individual objects** so that they can
be classified and tracked.

**Note:** The raw point cloud is first **downsampled and filtered** using the PCL library to reduce noise and data size, since LiDAR produces heavy, high-density data. This improves processing speed and makes segmentation and clustering more efficient. 
**Example: Voxel Grid Filtering**
Voxel Grid filtering reduces the number of points by dividing the space into small 3D cubes (voxels)  
and replacing all points inside each voxel with a single representative point (usually the centroid).  
This preserves the overall structure of the environment while **greatly reducing point cloud density** and computation cost.

#### Euclidean Clustering
Euclidean clustering groups points based on **physical distance**.
- Points close together are grouped into one cluster
- Points far apart form separate clusters
- Simple and intuitive — “points that are near each other likely belong
  to the same object”
This works well for separating obstacles within themselves eg. cars from pedestrians, poles, road signs, etc.

---

#### KD-Tree for Fast Search
A **KD-Tree (K-Dimensional Tree)** is a data structure used to quickly find
nearby points in a high-dimensional space.

- Instead of checking distance to every other point (slow, O(N²)),
  KD-Tree allows efficient neighbor lookups (fast, ~O(N log N))
- PCL uses KD-Tree to speed up Euclidean clustering
- Without a KD-Tree, clustering on large point clouds would be too slow
  for real-time driving



# Sensor Fusion Self-Driving Car Course

<img src="https://github.com/awbrown90/SensorFusionHighway/blob/master/media/ObstacleDetectionFPS.gif" width="700" height="400" />

### Welcome to the Sensor Fusion course for self-driving cars.

In this course we will be talking about sensor fusion, whch is the process of taking data from multiple sensors and combining it to give us a better understanding of the world around us. we will mostly be focusing on two sensors, lidar, and radar. By the end we will be fusing the data from these two sensors to track multiple cars on the road, estimating their positions and speed.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

**Radar** data is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction. This ability makes radars a very pratical sensor for doing things like cruise control where its important to know how fast the car infront of you is traveling. Radar sensors are also very affordable and common now of days in newer cars.

**Sensor Fusion** by combing lidar's high resoultion imaging with radar's ability to measure velocity of objects we can get a better understanding of the sorrounding environment than we could using one of the sensors alone.


## Installation

### Linux Ubuntu 16

Install PCL, C++

The link here is very helpful, 
https://larrylisky.com/2014/03/03/installing-pcl-on-ubuntu/

A few updates to the instructions above were needed.

* libvtk needed to be updated to libvtk6-dev instead of (libvtk5-dev). The linker was having trouble locating libvtk5-dev while building, but this might not be a problem for everyone.

* BUILD_visualization needed to be manually turned on, this link shows you how to do that,
http://www.pointclouds.org/documentation/tutorials/building_pcl.php

