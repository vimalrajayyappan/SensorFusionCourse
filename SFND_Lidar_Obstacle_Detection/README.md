# LiDAR Obstacle Detection:
![OUP1](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Lidar_Obstacle_Detection/LidarGif.gif)

## 1. LiDAR Intro:
### 1.1. What is Lidar?
LiDAR (Light Detection and Ranging) is a remote sensing technology that measures distance by firing rapid laser pulses and timing how long they take to reflect back from objects. By collecting millions of these points per second, LiDAR builds a precise 3D map of the environment.
It’s widely used in autonomous vehicles for detecting obstacles, road surfaces, and surrounding objects.
Some Lidar OEMs:
- **Ouster** – OS0 / OS1 / OS2 digital LiDAR series
- **Hesai** – Pandar, XT, QT series (widely used in RoboTaxis)
- **Luminar** – Long-range LiDAR for Level 3/4 autonomy

![Hessai](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Lidar_Obstacle_Detection/HessaiLidar.png)

### 1.2. What is a PointCloud?
A point cloud is a collection of 3D data points that represent the shape and location of objects in the environment.
Each point stores coordinates (x, y, z), and sometimes intensity or color, captured by sensors like LiDAR.
When combined, these points form a detailed digital model of the real world that algorithms can process to detect obstacles, surfaces, and objects.

The image below is a map of Pointclouds, one of the roads in Banglore, stitched using Normalized Distribution Transforms. The circular object which you see on the top right of the image is an overhead Water Tank! 

![PCD](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Lidar_Obstacle_Detection/PCD.png)

### 1.3. The PCL Library
The [Point Cloud Library (PCL)](https://pointclouds.org/) is an open-source C++ framework designed for working with 2D/3D point cloud data.
It provides ready-to-use tools for filtering, segmentation, registration, clustering, feature extraction, and visualization of LiDAR data.
PCL is widely used in robotics, autonomous driving, and perception systems because of its speed, flexibility, and large algorithm collection.

### 1.4. Why LiDAR for Autonomous Vehicles
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
      
    ![Cost](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Lidar_Obstacle_Detection/LidarCost.png)

    - Yes. Many limitations of LiDAR can be reduced through **sensor fusion** — combining LiDAR with cameras, radar, GPS/IMU, and maps.
    - Cameras provide color, texture, and sign/lane understanding
    - Radar performs better in rain, fog, and long range
    - Fusion creates a **more reliable and complete perception** than any single sensor alone
 
## 2. Point Cloud Segmentation in Autonomous Vehicles

### 2.1. Why Segment Point Clouds?
Segmentation helps split the 3D point cloud into **meaningful groups**, such as:
- **Ground vs obstacles**
- Free space where the car can drive

Without segmentation, all points are just raw data.  
With segmentation, perception systems can classify what is **drivable**, what must be **avoided**, and what objects should be **tracked**.

### 2.2. RANSAC for Ground Plane Segmentation

#### What is RANSAC?
RANSAC stands for Random Sample Consensus, and is a method for detecting outliers in data. RANSAC runs for a max number of iterations, and returns the model with the best fit. Each iteration randomly picks a subsample of the data and fits a model through it, such as a line or a plane. Then the iteration with the highest number of inliers or the lowest noise is used as the best model. Here the inliers refers to our points of interest, example the Lidar points on ground while noise or the ones that are non-ground.
In LiDAR perception,it is often used to estimate the **ground plane** and separate it from obstacles.

Here is a simple GIF on 2D Ransac algorithm working. The red coloured points are the subsets choosen in every iteration and Green Line is the best fit model for that subset choosen. Here our target is to choose the line/model that fits the most points along the diagonal.
![RANSAC1](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Lidar_Obstacle_Detection/ransac-linie-ani.gif)
In LiDAR perception, we need to deal with planes instead of lines as our point cloud dimension is 3D.
![RANSAC](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Lidar_Obstacle_Detection/RansacOverview.png)

#### Why RANSAC is Useful
- Robust against **noise and outliers**
- Works well even if ground is not perfectly flat, may be afew ups and downs
- Efficient enough for real-time perception
RANSAC is a foundational step before object clustering, tracking, and path planning.


## 3. Object Clustering in Point Clouds
Once the ground is removed, the remaining LiDAR points usually belong to **objects**
such as cars, pedestrians, barriers, and poles.  
Object clustering groups nearby points into **individual objects** so that they can
be classified and tracked.

**Note:** The raw point cloud is first **downsampled and filtered** using the PCL library to reduce noise and data size, since LiDAR produces heavy, high-density data. This improves processing speed and makes segmentation and clustering more efficient. 
**Example: Voxel Grid Filtering**
Voxel Grid filtering reduces the number of points by dividing the space into small 3D cubes (voxels)  
and replacing all points inside each voxel with a single representative point (usually the centroid).  
This preserves the overall structure of the environment while **greatly reducing point cloud density** and computation cost.

### 3.1. Euclidean Clustering
Euclidean clustering groups points based on **physical distance**.
- Points close together are grouped into one cluster
- Points far apart form separate clusters
- Simple and intuitive — “points that are near each other likely belong
  to the same object”
This works well for separating obstacles within themselves eg. cars from pedestrians, poles, road signs, etc.

### 3.2. KD-Tree for Fast Search
A [KD-Tree (K-Dimensional Tree)](https://www.geeksforgeeks.org/dsa/search-and-insertion-in-k-dimensional-tree/) is a data structure used to quickly find
nearby points in a high-dimensional space.
- Instead of checking distance to every other point (slow, O(N²)),
- KD-Tree allows efficient neighbor lookups (fast, ~O(N log N))
- PCL uses KD-Tree to speed up Euclidean clustering
- Without a KD-Tree, clustering on large point clouds would be too slow
  for real-time driving

## 4. Bounding Box Fitting
Once segmentation and clustering are complete, **bounding boxes** are fit around each detected object cluster.
- A simple approach uses the cluster’s **minimum and maximum (x, y, z) coordinates** to form an axis-aligned bounding box (AABB).
- For objects that may not be aligned with the axes (like angled cars), **Principal Component Analysis (PCA)** can be used to find the dominant direction of the points and generate an **oriented bounding box (OBB)**.
Bounding boxes help perception and planning modules track object size, motion, and location in the environment.

## Final Output
![OUP2](https://github.com/vimalrajayyappan/SensorFusionCourse/blob/main/SFND_Lidar_Obstacle_Detection/LidarGif.gif)


