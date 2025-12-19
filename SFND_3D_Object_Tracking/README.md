# SFND 3D Object Tracking:

With the knowledge from previous project on Keypoint Detectors, Descirptors, Matching and knowing how Lidar detection works, we levelled up further
in this project by utilising those informations to perform 3D tracking on objects. This invloves `(1)detection of objects in camera frame using YOLO- Deep learning framework`, `(2)associating the objects between each frames based on keypoint matches and bounding box we have`, `(3)Compute Median Distance ratio on the Keypoint matches to find camera based
TTC(Time TO COllision)`, and `(4)Clustering Lidar Points using bounding box between image frames to compute Lidar Only TTC`. Overall a great learning and yes its a lengthy course :D.

## Running the Project
- Run
    ```
    ./3D_object_tracking
    ```
- The command window shows options for choosing Detectors. Choose options [0-6] and press enter
    ```
    [0:FAST, 1:BRISK, 2:ORB, 3:AKAZE, 4:SIFT, 5:HARRIS, 6:SHITOMASI]
    ```
- Next the command window shows options for choosing Descriptors. Choose options [0-5] and press enter
    ```
    [0:BRISK, 1:BRIEF, 2:ORB, 3:FREAK, 4:AKAZE, 5:SIFT]
    ```
- Next the command window shows options for choosing Matchers. Choose options [0-1] and press enter
    ```
    [0:MAT_BF, 1:MAT_FLANN]
    ```
- Next the command window shows options for choosing Selectors. Choose options [0-1] and press enter
    ```
    [0:SEL_NN, 1:SEL_KNN]
    ```

I used MAT_BF and SEL_NN Selector combinations.

Following shows how I achieved each rubric points.

## FP.1 Match 3D Objects:
`matchBoundingBoxes()` method in `camFusion_Student.cpp` is implemented to associate bounding boxes based on keypoint matches between the frames. The logic involves choosing
only corresponding ROI(region of interest) holds the filtered and matched keypoints. Then we iterate the bounding boxes for both previous and current frame, to store the
number of matched keypoints captured in each combination of bounding boxes. The combination with max-counts of keypoints are grouped and returned as associated bounding box
in the form of map.

## FP.2 Compute Lidar-based TTC

The clustering logic for Lidar points is already implemeneted which involves transforming the lidar points to camera frame and group them based on ROI(bounding box) in camera frame. Also there is a shrink factor, which shrinks the grouped lidar points so that the outliers along th eedges which are prone to errors are neglected.

Then the `computeTTCLidar()` function is implemented in `camFusion_Student.cpp`, where I just iterated through Lidar points in both the frames and found the corresponding mean
value both along X and Y. Computed the relative distance, which is then used to compute TTC. Using only closest point, will pick some outliers impacting TTC result.

## FP.3 Associate Keypoint Correspondences with Bounding Boxes
`clusterKptMatchesWithROI()` method in `camFusion_Student.cpp` is implemented. The logic follows, iterating all the keypoints in the bounding box and compute the euclidean distance with their corresponding matches in previous frame. Those keypoints are choosen, whose distance is approximately 1.2x closer to mean distance.These keypoints are then finalised for median distance ratio calculation for computing TTC.

## FP.4 Compute Camera-based TTC
`computeTTCCamera()` in `camFusion_Student.cpp` is implemented. Here the selected keypoints from FP.3 are taken and distance between one keypoint to everyother keypoints are computed for both current(disCurr) and previous frames(disPrev). Then the ratio (disCurr/disPrev) is calculated and finally their median is derived. This is called _Median Distance Ratio_ which is finally used  to calculate camera based TTC.


## FP.5 Performance Evaluation 1:

I have attached the TTC calculated by Lidar below. Even though Lidar is a very good sensor, the TTC calculation highly depends on choosing one best point in the point cloud.
I have seen deviations in the marked cells below. Though they are not too high, but its still need to be taken care. The main reason I found is accompaniying additional filter
algorithms to choose points that are effective would help reducing this error. Also the mean lidar point which we had computed is no guarntee cant be an outlier.
<img src="WriteupImages/TTC2.png" width="1000" height="1500" />

## FP.6 Performance Evaluation 2
Since we have no ground truth data to verify, taking LiDAR TTC as an initial approximation to validate against the values generated for image based TTCs.

I have run all the possible detector-descriptor combinations and the results are attached in the spreadsheet. As you can see Detector types _FAST, BRISK and SIFT_
there are TTC values which really not in the true range. Sometimes the TTCs are in the ranges of (40, 70 and even 90s). Even though _SIFT_ is comparitively better among the
mentioned 3 detectors, still it shows the values in 20s which is not recommended. 
<img src="WriteupImages/tableShot.PNG" width="1500" height="500" />


Comparing the rest, _ORB and HARRIS_ detectors, where the results are more erratic, as you can see the values in range of 500s for 10th image in _[ORB-SIFT]_ combination and
13th image of _[HARRIS+SIFT]_ combination.Some cases shows inf and negative values which are clearly due to poor selection of keypoints. Which is again not recommended.

Finally comparing _SHITOMASI and AKAZE_ - Gauging the values , I found SHITOMASI perfomed better as the change in values with respect to corresponding next frames are not 
that high and also considerably in the range compared to LiDAR and also on intutive understadning. Though the AKAZE perform secondly some values in the range of 17s are still
questionable. So I would choose *_[SHITOMASI + BRISK] or [SHITOMASI + ORB]_* combinations for TTC.


## Final Output - [SHITOMASI + BRISK] :
<img src="WriteupImages/AnimatedGIFS.gif" width="1200" height="300" />
