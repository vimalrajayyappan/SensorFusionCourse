## SFND  Camera 2D Feature Tracking

SFND 2d Fetaure Tracking is the Mid-term Project in Udacity Sensor Fusion Nanodegree Course. The project deals with the following concepts 
discussed in the classroom.

- KEYPOINT DETCTORS
Keypoint Detectors are algorithms used in computer vision to identify specific, distinctive, points or locations within
an image.These points are called "Keypoints"/"Interest Points"/Features which helps us to locate significant positions in 
image that can be used as reference to describe the nature of image.

- DESCRIPTORS:
Descriptors is another algorithm that utilises the keypoints and generates a vector data or numerical representation that 
can best decribe the keypoints. This description is targeted to be unique to each keypoints, which helps in finding their 
correspondences in another image efficiently.

- MATCHING KEPOINTS:
Matching refers to utilising generated Descriptors of different images to identify similar keypoints and their 
correspondences between them.This helps in wide a lot of applications involving Object Tracking, Detection,
Image Stitching etc.

The main aim of this project is to find a best match of Detector-Descriptor combinations as there are N number of 
combinations of available. The evaluation of best combinatiomns are done on various aspects like Number of Keypoints Detected, 
Time Taken for Detection,and Desription and finally on matching accuracy of features across images.
The best combination  can be utilised for calculating TimeToCollision(TTC) of the current ego vehicle to the vehicle before, 
in the next project.

I have addressed below how I have met each rubric points.

## Running the Project
- Run
    ```
    ./2D_Feature_Tracking
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
- Next the command window shows choice for Visualizing output. Choose options [0-1] and press enter
    ```
    [0:No, 1:Yes]
    ```
- By Default the normType of descriptors that are based on Graidents are chose to be "DES_HOG" and rest are set to
"DES_BINARY".

This can be run for multiple times for different combintions, except,
    >1.AKAZE Descriptor works only for AKAZE Keypoints
    >2.SIFT Detector does now work with ORB Descriptor

- Press Ctrl+C to exit

## MP 1 - Data Buffer Optimization:
The reason for tis implemenation is that we should not stack up all the images in the buffer, as we must optimise the memory.
This has been done by fixing a buffer size limit and whenever new image comes in, the oldest image in the buffer 
is removed and the new images is pushed_back maintaining the buffer limit across executions.

## MP 2 - Keypoint Detectors:
There are totall 7 Detectors [0:FAST, 1:BRISK, 2:ORB, 3:AKAZE, 4:SIFT, 5:HARRIS, 6:SHITOMASI] implemeneted. While the SHITOMASI
is already present and HARRIS is reused from classroom. The rest of the detector are implemented in `detKeypointsModern`, which 
utilizes OpenCV libraries for the same. The selection comes from user input while running the project

## MP 3 - Keypoint Removal:
Here the preceeding vehicle rectangle is defined using `cv::Rect`. Then the keypoints are iterated to find whether keypoints
falls within the rectangle using `vehicleRect.contains(keypoint.pt)`. If found only those keypoints are stored.

## MP 4 - Keypoint Descriptors
In this section, the relevant descriptors are implemented in `descKeypoints` function utilising keypoints generated from MP 2&3. `BRISK` descriptor was already
there, while the rest of decriptors [ 1:BRIEF, 2:ORB, 3:FREAK, 4:AKAZE, 5:SIFT] are added using OpenCV library.

## MP 5 - Descriptor Matching
`matchDescriptors` function deals with matching descriptors of first image to the descriptors of next image.
The matched keypoints and correspondences are shown visually. Two matcher functions are used, Brute Force and FLANN based matching.
For each matchers the normtype has to be defined based on descriptors, which I handled automatically based on descriptor
selection.'HAMMING" for Binary and 'L2_NORM' for HOG based descripors.

## MP 6 - Descriptor Distance Ratio
Descriptor Distance Ratio is very efficient way of reducing false positives. The method has been proposed by D Lowe in 1999
paper on SIFT. The idea is the best two match of keypoint in the source image is found in the reference image. With those two matches
ratio of descriptor distance is computed and the threshold is set at 0.8. If its below the threshold then the first best match
is stored , if its greater than the threshold, these two matches can be a potential false positives and can be skipped. Same is 
implemented under KNN selector here.

## MP 7.1 - 
The objective here is to count the number of keypoints for all 10 images, and to describe the distribution of points for each detector. 
The follwoing is the graph, which shows number of points detected by each detector.
<Graph>
<Data>

For neighbor_hood , I just averaged the [Keypoint size/Number of keypoints] for all images and the values are at th end.
Following represents the graph of time taken by each detector.
<time Graph>


With these things in hand, I have the following observations:

- ##### FAST
    Good number of keypoints defining vehicle edge [+0.8] Indicators are detected goofd[+1] . Rear window has quite a good numbers [+1] .
    Number plate is detected and one or two keypoints are identified. [+0.3]. Avg Running time - [0.6ms] which is impressive[+1] . 
    _My overall rating [4.1/5]_
    <Fast image>
- ##### BRISK
    The vehicle edges are very well detected at different scale levels[+1] 
    and good numbers of keypoints in the rear window too[+1]. 
    Indicators are also very well detected [+1]  at small scales. Trying to 
    capture most keypoints. Number plates has ok keypoints [0.5]. Avg 
    Running Time 33ms. [+0.1] -
    _My rating 3.6/5_


- ##### ORB
    The vehicle edges are being detected seems there are some spillovers to road [+0.7]. indicators has most number of keypoints in scales [+1] . Rear window also has a considerable keypoints [+0.7]. 
    No keypoints on Number plate.Avg Running time 7ms - [+0.8] . 
    _My Rating : 3.2/5_

- ##### AKAZE
    The vehicle edges are detected good [+1] , average key points on 
    rear window [+0.7], No Keypoints on Number plate . Indicators are 
    detected [+1]. Average Running Time : 201ms which way too high . 
    _My rating : [2.7/5]_


- ##### SIFT
    The vehicle edges are detected but some keypoints spilled on to 
    road(+0.5) and considerable number of key points on rear window  
    too(+0.6). Indicators are also detected  at small scales(+1). Number 
    plate has less keypoints for this setting (+0.4) . Average Running 
    Time : 139ms which is high. 
    _My rating 2.5/5_

- ##### HARRIS
    Seems very less number of keypoints on edges [+0.2] and indicators are detected [+1]. Rear window has meagre keypoints [+0.2]. 
    Average Running Time (15 ms) - [+0.6] -
    _My rating 2/5_

- ##### SHITOMASI
    Number plate has more number of keypoints (+1) compared to  
    vehicle eges[+0.4], the rear window has less number of points. Indicator 
    lights are also detected(+1). Overall Average Detector for the 
    current setting. Avg Running time (11ms) - (0.6) . 
    _My rating - [3.0/5]_

So FAST scored good in terms of computation speed and keypoint detection, distribution followed by BRISK, ORB and SHITOMASI.

## MP 7.2 
The obejctive here is to count the number of all the keypoints with all detector-descriptor combinations for all 10 images.The BF approach is used for matching and also descriptor distance ratio in KNN selector is set to 0.8.
Following are the table and graph which also shows the % of matched keypoints according to each detector-descriptor combinations.
<Table>
<Graph>

As you can see, the marked cells in the sheet shows maximum percentage, simultaneously on less execution time.Though SHITOMASI's matching percentage are higher,  the detector time(11ms) is high compared to FAST(0.6ms).


## MP 7.3
With the graph of time of execution, and detection capabilities of all detector-descriptor combinations, it states clearly the winner is one that works with combination of FAST as it performs better in terms of computation time and overall picking very good features.

On that way I will choose `FAST + BRIEF` and `FAST + ORB` as first two. On the third place 
I was pondering, and I valued matches over time hence I choose `ORB+BRISK`. 

Needless to mention that `SHITOMASI` also works good with BRIEF and ORB, capturing high number of matches. But execution time takes a huge hit but comparatively better than rest of other combinations.



 








