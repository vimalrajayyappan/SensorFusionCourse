#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType, double& timeToRun)
{
    // configure matcher
    bool crossCheck = true;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        
        int normType = cv::NORM_HAMMING;
        string normTypeName = "HAMMING";
        //FOR HOG DESCRIPTORS L2 NORM IS USED
        if (descriptorType == "DES_HOG")
        {
            normType = cv::NORM_L2;
            normTypeName = "L2 NORM";
        }

        matcher = cv::BFMatcher::create(normType, crossCheck);
        cout << "BF Based Matching based on "+ normTypeName << endl;
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F || descRef.type() != CV_32F)
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
      	cout << "FLANN Based Matching based on " << endl;
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)
		double t = (double)cv::getTickCount();
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        timeToRun = 1000 * t / 1.0 ;
        cout << " (NN) with n=" << matches.size() << " matches in " << timeToRun << " ms" << endl;
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
		vector<vector<cv::DMatch>> knn_matches;
        double t = (double)cv::getTickCount();
        cout << "Im in" << endl;
        matcher->knnMatch(descSource, descRef, knn_matches, 2); // finds the 2 best matches
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        timeToRun = 1000 * t / 1.0 ;
        cout << " (KNN) with n=" << knn_matches.size() << " matches in " << timeToRun << " ms" << endl;
        // filter matches using descriptor distance ratio test
        double minDescDistRatio = 0.8;
        for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
        {

            if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance)
            {
                matches.push_back((*it)[0]);
            }
        }
        cout << "# keypoints removed = " << knn_matches.size() - matches.size() << endl;
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType, double& timeToRun)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    if (descriptorType.compare("BRIEF") == 0)
    {
         extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
	//FREAK is also similar to BRIEF while BRIEF uses random selection of pairs around keypaints for description, while FREAK uses predefined pattern of selecting 256 pairs around Keypoint
    else if (descriptorType.compare("FREAK") == 0)
    {
        //
        bool rotationInvariant = true;
        bool scaleInvariant = true;
       //lower scale makes computation faster but less in description while larger value can capture more description at cost of computation
        float patternScale = 22.0f;
       //number of pyramid levels
        int nPyramidLevels = 4;
        const std::vector<int> selectedPairs = std::vector<int>();

        extractor = cv::xfeatures2d::FREAK::create(rotationInvariant, scaleInvariant, patternScale, nPyramidLevels, selectedPairs);
    }
    else if (descriptorType.compare("ORB") == 0)
    {
      
        cv::ORB::ScoreType scoreType = cv::ORB::FAST_SCORE; // Altered Value to FAST_SCORE from HARRIS_SCORE
        int nfeatures = 500;
        float scaleFactor = (1.200000048F);
        int nlevels = 8;
        int edgeThreshold = 31;
        int firstLevel = 0;
        int WTA_K = 2;
        int patchSize = 33; //Altered Patch size to 33 from 31
        int fastThreshold = 20;

        extractor = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);
    }
	
  //Similar to FREAK, but AKAZE can generate both Binary(MLDB) and FloatingPoint descriptors
  //Since AKAZE employs non-linear Scale Space Representation, its computationally more costly than simpler representation of FREAK/others.
    else if (descriptorType.compare("AKAZE") == 0)
    {
//       cout << "inside AKAZE" << endl;
        extractor = cv::AKAZE::create();
//       cout << "done AKAZE" << endl;
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        // Default Parameters
        int nfeatures = 0; //Number of features to detect
        int nOctaveLayers = 3; //controls the number of layers in each octave of scale space pyramid to be considered.it can influence sensitivity of algorithm to scale changes
        double contrastThreshold = 0.04; //Filters out low contrast keypoints
        double edgeThreshold = 10; //Filters out keypoints near image edges
        double sigma = 1.6; //initial scale of Gaussian blurring in pyramid
        extractor = cv::xfeatures2d::SiftDescriptorExtractor::create(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
    }

    // perform feature description
    double t = (double)cv::getTickCount();
//    cout << "O1" << endl;
    extractor->compute(img, keypoints, descriptors);
//    cout << "O2" << endl;
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    timeToRun = 1000 * t / 1.0 ;
    cout << descriptorType << " descriptor extraction in " << timeToRun << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, double& timeToRun, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    timeToRun = 1000 * t / 1.0;
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << timeToRun << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img,double &timeToRun, bool bVis)
{
    // cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); // convert to grayscale

    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)
	
    
    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    double timeCount = (double)cv::getTickCount();
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    // std::cout << dst << std::endl;
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    // std::cout << dst_norm << std::endl;
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);
    // std::cout << dst_norm_scaled << std::endl;
    // exit;
    // visualize results
    // string windowName = "Harris Corner Detector Response Matrix";
    // cv::namedWindow(windowName, 4);
    // cv::imshow(windowName, dst_norm_scaled);
    // cv::waitKey(0);

    // STUDENTS NEET TO ENTER THIS CODE (C3.2 Atom 4)

    // Look for prominent corners and instantiate keypoints
    // vector<cv::KeyPoint> keypoints;
    double maxOverlap = 0.0; // max. permissible overlap between two features in %, used during non-maxima suppression
    for (size_t j = 0; j < dst_norm.rows; j++)
    {
        for (size_t i = 0; i < dst_norm.cols; i++)
        {
            int response = (int)dst_norm.at<float>(j, i);
            if (response > minResponse)
            { // only store points above a threshold

                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i, j);
                newKeyPoint.size = 2 * apertureSize;
                newKeyPoint.response = response;

                // perform non-maximum suppression (NMS) in local neighbourhood around new key point
                bool bOverlap = false;
                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                    if (kptOverlap > maxOverlap)
                    {
                        bOverlap = true;
                        if (newKeyPoint.response > (*it).response)
                        {                      // if overlap is >t AND response is higher for new kpt
                            *it = newKeyPoint; // replace old key point with new one
                            break;             // quit loop over keypoints
                        }
                    }
                }
                if (!bOverlap)
                {                                     // only add new key point if no overlap has been found in previous NMS
                    keypoints.push_back(newKeyPoint); // store new keypoint in dynamic list
                }
            }
        } // eof loop over cols
    }     // eof loop over rows
    timeCount = ((double)cv::getTickCount() - timeCount) / cv::getTickFrequency();
  	timeToRun = 1000 * timeCount / 1.0;
    cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << timeToRun << " ms" << endl;


    if(bVis)
    {
    // visualize keypoints
        string windowName = "Harris Corner Detection Results";
        cv::namedWindow(windowName, 5);
        cv::Mat visImage = dst_norm_scaled.clone();
        cv::drawKeypoints(dst_norm_scaled, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imshow(windowName, visImage);
        cv::waitKey(0);
    }
    // EOF STUDENT CODE
}
    
void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType,double& timeToRun, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> detector;
    bool runDetector= true;
    if(detectorType.compare("FAST")==0)
    {
      int threshold = 30; //This parameter determines the minimum intensity difference between the center pixel and its neighboring pixels for a pixel to be considered a key point.
        bool bNMS = true;  //perform NMS
        cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16; //choose 9X16 pixels and ensure if atleast 9 out of 16 pixels are greater than threshold than that pixel is a potential keypoint
       detector = cv::FastFeatureDetector::create(threshold, bNMS, type);
    }
    
    else if(detectorType.compare("BRISK")==0)
    {
      //Default parameters
      detector = cv::BRISK::create(); 
    }
  	
  	else if(detectorType.compare("ORB")==0)
    {
//       int firstLevel - this parameter helps to start from which level of the default 8 levels of pyramid
//       int WTA_K - this parameter default value is 2, the number of random points to be considered for BRIEF descriptor
//       int scoreType - default value is cv::ORB::HARRIS_SCORE, but options include cv::ORB::FAST_SCORE
//       int patchSize - default value is 31, can be tuned
//       patchSize = 33, scoreType = cv::ORB::FAST_SCORE
     detector = cv::ORB::create(); 
    }
     
    //AKAZE is not discussed much in class, but its a modified version of KAZE detector, 
	// It also invloves non-linear scale-space selection using non-linear diffusion equation
	// It also adaptively selects appropriate scale and orientation for each keypoint
    else if (detectorType.compare("AKAZE") == 0)
    {
         detector = cv::AKAZE::create();                    
    }
    else if (detectorType.compare("SIFT") == 0)
    {
         detector = cv::xfeatures2d::SIFT::create();
    }
    else
    {
        cout << "Unknown Detector Type. Please change to type specified in code." << endl;
        runDetector = false;
        exit;
    }

    if(runDetector)
    {
      double t = (double)cv::getTickCount();
      detector->detect(img, keypoints);
      t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
      timeToRun = 1000 * t / 1.0;
      cout << detectorType << " Detector with n=" << keypoints.size() << " keypoints in " << timeToRun << " ms" << endl;
//       timeTakenVec.push_back(1000 * t / 1.0);
     
    }
  
  
}