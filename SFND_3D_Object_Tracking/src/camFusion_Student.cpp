
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
  //////////////////////////////////////////////
  // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0;
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 4);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
  //////////////////////////////////////////////
    
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
 
    std::vector<cv::DMatch> matchPointsInROI;
    std::vector<double> distBetweenKeypoints;

    // Accumulate the keypoints that are inside specific ROI
    for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it)
    {
        cv::KeyPoint prevKeyPt = kptsPrev[it->queryIdx];
        cv::KeyPoint currKeyPt = kptsCurr[it->trainIdx];

        if (boundingBox.roi.contains(currKeyPt.pt))
        {
            matchPointsInROI.push_back(*it);
            distBetweenKeypoints.push_back(cv::norm(currKeyPt.pt - prevKeyPt.pt));
        }
    }

    // Find the mean distance of all keypoints and filter the points based on threshold value 
    double thresholdVal = 1.2;//1.2;//1////0.7//0.9;
    double avgDist = std::accumulate(distBetweenKeypoints.begin(), distBetweenKeypoints.end(), 0.0) / distBetweenKeypoints.size();
    for (int idx = 0; idx < distBetweenKeypoints.size(); ++idx)
    {
        if (distBetweenKeypoints[idx] < (avgDist*thresholdVal))
        {
            int trainIndex = matchPointsInROI[idx].trainIdx;
            boundingBox.keypoints.push_back(kptsCurr[trainIndex]);
            boundingBox.kptMatches.push_back(matchPointsInROI[idx]);
        }
    }

    std::cout << "Total" << boundingBox.kptMatches.size() <<" selected out of " << kptMatches.size() << "Keypoints"<< std::endl;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
    // compute distance ratios between all matched keypoints
    std::cout << "In Compute TTC Camera : "<< kptMatches.size() << std::endl;
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                // std::cout << distRatio << std::endl;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts
    std::cout << "DIST RATIO SIZE : " << distRatios.size() << std::endl;
    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }


   
    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence
    std::cout << medDistRatio << " medDistRatio"<< std::endl;
    double dT = 1 / frameRate;
    TTC = -dT / (1 - medDistRatio);
    std::cout << TTC << " TTC TIME"<< std::endl;
}



//IQR Filetering:
std::vector<double> IQR_Filter(const std::vector<double>& xVals)
{
    std::vector<double> filtered;
    //General fomula of IQR:
    //Q1 - 25th percentile
    //Q2- 50th percentile
    //Q3 - 75th percentile
    //IQR = Q3-Q1
    //lowerBound = Q1 - 1.5*IQR
    //higherBound = Q3 + 1.5*IQR
    int n = xVals.size();
    double q1 = xVals[n / 4];
    double q3 = xVals[(3 * n) / 4];
    double iqr = q3 - q1;

    double lower = q1 - 1.5 * iqr;
    double upper = q3 + 1.5 * iqr;

    for (double x : xVals)
    {
        if (x >= lower && x <= upper)
            filtered.push_back(x);
    }
    return filtered;
}
//IQR-based filtering means removing outliers using the Interquartile Range (IQR), 
//a robust statistical method that is much less sensitive to noise and extreme values than mean-based methods.
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                              std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    double dT = 1.0 / frameRate;   // time between frames
    double laneWidth = 4.0;        // ego lane width

    std::vector<double> prevX, currX;


    //Filter Lidar points based on lanewidth 
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        if (abs(it->y) <= (laneWidth/2.0))
        {
            prevX.push_back(it->x);
        }
    }

    //Filter Lidar points based on lanewidth 
    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        if (abs(it->y) <= (laneWidth/2.0))
        {
             currX.push_back(it->x);
        }
    }

    if(prevX.size()>0 && currX.size()>0)
    {
        //Sort the X values
        std::sort(prevX.begin(), prevX.end());
        std::sort(currX.begin(), currX.end());

        // Applly IQR filter
        std::vector<double> prevFiltered = IQR_Filter(prevX);
        std::vector<double> currFiltered = IQR_Filter(currX);

        if(prevFiltered.empty() || currFiltered.empty())
        {
            TTC = NAN;
            return;
        }

        //Find Median
        double medPrev = prevFiltered[prevFiltered.size()/2];
        double medCurr = currFiltered[currFiltered.size()/2];

        if(medPrev <= medCurr)
        {
            TTC = NAN;
            return;
        }

        TTC = (medCurr*dT)/(medPrev-medCurr);
        std::cout << "PrevDistance : " << medPrev << std::endl;
        std::cout << "CurrDistance : " << medCurr << std::endl;
        std::cout << "dT " << dT << std::endl;


    }
    else //Not needed as in all frames we have detections, jus safety check if we know the relative speed, simply use distance/rel_speed
    {
        TTC = NAN;
    }
    std::cout <<"TTC Lidar IQR based done : "<< TTC << "s" <<std::endl;
    

}

//This is previos approach - can be neglected
void computeTTCLidarMeanBased(std::vector<LidarPoint> &lidarPointsPrev,
                              std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // ...
    double dT = 1.0 / frameRate;    // delta time between two frames in seconds
    double laneWidth = 4.0;         // assumed width of the ego lane    
    std::vector<LidarPoint> prevLaneFilteredLidarpoint;
    std::vector<LidarPoint> currLaneFilteredLidarpoint;

    //Filter Lidar points based on lanewidth 
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        if (abs(it->y) <= (laneWidth/2.0))
        {
            prevLaneFilteredLidarpoint.push_back(*it);
        }
    }

    //Filter Lidar points based on lanewidth 
    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        if (abs(it->y) <= (laneWidth/2.0))
        {
            currLaneFilteredLidarpoint.push_back(*it);
        }
    }

    //Get the centroid of  Lidar Point cluster 
    double meanXPrev, meanYPrev,meanXCurr, meanYCurr = 0;
    if(prevLaneFilteredLidarpoint.size()>0 && currLaneFilteredLidarpoint.size()>0)
    {
        double xSum,ySum = 0;
        for (auto it = prevLaneFilteredLidarpoint.begin(); it != prevLaneFilteredLidarpoint.end(); ++it)
        {
            xSum += it->x;
            ySum += it->y;
        }
        meanXPrev = xSum/prevLaneFilteredLidarpoint.size();
        meanYPrev = ySum/prevLaneFilteredLidarpoint.size();

        xSum = 0;
        ySum = 0;
        for (auto it = currLaneFilteredLidarpoint.begin(); it != currLaneFilteredLidarpoint.end(); ++it)
        {
            xSum += it->x;
            ySum += it->y;
        }

        meanXCurr = xSum/currLaneFilteredLidarpoint.size();
        meanYCurr = ySum/currLaneFilteredLidarpoint.size();

        double disPrev = sqrt(pow(meanXPrev, 2) + pow(meanYPrev, 2));
        double disCurr = sqrt(pow(meanXCurr, 2) + pow(meanYCurr, 2));
        //Compute TTC
        TTC = disCurr * dT / (disPrev - disCurr);
    }
    else //Not needed as in all frames we have detections, jus safety check if we know the relative speed, simply use distance/rel_speed
    {
        TTC = NAN;
    }

}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    //Get the number of bounding boxes in each frame and create a 2D array of possible correspondences.
    int prevBBCounts = prevFrame.boundingBoxes.size(); //p
    int currBBCounts = currFrame.boundingBoxes.size(); //c
    int numOfKeyPointsInEachPair[prevBBCounts][currBBCounts] = {}; //pt_counts

    //Itearte through each matching points
    for (auto it = matches.begin(); it != matches.end() - 1; ++it)
    {
        int prevKptIdx = it->queryIdx;
        int currKptIdx = it->trainIdx;

        cv::KeyPoint prevKpt = prevFrame.keypoints[prevKptIdx];
        cv::KeyPoint currKpt = currFrame.keypoints[currKptIdx];


        std::vector<int> prevBoxIndxs, currBoxIndxs;

        for (int i = 0; i < prevBBCounts; i++)
        {
            if (prevFrame.boundingBoxes[i].roi.contains(prevKpt.pt))
            {
                prevBoxIndxs.push_back(i);
            }
        }
        for (int i = 0; i < currBBCounts; i++)
        {
            if (currFrame.boundingBoxes[i].roi.contains(currKpt.pt))
            {
                currBoxIndxs.push_back(i);
            }
        }
        
        for (int idxPrev : prevBoxIndxs)
            for (int idxCurr : currBoxIndxs)
                numOfKeyPointsInEachPair[idxPrev][idxCurr] += 1;

    }

    for (int i = 0; i < prevBBCounts; i++)
    {
        int maxCount = 0;
        int maxIndx = 0;
        for (int j = 0; j < currBBCounts; j++)
            if (numOfKeyPointsInEachPair[i][j] > maxCount)
            {
                maxCount = numOfKeyPointsInEachPair[i][j];
                maxIndx = j;   
            }
        bbBestMatches[i] = maxIndx;
    }
    
}
//done