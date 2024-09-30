/* INCLUDES FOR THIS PROJECT */
#include <iostream>
// #include <conio>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

void DisplayVector(vector<double>& inp, std::string& out)
{
  out = "";
  std::string comma = ",";
  for (double num:inp)
  {
    std::string addThis = std::to_string(num);
    out = out + addThis;
    out = out + comma;
  }
  	
}

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
  while (true) 
  {
        bool valid = false;
        string detSelected,desSelected,matSelected,selSelected = "";
        string desType = "DES_BINARY";
        double visOption = 0;

        try {

            //User Options for Detector
            string input ="";
            std::cout << "Enter input (Ctrl+C to quit): " << endl;
            std::cout << "Type the index number of the Detector below to be tested: " << endl;
            std::vector<std::string> detectorStrings = {"FAST", "BRISK", "ORB", "AKAZE", "SIFT", "HARRIS", "SHITOMASI"};
            for(int i =0; i<detectorStrings.size() ; i++)
            {
              cout << i << " : "  << detectorStrings.at(i) <<endl;
            }
            std::getline(std::cin, input);
            detSelected = detectorStrings.at(std::stod(input));
            
            std::cout << "Selected Detector : " + detSelected << endl;
            cout << "" <<endl;

            //Options for Descriptor
            std::cout << "Type the index number of the Descriptor below to be tested: " << endl;
            std::vector<std::string> descriptorStrings = {"BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};
            for(int i =0; i<descriptorStrings.size() ; i++)
            {
              cout << i << " : "  << descriptorStrings.at(i) <<endl;
            }
            std::getline(std::cin, input);
            desSelected = descriptorStrings.at(std::stod(input));
                    
            if(desSelected == "SIFT")
            {
              desType = "DES_HOG";
            }
            std::cout << "Selected Descriptor : " + desSelected << endl;

            //User options for MAtcher
            std::cout << "Type the index number of the Matcher below to be tested: " << endl;
            std::vector<std::string> matcherStrings = {"MAT_BF", "MAT_FLANN"};
            for(int i =0; i<matcherStrings.size() ; i++)
            {
              cout << i << " : "  << matcherStrings.at(i) <<endl;
            }
            std::getline(std::cin, input);
            matSelected = matcherStrings.at(std::stod(input));

            std::cout << "Selected Matcher : " + matSelected << endl;
            
            //User Options for Selector
            std::cout << "Type the index number of the Selector for Matcher below to be tested: " << endl;
            std::vector<std::string> selectorStrings = {"SEL_NN", "SEL_KNN"};
            for(int i =0; i<selectorStrings.size() ; i++)
            {
              cout << i << " : "  << selectorStrings.at(i) <<endl;
            }
            std::getline(std::cin, input);
            selSelected = selectorStrings.at(std::stod(input));

            std::cout << "Selected Selector : " + selSelected << endl;

            //User options for visualization
            std::cout << "Should I Visualize the results? Choose Index number " << endl;
            std::vector<std::string> visStrings = {"NO", "YES"};
            for(int i =0; i<visStrings.size() ; i++)
            {
              cout << i << " : "  << visStrings.at(i) <<endl;
            }
            std::getline(std::cin, input);
            visOption = std::stod(input);

            valid = true;

        } catch (const std::ios::failure& e) {
            // Handle Ctrl+C or other input errors
            std::cerr << "Error: " << e.what() << std::endl;
            break;
        }
    
if(valid)
{
    /* INIT VARIABLES AND DATA STRUCTURES */
    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = visOption;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */
    double avgExecutionTime = 0;
    double totalTime = 0;
    std::vector<double> lstTimeTakenDET, lstTmTknDES,lsttmTknMatches, lstKeyPointsCount,lstNoOfMatches ;
    int imgNos =  imgEndIndex - imgStartIndex;
    double ngbhood_size = 0;
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        double timeTaken = 0;
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        //Checking whether the size limit has been reached in the ring, if so erasing the first
        //element
        if(dataBuffer.size() == dataBufferSize)
        {
            dataBuffer.erase(dataBuffer.begin());
        }
        dataBuffer.push_back(frame);

        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done " << "Current DataBuffer size = " << dataBuffer.size() << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        string detectorType = detSelected; //"AKAZE"; //SIFT //AKAZE //BRISK //SHITOMASI //HARRIS //FAST //ORB

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
		    std::vector<std::string> detectorStrings = {"FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray,timeTaken, false);
        }
        else if(detectorType.compare("HARRIS") == 0)
        {
            detKeypointsHarris(keypoints, imgGray,timeTaken, false);
        }
      	else if(std::count(detectorStrings.begin(), detectorStrings.end(),detectorType ))
        {
          detKeypointsModern(keypoints, imgGray, detectorType,timeTaken, false);
        }
        else
        {
          cout << "Unknown Detector Type : Please change to specific w.r.t code" << endl;
          exit;
        }
        lstTimeTakenDET.push_back(timeTaken);
        totalTime += timeTaken;
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
         
            vector<cv::KeyPoint> keypointFiltered;
            double kptSizeSum = 0;
            for (auto &keypoint : keypoints)
            {
                if (vehicleRect.contains(keypoint.pt))
                {
                    keypointFiltered.push_back(keypoint);
                    kptSizeSum += keypoint.size;

                }
            }
            
            ngbhood_size += (kptSizeSum/keypointFiltered.size());


            keypoints = keypointFiltered;
            std::cout << "Filtered Keypoints Size : " + std::to_string(keypoints.size())<< std::endl;
            lstKeyPointsCount.push_back(keypoints.size());
            if (imgIndex ==0 && false)
            {
              cv::Mat visImage = imgGray.clone();
              cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
              string windowName = detSelected + " Results";
              cv::namedWindow(windowName, 6);
              imshow(windowName, visImage);
              cv::waitKey(0);
            }
        
        }

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
      
        (dataBuffer.end() - 1)->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
       //NOTE : AKAZE Descriptor works only with AKAZE DETECTOR
        string descriptorType = desSelected; //"AKAZE"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
        string displayDes = descriptorType; //Saving this as this value is being altered again below. I can use this for window name
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType,timeTaken);
        //// EOF STUDENT ASSIGNMENT
        lstTmTknDES.push_back(timeTaken);
        totalTime += timeTaken;

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;
        
        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = matSelected;//"MAT_BF";        // MAT_BF, MAT_FLANN
            string descriptorType = desType; // DES_BINARY, DES_HOG
            string selectorType = selSelected;       // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType, matcherType, selectorType,timeTaken);
            
          lsttmTknMatches.push_back(timeTaken);
          totalTime += timeTaken;
            //// EOF STUDENT ASSIGNMENT
			
            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;
            lstNoOfMatches.push_back(matches.size());
            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
//             bVis = true;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "DET: "+ detectorType + " - " + "DES :" + displayDes +" - " + matcherType +" - "+selectorType ;
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
//             bVis = false;
        }

    } // eof loop over all images
	
  	avgExecutionTime = totalTime/(imgEndIndex-imgStartIndex + 1);
    std::string emptyString = "";
    std::cout << "DETECTOR : "+ detSelected <<endl;
    std::cout << "DESCRIPTOR : "+ desSelected << endl;
    DisplayVector(lstKeyPointsCount,emptyString);
    std::cout << "Keypoints Count of each image : " + emptyString << endl;
    DisplayVector(lstTimeTakenDET,emptyString);
    std::cout << "Time for DETECTION of each image : " + emptyString << endl;
    
    DisplayVector(lstTmTknDES,emptyString);
    std::cout << "Time for Description of each Pairs : " + emptyString << endl;
    DisplayVector(lstNoOfMatches,emptyString);
    std::cout << "Number of Matched KP of each Pairs : " + emptyString << endl;
    // DisplayVector(lsttmTknMatches,emptyString);
    // std::cout << "Time for Matching of each Pairs : " + emptyString << endl;
    cout << "Average Execution time for Detetction,Decription and Matching of selected option " + detSelected + "," + desSelected + "," + matSelected + "," + selSelected + "," + "is " + std::to_string(avgExecutionTime) + "ms"<< endl;
    cout << "Average NieghborHood Size = " << ngbhood_size/imgNos << std::endl;
//     return 0;
  }
  std::cout << "#################################################" << endl;
}
}
