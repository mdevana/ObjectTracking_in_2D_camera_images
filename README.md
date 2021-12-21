### 2D Feature Tracking in Camera Images for Sensor Fusion

<img src="images/keypoints.png" width="820" height="248" />

## Goal
The goal of this project is to build a collision detection system.  I have built the feature tracking part and test various detector / descriptor combinations to see which ones perform best. 

## Introduction


## Data Buffer Optimization
Since computer vision algorithms will be deployed in mobile hardware with limited resources, optimizing  the amount of data held in memory is of significant importance. For this purpose, i have implemented a data buffer modeled based on Queue data structure. Here the first image to enter will also leave first to accommodate next image and maintain a constant queue size, in this case 2.

```
        DataFrame frame;
        frame.cameraImg = imgGray;
        frame.imgName = imgFullFilename;
        if (dataBuffer.size() >= dataBufferSize){
            dataBuffer.erase(dataBuffer.begin());
        }
        dataBuffer.push_back(frame);
```
## Keypoint Detection Algorithm Selection
The computer vision library provides various algorithms to detect keypoint in images. I have selected and tested following algorithms. Harris, Shi-Tomasi, FAST, ORB, AKAZE and SIFT. 

I have developed dual mode to run my program. If the variable is_single_run is set to true, then values set to the string det_type and des_type will be considered. If the value of is_single_run is false, then all combination of detection and descriptor types will used and performance analysis will be conducted. Please set the variable to true, if you want to use your own combination
```
    bool is_single_run = true;
    string det_type = "FAST"; // Detector Type
    string des_type = "ORB";// Descriptor Type
```

The string det_type and des_type are checked if their values matches anyone of the algorithms is implemented and the corresponding call is made.

```
 if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, false,ctime_detection);
        }
        else if(detectorType.compare("HARRIS") == 0)
        {
            detKeypointsHarris(keypoints, imgGray, false,ctime_detection);
        }
        else if( (detectorType.compare("FAST") == 0) || (detectorType.compare("BRISK") == 0) || (detectorType.compare("ORB") == 0) || (detectorType.compare("AKAZE") == 0) || (detectorType.compare("SIFT") == 0) )
        {
            detKeypointsModern(keypoints, imgGray,detectorType, false,ctime_detection);
        }
```
While Shi-Tomasi and Harris have their own function calls, other algortihms are clustered into a single function call detKeypointsModern(). The function is implemented in matching2D_Student.cpp. Inside this function, corresponding call to respective algorithms are made as shown below. 

```
 if (detectorType.compare("FAST") == 0){

        t = (double)cv::getTickCount();
        cv::Ptr<cv::FastFeatureDetector> fast_detect = cv::FastFeatureDetector::create(30,true);
        fast_detect->detect(img,keypoints);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "FAST detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    }

    else if (detectorType.compare("BRISK") == 0){

        t = (double)cv::getTickCount();
        cv::Ptr<cv::FeatureDetector> detector = cv::BRISK::create();
        detector->detect(img,keypoints);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "BRISK detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    }

    else if (detectorType.compare("ORB") == 0){

        t = (double)cv::getTickCount();
        cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
        detector->detect(img,keypoints);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "ORB detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    }

    else if (detectorType.compare("AKAZE") == 0){

        t = (double)cv::getTickCount();
        cv::Ptr<cv::FeatureDetector> detector = cv::AKAZE::create();
        detector->detect(img,keypoints);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "AKAZE detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    }
    else if (detectorType.compare("SURF") == 0){


        int minHessian=400;
        t = (double)cv::getTickCount();
        cv::Ptr<cv::FeatureDetector> detector = cv::xfeatures2d::SURF::create(minHessian);
        detector->detect(img,keypoints);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "SURF detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    }
    else if (detectorType.compare("SIFT") == 0){

        t = (double)cv::getTickCount();
        cv::Ptr<cv::FeatureDetector> detector = cv::xfeatures2d::SIFT::create();
        detector->detect(img,keypoints);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "SIFT detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    }

```

## Keypoint Removal
Since project scope is restricted to detecting the vehicle at the front, the keypoints detected on front vehicles are alone considered for further processing. The variable bFocusOnVehicle should be set to true for keypoint restriction on Front vehicle. The bounding box for front vehicle is provided by the cv::Rect(). By looping through all all the detected keypoints and adding only the ones which fall into the box into a fresh vector, the keypoints are seperated. The inbuilt function of cv::Rect contains() can also be used to check if the keypoints fall into bounding box. Here , i have checked it manually.
```
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            vector<cv::KeyPoint> filteredPts;
            float a = vehicleRect.x;
            for (cv::KeyPoint kp:keypoints){
                if ( ( (kp.pt.x > vehicleRect.x ) && (kp.pt.x < ( vehicleRect.x + vehicleRect.width ) ) ) && ( (kp.pt.y > vehicleRect.y ) && (kp.pt.y < ( vehicleRect.y + vehicleRect.height ) ) ) ){
                    filteredPts.push_back(kp);
                }
                
                
            }
            keypoints = filteredPts;
            cout << " NOTE: Keypoints Restricted to box of preceding vehicle!" << keypoints.size()<<endl;
        }
```

## Keypoint Descriptors





## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.
