/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
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
    bool bVis = false;            // visualize results
    bool if_save_image = false;

    // Variables for single run or anaylse performance characteristics
    bool is_single_run = false;
    string det_type = "FAST"; // Detector Type
    string des_type = "ORB";// Descriptor Type

    /* MAIN LOOP OVER ALL IMAGES */
    /*************************Code for setting combinations ********************************/
    vector<string> detect_types = {"HARRIS","SHITOMASI","FAST","BRISK","ORB","AKAZE","SIFT"};
    vector<string> descriptor_types ={"BRISK","ORB","FREAK","AKAZE","SIFT"};
    vector<string> descriptor_class ={"DES_BINARY","DES_HOG"};
    
    int img_cnt = imgEndIndex - imgStartIndex + 1;
    int n_kps_mat[7][9][10];
    int n_kps_matches[7][9][9];
    float ctime_det_ext[7][9][10];

    for(int i=0;i<detect_types.size();i++){
        for(int j=0;j<descriptor_types.size();j++){

        string detectorType; 
        string descriptorType;

        if (is_single_run){
            detectorType = det_type;    
            descriptorType = des_type; 

        }
        else{
            detectorType = detect_types[i];    
            descriptorType = descriptor_types[j]; 

        }
           

        cout << "#1 : Using Detector : "<<detectorType<<" Descriptor :"<<descriptor_types[j];
        dataBuffer.clear();

        vector<int> n_kpts_img;

        if ( ( descriptorType=="AKAZE" && detectorType!="AKAZE" ) || (descriptorType=="ORB" && detectorType=="SIFT") ){
            cout << "skipping"<<endl;
            //n_kps_mat[i][j]=-1;
            for(int k = 0;k<img_cnt;k++){
                n_kps_mat[i][j][k]=-1;
                n_kps_matches[i][j][k]=-1;
                ctime_det_ext[i][j][k]=-1.0;
            }
            
            continue;
        }
        cout << "processing"<<endl;

        /**************************End Code for setting combinations ********************************/

        

        for (size_t imgIndex = 0; imgIndex <= (imgEndIndex - imgStartIndex) ; imgIndex++)
        {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
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
        frame.imgName = imgFullFilename;
        if (dataBuffer.size() >= dataBufferSize){
            dataBuffer.erase(dataBuffer.begin());
        }
        dataBuffer.push_back(frame);

        cout << "#1 : Size of Ring buffer"<<dataBuffer.size()<< endl;
        for(DataFrame df:dataBuffer)
            cout << "#1 : File in Ring buffer"<<df.imgName<< endl;
        
        
        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        //string detectorType = "SHITOMASI";
         // Here setting detection type from loop

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        float ctime_detection;
        float ctime_desextract;

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

        else
        {
            cout << "#2 :  Unknown detector . Not yet implemented" << endl;
        }
        
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
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

        n_kps_mat[i][j][imgIndex] = (keypoints.size());

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        //string descriptorType = "BRISK"; // BRIEF, ORB, FREAK, AKAZE, SIFT
        
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType,ctime_desextract);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;
        
        ctime_det_ext[i][j][imgIndex] = ctime_desextract + ctime_detection;

        /**************************start Code for collecting performance metrics and setting types********************************************************/
        string imgWritePath_keypts = dataPath + "output_images/" + "image_kpts" + std::to_string(imgIndex) + "_" + detectorType + "_" + descriptorType + ".png";
        string imgWritePath_keymatch = dataPath + "output_images/" + "image_keymatch" + std::to_string(imgIndex-1)+"_"+ std::to_string(imgIndex) + "_" + detectorType + "_" + descriptorType + ".png";
        cv::Mat Image_for_KP = (dataBuffer.end() - 1)->cameraImg;
        cv::Mat Image_for_KP_clone = Image_for_KP.clone();
        cv::drawKeypoints(Image_for_KP,keypoints,Image_for_KP_clone,cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        if (if_save_image)
            cv::imwrite(imgWritePath_keypts,Image_for_KP_clone);
        
        int desIndex = 0; // index for selecting binary / HOG descriptor class
        if (descriptorType == "SIFT")
            desIndex = 1;

        /************************** End Code for collecting performance metrics and setting types ********************************************************/


        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */
            

            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            
            string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN
            string descriptorType = descriptor_class[desIndex]; // DES_BINARY, DES_HOG
            
            cout << "#descriptor class :" << descriptorType<<endl;

            

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            n_kps_matches[i][j][imgIndex-1] = matches.size();
            

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;
            

            // visualize matches between current and previous image
            bVis = true;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                //cv::namedWindow(windowName, 7);
                //cv::imshow(windowName, matchImg);
                
                string fname = "";
                
                if (if_save_image)
                    bool check = cv::imwrite(imgWritePath_keymatch,matchImg);
                
                //cout << "Press key to continue to next image" << endl;
                //cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }
        

    } // eof loop over all images
    if (is_single_run)
        break;

    }// eof detectors

    if (is_single_run)
        break;
}// eof descriptors

/*********************************start code to collect performance metrics***********************************************/
bool is_show_keyPts_stats = true;
bool is_show_comptime_stats = true;
bool is_show_keymatch_stats = true;

if (is_show_keyPts_stats && (!is_single_run)){
    for(int i=0;i<detect_types.size();i++){
                cout <<detect_types[i]<<":";
                cout <<"( ";
                for (int k = 0;k<img_cnt;k++)
                    cout <<n_kps_mat[i][0][k]<<",";
                cout <<" )"<<endl;
            
    }
}
// Print Performance Matrics - Computation time
if (is_show_comptime_stats && (!is_single_run)){

    
        
    for(int i=0;i<detect_types.size();i++){
        for(int j=0;j<descriptor_types.size();j++){
            cout<<"( ";
            for (int k = 0;k<img_cnt;k++)
                cout <<ctime_det_ext[i][j][k]<<",";
            cout<<") "; 
        }
        cout << endl;
    }

    // Print average compute Time across all images

    for(int i=0;i<detect_types.size();i++){
            
            for(int j=0;j<descriptor_types.size();j++){
                double sum=0.0;
                for (int k = 0;k<img_cnt;k++){
                    if (ctime_det_ext[i][j][k] > 0)
                    sum+=ctime_det_ext[i][j][k];
                    }
                cout<<(double)sum/img_cnt<<"  ";
            }
            cout <<endl;
    }

}
    
if (is_show_keymatch_stats && (!is_single_run)){

    for (int k = 0;k<img_cnt-1;k++){ 
        cout<<" Keypoints matched between images "<<(k+1)<<" and "<<(k+2)<<endl;
        for(int i=0;i<detect_types.size();i++){
            
            for(int j=0;j<descriptor_types.size();j++)
                cout <<n_kps_matches[i][j][k]<<"  ";
            cout<<endl;
        }
    }
}

/*********************************End code to collect performance metrics***********************************************/


    return 0;
}
