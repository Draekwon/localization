/*
 * test_main.cpp
 *
 *  Created on: May 14, 2018
 *      Author: sveb36
 */


#include <stdio.h>
#include <iostream>
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"


using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;


int main()
{
	Mat img_1 = imread( "../../../../localization/captures/Lab_map_600x400_cropped.png", IMREAD_GRAYSCALE );
	Mat img_2 = imread( "../../../../localization/captures/undistorted_screenshot_25.05.2018.png", IMREAD_GRAYSCALE );

//    namedWindow( "img_1", CV_WINDOW_AUTOSIZE );
//    imshow( "img_1", img_1 );
//    namedWindow( "img_2", CV_WINDOW_AUTOSIZE );
//    imshow( "img_2", img_2 );

    Mat edge1, draw1;
    Canny( img_1, edge1, 50, 150, 3);

    edge1.convertTo(draw1, CV_8U);
//    namedWindow("image1", CV_WINDOW_AUTOSIZE);
//    imshow("image1", draw1);

    Mat edge2, draw2;
    Canny( img_2, edge2, 50, 150, 3);

    edge2.convertTo(draw2, CV_8U);
//    namedWindow("image2", CV_WINDOW_AUTOSIZE);
//    imshow("image2", draw2);
//
//    waitKey(0);
//    return 0;

    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
      int minHessian = 400;
      Ptr<SURF> detector = SURF::create();
      detector->setHessianThreshold(minHessian);
      std::vector<KeyPoint> keypoints_1, keypoints_2;
      Mat descriptors_1, descriptors_2;
      detector->detectAndCompute( draw1, Mat(), keypoints_1, descriptors_1 );
      detector->detectAndCompute( draw2, Mat(), keypoints_2, descriptors_2 );
      //-- Step 2: Matching descriptor vectors using FLANN matcher
      FlannBasedMatcher matcher;
      std::vector< DMatch > matches;
      matcher.match( descriptors_1, descriptors_2, matches );
      double max_dist = 0; double min_dist = 100;
      //-- Quick calculation of max and min distances between keypoints
      for( int i = 0; i < descriptors_1.rows; i++ )
      { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
      }
      printf("-- Max dist : %f \n", max_dist );
      printf("-- Min dist : %f \n", min_dist );
      //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
      //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
      //-- small)
      //-- PS.- radiusMatch can also be used here.
      std::vector< DMatch > good_matches;
      for( int i = 0; i < descriptors_1.rows; i++ )
      { if( matches[i].distance <= max(2*min_dist, 0.02) )
        { good_matches.push_back( matches[i]); }
      }
      //-- Draw only "good" matches
      Mat img_matches;
      drawMatches( draw1, keypoints_1, draw2, keypoints_2,
                   good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                   vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
      //-- Show detected matches
      imshow( "Good Matches", img_matches );
      for( int i = 0; i < (int)good_matches.size(); i++ )
      { printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }
      waitKey(0);
      return 0;

}
