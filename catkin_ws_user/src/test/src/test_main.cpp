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

using namespace std;
using namespace cv;


int main()
{
	Mat img_1 = imread( "../../../../localization/captures/Lab_map_600x400_cropped.png", IMREAD_GRAYSCALE );
	Mat img_2 = imread( "../../../../localization/captures/undistorted_screenshot_25.05.2018.png", IMREAD_GRAYSCALE );

    namedWindow( "img_1", CV_WINDOW_AUTOSIZE );
    imshow( "img_1", img_1 );
    namedWindow( "img_2", CV_WINDOW_AUTOSIZE );
    imshow( "img_2", img_2 );

    Mat edge1, draw1;
    Canny( img_1, edge1, 50, 150, 3);

    edge1.convertTo(draw1, CV_8U);
    namedWindow("image1", CV_WINDOW_AUTOSIZE);
    imshow("image1", draw1);

    Mat edge2, draw2;
    Canny( img_2, edge2, 50, 150, 3);

    edge2.convertTo(draw2, CV_8U);
    namedWindow("image2", CV_WINDOW_AUTOSIZE);
    imshow("image2", draw2);

    waitKey(0);
    return 0;
}
