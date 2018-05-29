/*
 * test_main.cpp
 *
 *  Created on: May 14, 2018
 *      Author: sveb36
 */

// c++
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <time.h>
#include <unistd.h>
// ros
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
// opencv
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

//! copy the following code for benchmarking:
/*
  	clock_t t1 = clock();

	- methods to be benchmarked -

	clock_t t2 = clock();
	cout << "fSecondTS - fFirstTimeStamp: " << (float(t2 - t1)/CLOCKS_PER_SEC) << endl;
 */

class CCameraOverlay
{
	ros::NodeHandle m_oNodeHandle;
	image_transport::ImageTransport m_oImgTransport;
	image_transport::Subscriber m_oImageSub;
	image_transport::Publisher m_oImagePub;
	geometry_msgs::Pose m_oPose;

	Mat m_oMapImg;

	ros::Subscriber m_oOdomSub;

public:

	CCameraOverlay()
	: m_oImgTransport(m_oNodeHandle)
	{
		m_oMapImg = imread("../../../../localization/captures/Lab_map_600x400_scaled.png", IMREAD_GRAYSCALE);

		// Subscribe to input video feed and publish output video feed
		m_oImageSub = m_oImgTransport.subscribe("/usb_cam/image_undistorted", 1,
		  &CCameraOverlay::ImageCallback, this);
		m_oImagePub = m_oImgTransport.advertise("/camera_overlay", 1);

		m_oOdomSub = m_oNodeHandle.subscribe("/odom", 1, &CCameraOverlay::OdomCallback, this);

//		namedWindow("Image window");
	}

	~CCameraOverlay()
	{
//		destroyWindow("Image window");
	}

	void OdomCallback(const nav_msgs::OdometryConstPtr& msg)
	{
		m_oPose = msg->pose.pose;
	}

	void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
//		clock_t t1 = clock();

		cv_bridge::CvImagePtr pCvImg;

		Mat oMapImg = m_oMapImg.clone();

		try
		{
		  pCvImg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8 /*BGR8*/);
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}

		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
	    findContours(pCvImg->image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0,0));
	    cout << "contour count: " << contours.size() << endl;
	    Mat oContourImg = Mat::zeros( pCvImg->image.size(), CV_8U);
		for( size_t i = 0; i< contours.size(); i++ )
		{
			Scalar color = Scalar( 255 );
			drawContours( oContourImg, contours, (int)i, color, 3, 8, hierarchy, 0, Point() );
		}

	    int nPixelX = (600.0 - m_oPose.position.y * 100.0) * (130.0 / 70.0);
	    int nPixelY = (600.0 - m_oPose.position.x * 100.0) * (130.0 / 70.0);
	    Point oCenter(nPixelX, nPixelY);

	    Point2f oSmallImageCenter((oContourImg.cols - 1) / 2.0, (oContourImg.rows - 1) / 2.0);
	    double theta = acos(m_oPose.orientation.w) * 2;
	    double k3 = m_oPose.orientation.z / (sin(theta /2));
	    double fAngle = (theta / k3) * (180 / M_PI) + 180;
	    Mat oRotMat = getRotationMatrix2D(oSmallImageCenter, fAngle, 1.0);

	    Rect2f oBoundingBox = RotatedRect(oSmallImageCenter, oContourImg.size(), fAngle).boundingRect2f();
	    oRotMat.at<double>(0,2) += oBoundingBox.width / 2.0 - oContourImg.cols / 2.0;
	    oRotMat.at<double>(1,2) += oBoundingBox.height / 2.0 - oContourImg.rows / 2.0;

	    Mat oRotatedImage;
	    warpAffine(oContourImg, oRotatedImage, oRotMat, oBoundingBox.size());

	    try {

	    Point oTopLeft(oCenter.x - oRotatedImage.cols / 2, oCenter.y - oRotatedImage.rows / 2);
	    Rect oRegionOfInterest(oTopLeft, oRotatedImage.size());

	    int nCroppedX = oTopLeft.x < 0 ? -oTopLeft.x : 0;
	    int nCroppedY = oTopLeft.y < 0 ? -oTopLeft.y : 0;
	    int nCroppedWidth = oRegionOfInterest.width - nCroppedX - 1;
	    int nCroppedHeight = oRegionOfInterest.height - nCroppedY - 1;

	    nCroppedWidth -= oTopLeft.x + oRegionOfInterest.width >= oMapImg.cols ?
	    		oTopLeft.x + oRegionOfInterest.width - oMapImg.cols: 0;
	    nCroppedHeight -= oTopLeft.y + oRegionOfInterest.height >= oMapImg.rows ?
	    		oTopLeft.y + oRegionOfInterest.height - oMapImg.rows: 0;

	    Rect oCropRoi(nCroppedX, nCroppedY, nCroppedWidth, nCroppedHeight);
	    Mat oCroppedRotatedImage = oRotatedImage(oCropRoi);

	    oRegionOfInterest = Rect(Point(oTopLeft.x + nCroppedX,
	    		oTopLeft.y + nCroppedY), oCroppedRotatedImage.size());

	    Mat oDestinationRoi = oMapImg(oRegionOfInterest);
	    oCroppedRotatedImage.copyTo(oDestinationRoi, oCroppedRotatedImage);

	    }
	    catch(Exception& e)
	    {
			  ROS_ERROR("img crop exception: %s", e.what());
	    }

	    try
	    {
	    	cv::circle(oMapImg, oCenter, 10, (255,0,255), -1);
	    }
		catch(Exception& e)
		{
			ROS_ERROR("img crop exception: %s", e.what());

		}

		// Update GUI Window
//		cv::imshow("Image window", oMapImg);
//		cv::waitKey(3);

		sensor_msgs::ImagePtr oPubMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", oMapImg).toImageMsg();
		// Output modified video stream
		m_oImagePub.publish(oPubMsg);

//		clock_t t2 = clock();
//		cout << "fSecondTS - fFirstTimeStamp: " << (float(t2 - t1)/CLOCKS_PER_SEC) << endl;
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_overlay");
	CCameraOverlay oImgTest;
	ros::spin();
	return 0;
}
