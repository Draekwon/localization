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
#include "geometry_msgs/Point.h"
// opencv
#include "opencv2/core.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "utility.h"

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
	ros::Publisher m_oOdomPub;

	geometry_msgs::Pose m_oOdomPose;
	geometry_msgs::Pose m_oOldOdomPose;
	geometry_msgs::Pose m_oCurrentPose;

	ros::Publisher vis_pub;

	Mat m_oMapImg;
	Mat m_oForceMap;
	Mat m_oDistanceMap;

	ros::Subscriber m_oOdomSub;

public:

	CCameraOverlay()
	: m_oImgTransport(m_oNodeHandle)
	{
		m_oMapImg = imread("../../../captures/Lab_map_600x400_scaled.png", IMREAD_GRAYSCALE);

		cv::FileStorage fs("../../../forcemap.xml", cv::FileStorage::READ);
		fs["ForceMap"] >> m_oForceMap;
		cv::FileStorage fs2("../../../distancemap.xml", cv::FileStorage::READ);
		fs2["DistanceMap"] >> m_oDistanceMap;

		cout << "forcemap size " << m_oForceMap.size << endl;

		m_oOdomPose.position.x = 0;
		m_oOdomPose.position.y = 0;
		m_oOdomPose.position.z = 0;
		m_oOdomPose.orientation = tf::createQuaternionMsgFromYaw(0);
		m_oOldOdomPose.position.x = 0;
		m_oOldOdomPose.position.y = 0;
		m_oOldOdomPose.position.z = 0;
		m_oOldOdomPose.orientation = tf::createQuaternionMsgFromYaw(0);
		m_oCurrentPose.position.x = 0;
		m_oCurrentPose.position.y = 0;
		m_oCurrentPose.position.z = 0;
		m_oCurrentPose.orientation = tf::createQuaternionMsgFromYaw(0);

		// publish a position by publishing odometry
		m_oOdomPub = m_oNodeHandle.advertise<nav_msgs::Odometry>("MATRIX_Location", 50);
		m_oImagePub = m_oImgTransport.advertise("/camera_overlay", 1);

		// Subscribe to input video feed
		m_oImageSub = m_oImgTransport.subscribe("/usb_cam/image_undistorted", 1,
		  &CCameraOverlay::ImageCallback, this);
		m_oOdomSub = m_oNodeHandle.subscribe("/odom", 1, &CCameraOverlay::OdomCallback, this);
	}

	~CCameraOverlay()
	{
		destroyWindow("Display window");
	}


	void OdomCallback(const nav_msgs::OdometryConstPtr& msg)
	{
		m_oOdomPose = msg->pose.pose;
	}

	Point2d GetForceVector(Mat oRotatedImg, Point3d oOdomPos, double &oTorque)
	{
		Point3d tempPoint = Point3d(6, 4, 0) - oOdomPos;
		cout << "tempPoint: " << tempPoint << endl;
		tempPoint *= SCALE_TO_PX;
		Point oCenter = Point(tempPoint.y, tempPoint.x);

		Point oMatrixMinimum(0 - m_oMapImg.cols / 2, 0 - m_oMapImg.rows / 2);
		Point oMatrixMaximum(m_oMapImg.cols + m_oMapImg.cols / 2, m_oMapImg.rows + m_oMapImg.rows / 2);

		Point oTopLeft(oCenter.x - oRotatedImg.cols / 2, oCenter.y - oRotatedImg.rows / 2);
		Point oBottomRight(oTopLeft.x + oRotatedImg.cols, oTopLeft.y + oRotatedImg.rows);

		int nCroppedX = oTopLeft.x < oMatrixMinimum.x ? oMatrixMinimum.x - oTopLeft.x : 0;
		int nCroppedY = oTopLeft.y < oMatrixMinimum.y ? oMatrixMinimum.y - oTopLeft.y : 0;
		int nCroppedX2 = oBottomRight.x > oMatrixMaximum.x ? oBottomRight.x - oMatrixMaximum.x : 0;
		int nCroppedY2 = oBottomRight.y > oMatrixMaximum.y ? oBottomRight.y - oMatrixMaximum.y : 0;
		int nCroppedWidth = oRotatedImg.cols - nCroppedX - nCroppedX2;
		int nCroppedHeight = oRotatedImg.rows - nCroppedY - nCroppedY2;

		Rect oCropRoi(nCroppedX, nCroppedY, nCroppedWidth, nCroppedHeight);
		Mat oCroppedRotatedImage = oRotatedImg(oCropRoi);

		Point oCroppedTopLeft(oTopLeft.x + nCroppedX, oTopLeft.y + nCroppedY);

//		// publish the overlay to double check rotation and stuff
//		Rect oRegionOfInterest = Rect(oCroppedTopLeft, oCroppedRotatedImage.size());
//		PublishCroppedRotatedImage(oCroppedRotatedImage, oRegionOfInterest, oCenter);

		const double fMapUnit = 0.1/*10 cm*/ * SCALE_TO_PX;

		// its complicated...
		// if (cutoff) map index * unit size + half unit size is outside the rotated image rect,
		// add 1.
		// example: 186 / 100 = 1.86; cutoff = 1, 
		// add half unit size = 1 * unit size + unit size /2 = 150
		// which is smaller than 186, so add one to the unit count:
		// 2 * unit size + 0.5 * unit size = 250, which is bigger than 186.
		int nStartingOffsetX = oCroppedTopLeft.x / fMapUnit;
		nStartingOffsetX = nStartingOffsetX * fMapUnit + fMapUnit / 2 < oCroppedTopLeft.x ?
			(nStartingOffsetX + 1) * fMapUnit + fMapUnit / 2 :
			nStartingOffsetX * fMapUnit + fMapUnit / 2;
		int nStartingOffsetY = oCroppedTopLeft.y / fMapUnit;
		nStartingOffsetY = nStartingOffsetY * fMapUnit + fMapUnit / 2 < oCroppedTopLeft.y ?
			(nStartingOffsetY + 1) * fMapUnit + fMapUnit / 2 :
			nStartingOffsetY * fMapUnit + fMapUnit / 2;

		Point2d oForceVector(0,0);
		int counter = 0;
		for (int nCol = nStartingOffsetX; nCol < oCroppedRotatedImage.cols; nCol += fMapUnit)
		{
			for (int nRow = nStartingOffsetY; nRow < oCroppedRotatedImage.rows; nRow += fMapUnit)
			{
				if (oCroppedRotatedImage.at<int>(nCol, nRow) > 128)
				{
					// convert to map units
					int x = (nCol + oCroppedTopLeft.x) / (fMapUnit);
					int y = (nRow + oCroppedTopLeft.y) / (fMapUnit);
					counter++;
//					cout << "[x,y]: " << x << " " << y << endl;
					oForceVector += m_oDistanceMap.at<Point2d>(x, y);
					Point2d distanceVec = Point2d(nCol - oCenter.x, nRow - oCenter.y);
					oTorque += distanceVec.cross(oForceVector);
				}
			}
		}
//		cout << "counter: " << counter << endl;
		oForceVector /= counter == 0 ? 1 : counter;
		oTorque /= counter == 0 ? 1 : counter;

		return oForceVector;
	}

	void PublishCroppedRotatedImage(Mat oCroppedImg, Rect roi, Point2d oCenter)
	{
		Mat oMapImg = Mat::zeros(m_oMapImg.size() * 2, CV_8UC1);
		m_oMapImg.copyTo(oMapImg(Rect(oMapImg.size() / 4, m_oMapImg.size())));
		roi.x += oMapImg.cols / 4;
		roi.y += oMapImg.rows / 4;
		oCenter.x += oMapImg.cols / 4;
		oCenter.y += oMapImg.rows / 4;
		Mat oDestinationRoi = oMapImg(roi);
		oCroppedImg.copyTo(oDestinationRoi, oCroppedImg);
		cv::circle(oMapImg, oCenter, 10, 255, -1);

		sensor_msgs::ImagePtr oPubMsg = cv_bridge::CvImage(std_msgs::Header(),
				sensor_msgs::image_encodings::MONO8, /*oContourImg*/oMapImg).toImageMsg();
		// Output modified video stream
		m_oImagePub.publish(oPubMsg);
	}

	Mat GetRotatedImg(Mat oContourImg, double fYaw)
	{
		// I have no idea why the following should be necessary...
		fYaw = fYaw * 180 / M_PI - 180;
		// what this does is, it converts from radians to degrees and then switches front and back.

	    Point2f oImageCenter((oContourImg.cols - 1) / 2.0, (oContourImg.rows - 1) / 2.0);
	    Mat oRotMat = getRotationMatrix2D(oImageCenter, fYaw, 1.0);

	    Rect2f oBoundingBox = RotatedRect(oImageCenter, oContourImg.size(), fYaw).boundingRect2f();
	    oRotMat.at<double>(0,2) += oBoundingBox.width / 2.0 - oContourImg.cols / 2.0;
	    oRotMat.at<double>(1,2) += oBoundingBox.height / 2.0 - oContourImg.rows / 2.0;

	    Mat oRotatedImage;
	    warpAffine(oContourImg, oRotatedImage, oRotMat, oBoundingBox.size());

	    return oRotatedImage;
	}

	void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		clock_t t1 = clock();

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

		// hide the car
		Rect oModelCarRect(Rect(pCvImg->image.cols / 2 - CAR_SIZE.width / 2,
				pCvImg->image.rows / 2 - CAR_SIZE.height / 2,
				CAR_SIZE.width, CAR_SIZE.height));
		Mat emptyMat = Mat::zeros(CAR_SIZE.height, CAR_SIZE.width, CV_8UC1);
		emptyMat.copyTo(pCvImg->image(oModelCarRect));

		// get contours
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
	    findContours(pCvImg->image, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0,0));
	    cout << "contour count: " << contours.size() << endl;
	    Mat oContourImg = Mat::zeros( pCvImg->image.size(), CV_8UC1);
		for( size_t i = 0; i< contours.size(); i++ )
		{
			Scalar color = Scalar( 255 );
			drawContours( oContourImg, contours, (int)i, color, 1, 8, hierarchy, 0, Point() );
		}

		// blur for making getting more accurate force vectors easier
		// kind of a hack I guess
		Mat oBlurredImg = Mat::zeros(oContourImg.size(), CV_8UC1);
		blur(oContourImg, oBlurredImg, Size(5,5));

		// this should basically be the odometry of the car
		double fOdomYaw = tf::getYaw(m_oOdomPose.orientation);
		double fOldOdomYaw = tf::getYaw(m_oOldOdomPose.orientation);
		double fYaw = tf::getYaw(m_oCurrentPose.orientation);
		double fDifferentialYaw = fYaw + fOdomYaw - fOldOdomYaw;

		// ros does not like to add positions, so convert them to Opencv points...
		Point3d oOdomPosePosition(m_oOdomPose.position.x, m_oOdomPose.position.y, m_oOdomPose.position.z);
		Point3d oOldOdomPosePosition(m_oOldOdomPose.position.x, m_oOldOdomPose.position.y, m_oOldOdomPose.position.z);
		Point3d oCurrentPosePosition(m_oCurrentPose.position.x, m_oCurrentPose.position.y, m_oCurrentPose.position.z);
		Point3d oPosition = oCurrentPosePosition + oOdomPosePosition - oOldOdomPosePosition;
		try {
			double oTorque = 0;
			Mat oRotatedImg = GetRotatedImg(oBlurredImg, fDifferentialYaw);
			Point2d oForceVector = GetForceVector(oRotatedImg, oPosition, oTorque);


			cout << "oTorque " << oTorque << endl;
			// force vectors are scaled, so that the longest one is 1m
			// scale them down so that the longest is 1cm
			oForceVector *= 0.01;
			cout << "forcevector2 " << GetVectorLength(oForceVector) << endl;
			// add the force vector to the position
			oPosition.x += isinf(oForceVector.x) || isnan(oForceVector.x) ? 0 : oForceVector.x;
			oPosition.y += isinf(oForceVector.y) || isnan(oForceVector.y) ? 0 : oForceVector.y;

			cout << "Position: " << oPosition << ", forcevector: " << oForceVector << endl;

			nav_msgs::Odometry odom;
			odom.header.stamp = ros::Time::now();
			odom.header.frame_id = "odom";

			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(fDifferentialYaw);
			m_oCurrentPose.orientation = odom_quat;
			m_oCurrentPose.position.x = oPosition.x;
			m_oCurrentPose.position.y = oPosition.y;
			m_oCurrentPose.position.z = oPosition.z;

			//set the position
			odom.pose.pose = m_oCurrentPose;

			//set the velocity
	//		odom.child_frame_id = "base_link";

			//publish the message
			m_oOdomPub.publish(odom);
		}catch(Exception&)
		{

		}
		m_oOldOdomPose = m_oOdomPose;

		clock_t t2 = clock();
		cout << "ts: " << (float(t2 - t1)/CLOCKS_PER_SEC) << endl;
	}
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_overlay");
	CCameraOverlay oImgTest;
	ros::spin();
	return 0;
}
