/*
 * camera_overlay.cpp
 *
 *  Created on: May 14, 2018
 *      Author: sh
 */

// c++
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <time.h>
#include <unistd.h>
// ros
#include "ros/ros.h"
#include "ros/package.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/String.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include "nav_msgs/Odometry.h"
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
#include "opencv2/plot.hpp"

#include "global_localization.cpp"
#include "utility.h"

/**
 * This class subscribes to the /odom and /usb_cam/image_undistorted topics
 * to apply the MATRIX-algorithm and publish a new /MATRIX_Location
 * topic, as odometry.
 */
class CCameraOverlay
{
	ros::NodeHandle m_oNodeHandle;
	image_transport::ImageTransport m_oImgTransport;
	image_transport::Subscriber m_oImageSub;
	image_transport::Publisher m_oImagePub;
	ros::Publisher m_oOdomPub;
	tf::TransformBroadcaster m_oTransformPub;

	geometry_msgs::Pose m_oVisualOdomPose;
	geometry_msgs::Pose m_oOdomPose;
	geometry_msgs::Pose m_oOldOdomPose;
	geometry_msgs::Pose m_oCurrentPose;

	ros::Publisher vis_pub;

	cv::Mat m_oMapImg;
	cv::Mat m_oVectorField;

	ros::Subscriber m_oOdomSub;
	ros::Subscriber m_oVisualOdomSub;

	bool 	m_bPublishOverlay;
	double	m_fScaleFactor;

	bool m_bIsFrontFacingCamera = true;

	bool m_bIsFirstTime = true;
	bool m_bIsLocalizing = false;

	std::string m_sWindowName = "CamOverlay";

	cv::Point3d m_oFirstOdomToCeilingDist = cv::Point3d(-1,-1,-1);
	cv::Mat1d m_oCarOdomPlotData, m_oCarYawPlotData, m_oMatrixOdomPlotData, m_oMatrixYawPlotData, m_oPlotTimeStamps;
	std::string m_sCarOdomPlotData, m_sCarYawPlotData, m_sMatrixOdomPlotData, m_sMatrixYawPlotData, m_sPlotTimeStamps;


public:

	/**
	 * subscribes to /odom and /usb_cam/image_undistorted and publishes /MATRIX_Location (odometry) and /camera_overlay (image)
	 * @param fScaleFactor		scales the camera image so that 1px = 1cm
	 * @param sImageTopic		sets the image topic this class should subscribe to. If empty, this class does not subscribe to an image topic.
	 * @param bPublishOverlay	sets publishing of overlay to true or false.
	 */
	CCameraOverlay(double fScaleFactor, std::string sImageTopic = "", bool bPublishOverlay = false)
	: m_oImgTransport(m_oNodeHandle), m_fScaleFactor(fScaleFactor), m_bPublishOverlay(bPublishOverlay)
	{
		std::string sImagePath = ros::package::getPath(PACKAGE_NAME) + "/images/";
		std::string sMapPath = ros::package::getPath(PACKAGE_NAME) + "/mapTables/";

		m_oMapImg = imread(sImagePath + "fu_robotics_lab_map.jpg", cv::IMREAD_GRAYSCALE);
//		m_oMapImg = imread(sImagePath + "Lab_map_600x400.png", cv::IMREAD_GRAYSCALE);

		cv::FileStorage fs(sMapPath + "forcemap_4.xml", cv::FileStorage::READ);
		fs["ForceMap"] >> m_oVectorField;
//		cv::FileStorage fs2(sMapPath + "distancemap_4.xml", cv::FileStorage::READ);
//		fs2["DistanceMap"] >> m_oVectorField;

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

//		cv::namedWindow(m_sWindowName, CV_WINDOW_NORMAL);

		// publish a position by publishing odometry
		m_oOdomPub = m_oNodeHandle.advertise<nav_msgs::Odometry>("MATRIX_Location", 50);
		if (m_bPublishOverlay)
			m_oImagePub = m_oImgTransport.advertise("/camera_overlay", 1);

		// Subscribe to input video feed
//		m_oImageSub = m_oImgTransport.subscribe("/forward_rectified", 1,
//		  &CCameraOverlay::ImageCallback, this, image_transport::TransportHints("compressed"));
		if (!sImageTopic.empty())
		{
			m_oImageSub = m_oImgTransport.subscribe(sImageTopic, 1,
			  &CCameraOverlay::ImageCallback, this, image_transport::TransportHints("compressed"));
		}
		m_oVisualOdomSub = m_oNodeHandle.subscribe("/localization/odom/0", 1, &CCameraOverlay::VisualOdomCallback, this);
		m_oOdomSub = m_oNodeHandle.subscribe("/odom", 1, &CCameraOverlay::OdomCallback, this);
	}

	~CCameraOverlay()
	{
//	    cv::Ptr<cv::plot::Plot2d> pMatrixOdomPlot = cv::plot::Plot2d::create( m_oPlotTimeStamps, m_oMatrixOdomPlotData );
//	    cv::Ptr<cv::plot::Plot2d> pCarOdomPlot = cv::plot::Plot2d::create( m_oPlotTimeStamps, m_oCarOdomPlotData );
//	    cv::Ptr<cv::plot::Plot2d> pMatrixYawPlot = cv::plot::Plot2d::create( m_oPlotTimeStamps, m_oMatrixYawPlotData );
//	    cv::Ptr<cv::plot::Plot2d> pCarYawPlot = cv::plot::Plot2d::create( m_oPlotTimeStamps, m_oCarYawPlotData );
//	    cv::Mat oMatrixOdomPlotImg, oCarOdomPlotImg, oMatrixYawPlotImg, oCarYawPlotImg;
//	    pMatrixOdomPlot->render(oMatrixOdomPlotImg);
//	    pCarOdomPlot->render(oCarOdomPlotImg);
//	    pMatrixYawPlot->render(oMatrixYawPlotImg);
//	    pCarYawPlot->render(oCarYawPlotImg);
//
//	    cv::imwrite("oMatrixOdomPlotImg.png", oMatrixOdomPlotImg);
//	    cv::imwrite("oCarOdomPlotImg.png", oCarOdomPlotImg);
//	    cv::imwrite("oMatrixYawPlotImg.png", oMatrixYawPlotImg);
//	    cv::imwrite("oCarYawPlotImg.png", oCarYawPlotImg);


	    std::ofstream out("output.csv");

	    out << m_sPlotTimeStamps << std::endl << m_sMatrixOdomPlotData << std::endl;
	    out << m_sMatrixYawPlotData << std::endl << m_sCarOdomPlotData << std::endl;
	    out << m_sCarYawPlotData;
	    out.close();

	    m_oOdomPub.shutdown();
	    m_oOdomSub.shutdown();
	    m_oVisualOdomSub.shutdown();
	    m_oImagePub.shutdown();
	    m_oImageSub.shutdown();
	}


protected:

	bool m_bVisualOdomSet = false;
	bool m_bFirstOdom = true;

	void VisualOdomCallback(const nav_msgs::OdometryConstPtr& msg)
	{
		m_oVisualOdomPose = msg->pose.pose;
		if (!m_bVisualOdomSet)
		{
			m_oCurrentPose = msg->pose.pose;
			m_bVisualOdomSet = true;
		}
	}

	/**
	 * this callback saves the current odometry in a class variable
	 * @param msg	the odometry-message
	 */
	void OdomCallback(const nav_msgs::OdometryConstPtr& msg)
	{
		m_oOdomPose = msg->pose.pose;
		if (m_oFirstOdomToCeilingDist == cv::Point3d(-1,-1,-1) && m_bVisualOdomSet)
		{
			m_oFirstOdomToCeilingDist.x = m_oVisualOdomPose.position.x - m_oOdomPose.position.x;
			m_oFirstOdomToCeilingDist.y = m_oVisualOdomPose.position.y - m_oOdomPose.position.y;
			m_oFirstOdomToCeilingDist.z = m_oVisualOdomPose.position.z - m_oOdomPose.position.z;
		}
		if (m_bFirstOdom)
		{
			m_bFirstOdom = false;
			m_oOldOdomPose = m_oOdomPose;
		}
	}

	/**
	 * this method rotates and overlays the camera image on the map. the result is published to "/camera_overlay"
	 * @param oContourImg	the image contours of the camera image
	 * @param fYaw			the angle in radians
	 * @param oCenter		the assumed position of the car in pixel coordinates
	 */
	void PublishMapOverlay(const cv::Mat& oContourImg, const cv::Mat& oRotMat, cv::Point oCenter, cv::Point3d oOdomCenter, cv::Point3d oVisualOdomCenter)
	{
		cv::Mat oMapImg = cv::Mat::zeros(m_oMapImg.size() * 2, CV_8UC3);
		cv::Rect2f oBoundingBox(cv::Point2f(0,0), m_oMapImg.size() * 2);

		cv::Mat oBgrMap;
		cv::cvtColor(m_oMapImg, oBgrMap, cv::COLOR_GRAY2BGR);
		oBgrMap.copyTo(oMapImg(cv::Rect(oMapImg.size() / 4, m_oMapImg.size())));
		oOdomCenter *= M_TO_CM;
		oOdomCenter.x += oMapImg.cols / 4;
		oOdomCenter.y += oMapImg.rows / 4;
		oVisualOdomCenter *= M_TO_CM;
		oVisualOdomCenter.x += oMapImg.cols / 4;
		oVisualOdomCenter.y += oMapImg.rows / 4;
		oCenter.x += oMapImg.cols / 4;
		oCenter.y += oMapImg.rows / 4;


		cv::Point2d oForceVectorSum;
		int counter = 0;
		int nArrowCounter = 0;

		for (int x = 0; x < oContourImg.cols; x++)
		{
			for (int y = 0; y < oContourImg.rows; y++)
			{
				if (oContourImg.at<uchar>(y, x) > 128)
				{
					cv::Mat1d oValueMat(3, 1);
					oValueMat.at<double>(0) = x;
					oValueMat.at<double>(1) = y;
					oValueMat.at<double>(2) = 1;
					oValueMat = oRotMat * oValueMat;
					cv::Point oVectorFieldCoordinate = cv::Point(oValueMat) / VECTOR_FIELD_DISTANCE;
					cv::Point oMapCoordinate = cv::Point(oValueMat);
					cv::Rect rect(cv::Point(), m_oVectorField.size());
					if (!rect.contains(oVectorFieldCoordinate))
					{
//						std::cout << "out of forcemap range: " << oVectorFieldCoordinate << std::endl;
						continue;
					}
					cv::Point2d oForceVector = m_oVectorField.at<cv::Point2d>(oVectorFieldCoordinate.y, oVectorFieldCoordinate.x) * 1000;
					oForceVectorSum += oForceVector;

					oMapImg.at<cv::Vec3b>(oMapCoordinate.y, oMapCoordinate.x) = cv::Vec3b(0,255,255);
					if (nArrowCounter == 0)
						arrowedLine(oMapImg, oMapCoordinate, oMapCoordinate + cv::Point(oForceVector), cv::Scalar(255,255,0), 1, cv::LINE_AA);
					nArrowCounter = ++nArrowCounter % 9;

					counter++;
				}
			}
		}
		cv::circle(oMapImg, cv::Point(oVisualOdomCenter.x, oOdomCenter.y), 5, cv::Scalar(255, 0, 0), -1);
		cv::circle(oMapImg, cv::Point(oOdomCenter.x, oOdomCenter.y), 5, cv::Scalar(255, 0, 255), -1);
		cv::circle(oMapImg, oCenter, 5, cv::Scalar(0, 255, 0), -1);
		counter = counter == 0 ? 1 : counter;
		arrowedLine(oMapImg, oCenter, oCenter + cv::Point(oForceVectorSum / counter * 10), cv::Scalar(0,0,255), 2, cv::LINE_AA);

		sensor_msgs::ImagePtr oPubMsg = cv_bridge::CvImage(std_msgs::Header(),
				sensor_msgs::image_encodings::BGR8, oMapImg).toImageMsg();
		m_oImagePub.publish(oPubMsg);
	}


	/**
	 *
	 * @param nCamWidth		width of the camera image
	 * @param nCamHeight	height of the camera image
	 * @param nMapWidth		width of the map
	 * @param nMapHeight	height of the map
	 * @param oCenter		current assumed position of the car in image coordinates
	 * @param fYaw			current assumed rotation of the car in radians
	 * @return				the transformation matrix
	 */
	cv::Mat GetTransformationMatrix(int nCamWidth, int nCamHeight, int nMapWidth, int nMapHeight, cv::Point2d oCenter, double dYaw)
	{
		// I have no idea why the following should be necessary...
		dYaw = dYaw * 180 / M_PI;
		// what this does is, it converts from radians to degrees and then switches front and back.
		// front and back switch is simple: camera image is reversed.
		// but radians -> degrees? no idea.

		if (m_bIsFrontFacingCamera)
		{
			dYaw -= 90;
			dYaw = dYaw < -180 ? dYaw + 360 : dYaw;

			// turn the image points around the car
			cv::Point2f oRotationCenter(nCamWidth / 2.0, nCamHeight + FOV_DISTANCE_TO_CAR);
			cv::Mat oRotMat = getRotationMatrix2D(oRotationCenter, dYaw, 1.0);
			// add a translation to the matrix
			oRotMat.at<double>(0,2) += nMapWidth / 2 + oCenter.x - nCamWidth / 2.0;
			oRotMat.at<double>(1,2) += nMapHeight / 2 + oCenter.y - nCamHeight - FOV_DISTANCE_TO_CAR;

			return oRotMat;
		}
		else
		{
			// turn the image points around the image center
			cv::Point2f oRotationCenter(nCamWidth / 2.0, nCamHeight / 2.0);
			cv::Mat oRotMat = getRotationMatrix2D(oRotationCenter, dYaw, 1.0);
			// add a translation to the matrix
			oRotMat.at<double>(0,2) += nMapWidth / 2 + oCenter.x - nCamWidth/2.0;
			oRotMat.at<double>(1,2) += nMapHeight / 2 + oCenter.y - nCamHeight/2.0;

			return oRotMat;
		}

	}

	/**
	 * uses a transformationmatrix to map image pixels to the force field and returns the average of the force vectors
	 * @param oTransMat	transformation matrix
	 * @param oImg		image
	 * @param oTorque	the torque calculated by using the 2D-cross product. It will be saved in this var.
	 * @param oCenter	the assumed position of the  car
	 * @return			the averaged force vector
	 */
	cv::Point2d GetForceVector(const cv::Mat& oTransMat, const cv::Mat& oImg, double &oTorque, cv::Point2d oCenter)
	{
		cv::Point2d oForceVector(0,0);
		int counter = 0;

		cv::Rect2f oBoundingBox(cv::Point2f(0,0), m_oMapImg.size() * 2);

		for (int x = 0; x < oImg.cols; x++)
		{
			for (int y = 0; y < oImg.rows; y++)
			{
				// the type uchar is REALLY REALLY important
				if (oImg.at<uchar>(y, x) > 128)
				{
					cv::Mat1d oValueMat(3, 1);
					oValueMat.at<double>(0) = x;
					oValueMat.at<double>(1) = y;
					oValueMat.at<double>(2) = 1;
					oValueMat = oTransMat * oValueMat;
					cv::Point oVectorFieldCoordinate = cv::Point(oValueMat) / VECTOR_FIELD_DISTANCE;
					cv::Rect rect(cv::Point(), m_oVectorField.size());
					if (!rect.contains(oVectorFieldCoordinate))
					{
						continue;
					}

					oForceVector += m_oVectorField.at<cv::Point2d>(oVectorFieldCoordinate.y, oVectorFieldCoordinate.x);
					cv::Point oMapCoordinate = cv::Point(oValueMat);
					cv::Point2d distanceVec = cv::Point2d(oMapCoordinate.x - oCenter.x, oMapCoordinate.y - oCenter.y);
					oTorque += distanceVec.cross(oForceVector);

					counter++;
				}
			}
		}

//		oForceVector = counter == 0 ? 1 : counter;
//		oTorque /= counter == 0 ? 1 : counter;
		return oForceVector * CM_TO_M;
	}

	/**
	 * scales, filters and converts the camera image to gray scale
	 * @param oCameraImg	Camera image
	 * @return
	 */
	cv::Mat PrepareCameraImage(cv::Mat oCameraImg)
	{
		// maybe erode and dilute before scaling the img?

		cv::Mat oTmpImg;
		cv::resize(oCameraImg, oTmpImg, cv::Size(), m_fScaleFactor, m_fScaleFactor);
		oCameraImg = oTmpImg;

		// flip by y axis
		cv::flip(oCameraImg, oTmpImg, 1);
		oCameraImg = oTmpImg;

		// hide the car
		if (!m_bIsFrontFacingCamera)
		{
			cv::Rect oModelCarRect(cv::Rect(oCameraImg.cols / 2 - CAR_SIZE.width / 2,
					oCameraImg.rows / 2 - CAR_SIZE.height / 2,
					CAR_SIZE.width, CAR_SIZE.height));
			cv::Mat emptyMat = cv::Mat::zeros(CAR_SIZE.height, CAR_SIZE.width, CV_8UC3);
			emptyMat.copyTo(oCameraImg(oModelCarRect));

			// rotate by 90 degrees
			cv::rotate(oCameraImg, oTmpImg, cv::ROTATE_90_COUNTERCLOCKWISE);
			oCameraImg = oTmpImg;
		}

//		cv::Mat oYuvImg;
//		cv::cvtColor(oCameraImg, oYuvImg, CV_BGR2YUV);
//		cv::Mat oRangedImg;
//		cv::inRange(oYuvImg, cv::Scalar(120, 0, 0), cv::Scalar(255, 255, 255), oRangedImg);

		return oCameraImg;
	}


	/**
	 * callback for /image_undistorted.
	 * This is the main part of this class.
	 * Here, the camera image is used to improve the position the odometry returns.
	 * @param msg 	The image-message containing the camera-image.
	 */
	void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr pCvImg;

		try
		{
		  pCvImg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}

		ProcessImg(pCvImg->image);
	}


	void ProcessImg(cv::Mat oImg)
	{
		if (!m_bVisualOdomSet)
			return;

		clock_t t1 = clock();

		// this should basically be the odometry of the car
		double fOdomYaw = tf::getYaw(m_oOdomPose.orientation);
		double fOldOdomYaw = tf::getYaw(m_oOldOdomPose.orientation);
		double fYaw = tf::getYaw(m_oCurrentPose.orientation);
		double fVisualYaw = tf::getYaw(m_oVisualOdomPose.orientation);


		// ros does not like to add positions, so convert them to Opencv points...
		cv::Point3d oVisualPosePosition(m_oVisualOdomPose.position.x, m_oVisualOdomPose.position.y, m_oVisualOdomPose.position.z);
		cv::Point3d oOdomPosePosition(m_oOdomPose.position.x, m_oOdomPose.position.y, m_oOdomPose.position.z);
		cv::Point3d oOldOdomPosePosition(m_oOldOdomPose.position.x, m_oOldOdomPose.position.y, m_oOldOdomPose.position.z);
		cv::Point3d oCurrentPosePosition(m_oCurrentPose.position.x, m_oCurrentPose.position.y, m_oCurrentPose.position.z);

		double fDifferentialYaw;
		cv::Point3d oPosition;

		double dOdomDiff;
		if ((fOdomYaw < -2 && fOldOdomYaw > 2) || (fOdomYaw > 2 && fOldOdomYaw < -2))
			dOdomDiff = abs(fOdomYaw + fOldOdomYaw);
		else
			dOdomDiff = abs(fOdomYaw - fOldOdomYaw);

		if (!m_bFirstOdom)
		{
			fDifferentialYaw = fYaw + fOdomYaw - fOldOdomYaw;
			oPosition = oCurrentPosePosition + oOdomPosePosition - oOldOdomPosePosition;
		}
		else
		{
			fDifferentialYaw = fYaw ;
			oPosition = oCurrentPosePosition;
		}

		//! this adds a normal distribution to the odometry like in the paper
		//! comment this out to save (a lot) of performance
//		{
//			cv::RNG rng(12354);
//			cv::Mat3d oRandomPositions(1,10);
//			cv::Vec3d deviation(abs(oOdomPosePosition.x - oOldOdomPosePosition.x), abs(oOdomPosePosition.y - oOldOdomPosePosition.y), dOdomDiff);
//			rng.fill(oRandomPositions, cv::RNG::NORMAL, cv::Vec3d(oPosition.x, oPosition.y, fDifferentialYaw), deviation);
//			double dRating = -1;
//			for (int nCol = 0; nCol < oRandomPositions.cols; nCol++)
//			{
//				cv::Vec3d oNormalPosition = oRandomPositions.at<cv::Vec3d>(0, nCol);
//				cv::Point oNormalCenter = cv::Point(oNormalPosition[0] * M_TO_CM, oNormalPosition[1] * M_TO_CM);
//				cv::Mat oNormalTransMat = GetTransformationMatrix(oPreparedCameraImg.cols, oPreparedCameraImg.rows,
//						m_oMapImg.cols, m_oMapImg.rows, oNormalCenter, -oNormalPosition[2]);
//				double dCurrentRating = GetMatchQuality(m_oForceMap, oNormalTransMat, oPreparedCameraImg, m_oMapImg.size());
//
//				if (dCurrentRating < dRating || dRating < 0)
//				{
//					dRating = dCurrentRating;
//					oPosition.x = oNormalPosition[0];
//					oPosition.y = oNormalPosition[1];
//					// assuming z = 0
//					fDifferentialYaw = oNormalPosition[2];
//				}
//			}
//		}

		double dTorque = 0;
		//! oCenter is the position of the car in image coordinates (cm, since we defined 1px = 1cm on the map image)
		cv::Point oCenter = cv::Point(oPosition.x * M_TO_CM, oPosition.y * M_TO_CM);

		cv::Mat oPreparedCameraImg = PrepareCameraImage(oImg);
		cv::Mat oTransformationMat = GetTransformationMatrix(oPreparedCameraImg.cols, oPreparedCameraImg.rows,
				m_oMapImg.cols, m_oMapImg.rows, oCenter, -fDifferentialYaw);
		// force in meter
		cv::Point2d oForceVector = GetForceVector(oTransformationMat, oPreparedCameraImg, dTorque, oCenter);

		// angle - value = turn left
		// angle + value = turn right
		double dAngleCorrection;

		double dTorqueMin = std::min(0.0001 * abs(dTorque), 0.1 * M_PI / 180);
		if (dTorqueMin < 0.01 * M_PI / 180)
			dTorqueMin = 0;

		std::cout << "dTorqueMin: " << dTorqueMin << std::endl;

		if (dTorque < 0)
			dAngleCorrection -= 0.1 * dOdomDiff + dTorqueMin ;
		else
			dAngleCorrection += 0.1 * dOdomDiff + dTorqueMin;

		double dForceVecLen = std::min(0.01 * GetVectorLength(oForceVector), 0.01);
		if (dForceVecLen < 0.0004)
			dForceVecLen = 0;

		std::cout << "dForceVecLen: " << dForceVecLen << std::endl;

		// normalize the force vectorfOdomYaw
		oForceVector /= GetVectorLength(oForceVector);
		// set the length of the force vector that will be added to the odometry as correction
		// right now its 10% of the length of the movement + at most 0.01
		oForceVector *= (GetVectorLength(oOdomPosePosition - oOldOdomPosePosition) * 0.1  + dForceVecLen);


//		if (m_bIsFirstTime && !m_bIsLocalizing)
//		{
//			m_bIsLocalizing = true;
//
//			cv::Point2d oFirstPosition;
//			double dFirstAngle;
////			GetGlobalPositionAndAngle(oFirstPosition, dFirstAngle, oPreparedCameraImg);
////			oPosition.x = oFirstPosition.x;
////			oPosition.y = oFirstPosition.y;
////			fDifferentialYaw = dFirstAngle;
//			oPosition.x = 0;
//			oPosition.y = 0;
//			fDifferentialYaw = 0;
//
//			oForceVector = cv::Point2d(0,0);
//			dAngleCorrection = 0;
//			m_bIsFirstTime = false;
//			m_bIsLocalizing = false;
//
//		}
//		else if (m_bIsFirstTime && m_bIsLocalizing)
//		{
//			// if the global localization is running, dont do anything
//			return;
//		}

		std::cout << "dAngleCorrection: " << dAngleCorrection << std::endl;
		std::cout << "forcevector length " << GetVectorLength(oForceVector) << std::endl;
		// add the force vector to the position
		oPosition.x += std::isinf(oForceVector.x) || std::isnan(oForceVector.x) ? 0 : oForceVector.x;
		oPosition.y += std::isinf(oForceVector.y) || std::isnan(oForceVector.y) ? 0 : oForceVector.y;

		fDifferentialYaw += dAngleCorrection;

		if (fDifferentialYaw >= M_PI)
			fDifferentialYaw -= 2 * M_PI;
		if (fDifferentialYaw < -M_PI)
			fDifferentialYaw += 2 * M_PI;


		std::cout << "oTorque " << dTorque << ", fDifferentialYaw " << fDifferentialYaw << std::endl;
		std::cout << "Position: " << oPosition << ", forcevector: " << oForceVector << std::endl;

		if (m_bPublishOverlay)
			PublishMapOverlay(oPreparedCameraImg, oTransformationMat, oCenter, oOdomPosePosition, oVisualPosePosition);


		double dMatrixDistance = GetVectorLength(oPosition - oVisualPosePosition);
		double dOdomDistance = GetVectorLength((oOdomPosePosition + m_oFirstOdomToCeilingDist) - oVisualPosePosition);

		std::cout << "oOdomPosePosition " << oOdomPosePosition << " m_oFirstOdomToCeilingDist " << m_oFirstOdomToCeilingDist << " dOdomDistance " << dOdomDistance << std::endl;

		double dMatrixYawDistance, dOdomYawDistance;
		if ((fVisualYaw > 2 && fDifferentialYaw < -2) || (fVisualYaw < 2 && fDifferentialYaw > -2))
			dMatrixYawDistance = abs(fVisualYaw + fDifferentialYaw);
		else
			dMatrixYawDistance = abs(fVisualYaw - fDifferentialYaw);
		if ((fVisualYaw > 2 && fOdomYaw < -2) || (fVisualYaw < 2 && fOdomYaw > -2))
			dOdomYawDistance = abs(fVisualYaw + fOdomYaw);
		else
			dOdomYawDistance = abs(fVisualYaw - fOdomYaw);


		ros::Time oTimeStamp = ros::Time::now();

		m_oPlotTimeStamps.push_back(oTimeStamp.toSec());
		m_oMatrixOdomPlotData.push_back(dMatrixDistance);
		m_oCarOdomPlotData.push_back(dOdomDistance);
		m_oMatrixYawPlotData.push_back(dMatrixYawDistance * 180 / M_PI);
		m_oCarYawPlotData.push_back(dOdomYawDistance * 180 / M_PI);

		m_sPlotTimeStamps += std::to_string(oTimeStamp.toSec()) + ",";
		m_sMatrixOdomPlotData += std::to_string(dMatrixDistance) + ",";
		m_sCarOdomPlotData += std::to_string(dOdomDistance) + ",";
		m_sMatrixYawPlotData += std::to_string(dMatrixYawDistance * 180 / M_PI) + ",";
		m_sCarYawPlotData += std::to_string(dOdomYawDistance * 180 / M_PI) + ",";


	    cv::Ptr<cv::plot::Plot2d> pMatrixOdomPlot = cv::plot::Plot2d::create( m_oPlotTimeStamps, m_oMatrixOdomPlotData );
	    pMatrixOdomPlot->setShowText( true );
	    pMatrixOdomPlot->setShowGrid( false );
	    pMatrixOdomPlot->setPlotBackgroundColor( cv::Scalar( 255, 255, 255 ) );
	    pMatrixOdomPlot->setPlotLineColor( cv::Scalar( 255, 0, 0 ) );
	    pMatrixOdomPlot->setPlotLineWidth( 2 );
	    pMatrixOdomPlot->setPlotTextColor(cv::Scalar(0,0,0));
	    cv::Ptr<cv::plot::Plot2d> pCarOdomPlot = cv::plot::Plot2d::create( m_oPlotTimeStamps, m_oCarOdomPlotData );
	    pCarOdomPlot->setShowText( true );
	    pCarOdomPlot->setShowGrid( false );
	    pCarOdomPlot->setPlotBackgroundColor( cv::Scalar( 255, 255, 255 ) );
	    pCarOdomPlot->setPlotLineColor( cv::Scalar( 255, 0, 0 ) );
	    pCarOdomPlot->setPlotLineWidth( 2 );
	    pCarOdomPlot->setPlotTextColor(cv::Scalar(0,0,0));
	    cv::Ptr<cv::plot::Plot2d> pMatrixYawPlot = cv::plot::Plot2d::create( m_oPlotTimeStamps, m_oMatrixYawPlotData );
	    pMatrixYawPlot->setShowText( true );
	    pMatrixYawPlot->setShowGrid( false );
	    pMatrixYawPlot->setPlotBackgroundColor( cv::Scalar( 255, 255, 255 ) );
	    pMatrixYawPlot->setPlotLineColor( cv::Scalar( 255, 0, 0 ) );
	    pMatrixYawPlot->setPlotLineWidth( 2 );
	    pMatrixYawPlot->setPlotTextColor(cv::Scalar(0,0,0));
	    cv::Ptr<cv::plot::Plot2d> pCarYawPlot = cv::plot::Plot2d::create( m_oPlotTimeStamps, m_oCarYawPlotData );
	    pCarYawPlot->setShowText( true );
	    pCarYawPlot->setShowGrid( false );
	    pCarYawPlot->setPlotBackgroundColor( cv::Scalar( 255, 255, 255 ) );
	    pCarYawPlot->setPlotLineColor( cv::Scalar( 255, 0, 0 ) );
	    pCarYawPlot->setPlotLineWidth( 2 );
	    pCarYawPlot->setPlotTextColor(cv::Scalar(0,0,0));

	    cv::Mat oMatrixOdomPlotImg, oCarOdomPlotImg, oMatrixYawPlotImg, oCarYawPlotImg;
	    pMatrixOdomPlot->render(oMatrixOdomPlotImg);
	    pCarOdomPlot->render(oCarOdomPlotImg);
	    pMatrixYawPlot->render(oMatrixYawPlotImg);
	    pCarYawPlot->render(oCarYawPlotImg);

	    cv::imshow("oMatrixOdomPlotImg", oMatrixOdomPlotImg);
	    cv::imshow("oCarOdomPlotImg", oCarOdomPlotImg);
	    cv::imshow("oMatrixYawPlotImg", oMatrixYawPlotImg);
	    cv::imshow("oCarYawPlotImg", oCarYawPlotImg);
	    cv::waitKey(1);


		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(fDifferentialYaw);

		nav_msgs::Odometry odom;
		odom.header.stamp = oTimeStamp;
		odom.header.frame_id = "odom";

		m_oCurrentPose.orientation = odom_quat;
		m_oCurrentPose.position.x = oPosition.x;
		m_oCurrentPose.position.y = oPosition.y;
		m_oCurrentPose.position.z = oPosition.z;

		//set the position
		odom.pose.pose = m_oCurrentPose;

		//publish the message
		m_oOdomPub.publish(odom);

		m_oOldOdomPose = m_oOdomPose;

		clock_t t2 = clock();
		std::cout << "ts: " << (float(t2 - t1)/CLOCKS_PER_SEC) << std::endl;
	}


public:

	void ProcessNewImg(cv::Mat oNewImg)
	{
		ProcessImg(oNewImg);
	}

};


//int main(int argc, char **argv)
//{
//	ros::init(argc, argv, "camera_overlay");
////	CCameraOverlay oImgTest(67.0 / 144.0, /usb_cam/image_rectified, true);
//
//
//	ros::spin();
//	return 0;
//}
