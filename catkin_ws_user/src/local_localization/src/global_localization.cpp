/*
 * global_localization.cpp
 *
 *  Created on: Sep 17, 2018
 *      Author: svenh
 */


// c++
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <time.h>
#include <unistd.h>
// ros
#include "ros/package.h"
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
cv::Mat GetTransformationMatrix(int nCamWidth, int nCamHeight, int nMapWidth, int nMapHeight, cv::Point oCenter, double dYaw)
{
	// I have no idea why the following should be necessary...
	dYaw = dYaw * 180 / M_PI;
	// what this does is, it converts the angle from radians to degrees.
	// why radians -> degrees? no idea.

	//! for now we assume everything is always front facing
//		if (m_bIsFrontFacingCamera)
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
//		else
//		{
//			// turn the image points around the image center
//			cv::Point2f oRotationCenter(nCamWidth / 2.0, nCamHeight / 2.0);
//			cv::Mat oRotMat = getRotationMatrix2D(oRotationCenter, dYaw, 1.0);
//			// add a translation to the matrix
//			oRotMat.at<double>(0,2) += nMapWidth / 2 + oCenter.x - nCamWidth/2.0;
//			oRotMat.at<double>(1,2) += nMapHeight / 2 + oCenter.y - nCamHeight/2.0;
//
//			return oRotMat;
//		}
}


//! for debugging purposes only
//! shows position and rotation of the car with corresponding vectors
void ShowPos(const cv::Mat& oContourImg, const cv::Mat& oRotMat, cv::Point oCenter, const cv::Mat& oOtherMapImg, const cv::Mat& oVectorField)
{
	cv::Mat oMapImg = cv::Mat::zeros(oOtherMapImg.size() * 2, CV_8UC3);
	oOtherMapImg.copyTo(oMapImg(cv::Rect(oMapImg.size() / 4, oOtherMapImg.size())));

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
				cv::Rect rect(cv::Point(), oVectorField.size());
				if (!rect.contains(oVectorFieldCoordinate))
				{
//						std::cout << "out of forcemap range: " << oVectorFieldCoordinate << std::endl;
					continue;
				}
				cv::Point2d oForceVector = oVectorField.at<cv::Point2d>(oVectorFieldCoordinate.y, oVectorFieldCoordinate.x) * 1000;
				oForceVectorSum += oForceVector;

				oMapImg.at<cv::Vec3b>(oMapCoordinate.y, oMapCoordinate.x) = cv::Vec3b(0,255,255);
				if (nArrowCounter == 0)
					arrowedLine(oMapImg, oMapCoordinate, oMapCoordinate + cv::Point(oForceVector), cv::Scalar(255,255,0), 1, cv::LINE_AA);
				nArrowCounter = ++nArrowCounter % 9;

				counter++;
			}
		}
	}
	cv::circle(oMapImg, oCenter, 8, cv::Scalar(0, 255, 0), -1);
	counter = counter == 0 ? 1 : counter;
//	arrowedLine(oMapImg, oCenter, oCenter + cv::Point(oForceVectorSum / counter * 10), cv::Scalar(0,0,255), 2, cv::LINE_AA);

	cv::imshow("pos", oMapImg);
	cv::waitKey(0);
}


/**
 * gets the match quality (the lengths of the force vectors added) for a position and rotation of the car.
 * Position and rotation are contained in the transformation matrix.
 * @param oForceMap		the force vector field this position should be matched to
 * @param oTransMat		the transformation matrix containing rotation and translation (3x2)
 * @param oImg			the prepared camera image of the car
 * @param oMapImgSize	the size of the map image
 * @return
 */
double GetMatchQuality(const cv::Mat& oVectorField, const cv::Mat& oTransMat, const cv::Mat& oImg, const cv::Size& oMapImgSize)
{
	cv::Rect2f oBoundingBox(cv::Point2f(0,0), oMapImgSize * 2);

	double dForceVectorLength = 0;

	int nCounter = 0;

	for (int x = 0; x < oImg.cols; x++)
	{
		for (int y = 0; y < oImg.rows; y++)
		{
			if (oImg.at<uchar>(y, x) > 128)
			{
				cv::Mat1d oValueMat(3, 1);
				oValueMat.at<double>(0) = x;
				oValueMat.at<double>(1) = y;
				oValueMat.at<double>(2) = 1;
				oValueMat = oTransMat * oValueMat;
				cv::Point oVectorFieldCoordinate = cv::Point(oValueMat) / VECTOR_FIELD_DISTANCE;
				cv::Rect rect(cv::Point(), oVectorField.size());
				if (!rect.contains(oVectorFieldCoordinate))
				{
					// if the point is outside the already enlarged map, just add the biggest vector length to the quality value
					dForceVectorLength += GetVectorLength(oVectorField.at<cv::Point2d>(0,0));
//					std::cout << "index out of bounds! " << cv::Point(oValueMat) << " --> " << oVectorFieldCoordinate << std::endl;
					continue;
				}
				double dVectorLength = GetVectorLength(oVectorField.at<cv::Point2d>(oVectorFieldCoordinate.y, oVectorFieldCoordinate.x));
				dForceVectorLength += dVectorLength;
				nCounter++;
			}
		}
	}
	nCounter = nCounter == 0 ? 1 : nCounter;
	return dForceVectorLength / nCounter;
}


void GetGlobalPositionAndAngle(cv::Point2d& oPosition, double& dAngle, const cv::Mat& oImg)
{
	std::string sImagePath = ros::package::getPath(PACKAGE_NAME) + "/images/";
	std::string sMapPath = ros::package::getPath(PACKAGE_NAME) + "/mapTables/";
	cv::Mat oMapImg = cv::imread(sImagePath + "fu_robotics_lab_map.jpg", cv::IMREAD_COLOR);
	cv::Mat oVectorField;
//	cv::FileStorage fs(sMapPath + "forcemap.xml", cv::FileStorage::READ);
//	fs["ForceMap"] >> oVectorField;
	cv::FileStorage fs2(sMapPath + "distancemap_4.xml", cv::FileStorage::READ);
	fs2["DistanceMap"] >> oVectorField;

	std::cout << "vector field size: " << oVectorField.size() << std::endl;

	const double dScalingInCm = 80;

	// iterate through half of the map
	int nColsToIterate = oMapImg.cols / 2;
	int nRowsToIterate = oMapImg.rows;

	// for every meter there is a point
	int nXDistance = (nColsToIterate - 1) / round(nColsToIterate / dScalingInCm);
	int nYDistance = (nRowsToIterate - 1) / round(nRowsToIterate / dScalingInCm);
	double dAngleDistance = 2.0 * M_PI / 32.0;

	double dMinimumForceVectorLength = -1;

  	clock_t t1 = clock();

	for (int x = 0; x < nColsToIterate; x += nXDistance)
	{
		for (int y = 0; y < nRowsToIterate; y += nYDistance)
		{
			for (double dCurrentAngle = -M_PI; dCurrentAngle < M_PI; dCurrentAngle += dAngleDistance)
			{
				cv::Point oCurrentPos = cv::Point2d(x, y);
				cv::Mat oTransMat = GetTransformationMatrix(oImg.cols, oImg.rows, oMapImg.cols, oMapImg.rows, oCurrentPos, -dCurrentAngle);
				double dMatchQuality = GetMatchQuality(oVectorField, oTransMat, oImg, oMapImg.size());

				std::cout << "x,y=(" << x << "," << y << ") angle=" << dCurrentAngle << " dMatchQuality=" << dMatchQuality << std::endl;
//				if (dMatchQuality < 0.05)
//					ShowPos(oImg, oTransMat, oCurrentPos, oMapImg, oVectorField);

				if (dMinimumForceVectorLength < 0 || dMinimumForceVectorLength > dMatchQuality)
				{
					dMinimumForceVectorLength = dMatchQuality;
					oPosition = cv::Point2d(x,y) / 100.0;
					dAngle = dCurrentAngle;
				}
			}
		}
	}

	cv::destroyAllWindows();
//	oPosition = cv::Point2d(0,0);
//	dAngle = 0;

	clock_t t2 = clock();
	std::cout << "global localization time: " << (float(t2 - t1)/CLOCKS_PER_SEC) << std::endl;
	std::cout << "global pos: (" << oPosition.x << "," << oPosition.y << ") angle: " << dAngle << " match quality: " << dMinimumForceVectorLength << std::endl;
}






