/*
 * utility.h
 *
 *  Created on: Jun 8, 2018
 *      Author: sh
 */

#ifndef SRC_UTILITY_H_
#define SRC_UTILITY_H_



#include <opencv2/core.hpp>
#include <math.h>

const std::string PACKAGE_NAME = "local_localization";

const double M_TO_CM = 100.0;
const double CM_TO_M = 1.0/100.0;
//! grid size of the vector field compared to its original image
const int VECTOR_FIELD_DISTANCE = 4;
//! car size is in cm
const cv::Size CAR_SIZE = cv::Size(26, 60);
//const cv::Size CAR_SIZE = cv::Size(0, 0);

//! 50 cm distance between the camera and the first point it can see in the forward camera perspective
//! 22 cm distance between the camera and the hind axis
const double FOV_DISTANCE_TO_CAR = 50 + 22;

// convenience functions

/**
 * calculates euclidean distance between two points:
 * sqrt( (P1.x - P2.x)² + (P1.y - P2.y)² )
 * @param oP1	Point 1
 * @param oP2	Point 2
 * @return		euclidean distance between the points
 */
double GetDistance(cv::Point oP1, cv::Point oP2)
{
	return sqrt(pow(oP1.x - oP2.x, 2) + pow(oP1.y - oP2.y, 2));
}

/**
 * calculates euclidean distance between two points:
 * sqrt( (P1.x - P2.x)² + (P1.y - P2.y)² )
 * @param oP1	Point2d 1
 * @param oP2	Point2d 2
 * @return		euclidean distance between the points
 */
double GetDistance(cv::Point2d oP1, cv::Point2d oP2)
{
	return sqrt(pow(oP1.x - oP2.x, 2) + pow(oP1.y - oP2.y, 2));
}

/**
 * calculates euclidean distance between two points:
 * sqrt( (P1.x - P2.x)² + (P1.y - P2.y)² + (P1.z - P2.z)² )
 * @param oP1	Point2d 1
 * @param oP2	Point2d 2
 * @return		euclidean distance between the points
 */
double GetDistance(cv::Point3d oP1, cv::Point3d oP2)
{
	return sqrt(pow(oP1.x - oP2.x, 2) + pow(oP1.y - oP2.y, 2) + pow(oP1.z - oP2.z, 2));
}

/**
 * calculates length of a vector by calculating the distance between (0,0) and the Point
 * @param oVec	Vector, represented as Point2d
 * @return		length of the Vector
 */
double GetVectorLength(cv::Point2d oVec)
{
	return GetDistance(cv::Point2d(0,0), oVec);
}

/**
 * calculates length of a vector by calculating the distance between (0,0,0) and the Point
 * @param oVec	Vector, represented as Point2d
 * @return		length of the Vector
 */
double GetVectorLength(cv::Point3d oVec)
{
	return GetDistance(cv::Point3d(0,0,0), oVec);
}

//! copy the following code for benchmarking:
/*
  	clock_t t1 = clock();

	- methods to be benchmarked -

	clock_t t2 = clock();
	std::cout << "ts: " << (float(t2 - t1)/CLOCKS_PER_SEC) << std::endl;
 */



#endif /* SRC_UTILITY_H_ */
