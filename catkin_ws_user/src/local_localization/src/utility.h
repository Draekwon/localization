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
//! car size is in cm
const cv::Size CAR_SIZE = cv::Size(26, 60);
const cv::Size MAP_SIZE = cv::Size(120 - 1, 80 - 1);

// convenience functions

double GetDistance(cv::Point oP1, cv::Point oP2)
{
	return sqrt(pow(oP1.x - oP2.x, 2) + pow(oP1.y - oP2.y, 2));
}
double GetDistance(cv::Point2d oP1, cv::Point2d oP2)
{
	return sqrt(pow(oP1.x - oP2.x, 2) + pow(oP1.y - oP2.y, 2));
}
double GetVectorLength(cv::Point2d oVec)
{
	return GetDistance(cv::Point2d(0,0), oVec);
}

//! copy the following code for benchmarking:
/*
  	clock_t t1 = clock();

	- methods to be benchmarked -

	clock_t t2 = clock();
	std::cout << "ts: " << (float(t2 - t1)/CLOCKS_PER_SEC) << std::endl;
 */



#endif /* SRC_UTILITY_H_ */
