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

using namespace cv;


const double SCALE_TO_PX = 1114.0 * 100 / 600.0;
const double SCALE_TO_M = 600.0 / (1114.0 * 100);
const Size CAR_SIZE = Size(44, 92);
const Size MAP_SIZE = Size(80 - 1, 120 - 1);

// convenience functions

double GetDistance(Point oP1, Point oP2)
{
	return sqrt(pow(oP1.x - oP2.x, 2) + pow(oP1.y - oP2.y, 2));
}
double GetDistance(Point2d oP1, Point2d oP2)
{
	return sqrt(pow(oP1.x - oP2.x, 2) + pow(oP1.y - oP2.y, 2));
}
double GetVectorLength(Point2d oVec)
{
	return GetDistance(Point2d(0,0), oVec);
}





#endif /* SRC_UTILITY_H_ */
