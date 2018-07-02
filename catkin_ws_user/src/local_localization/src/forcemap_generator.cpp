/*
 * forcemap_generator.cpp
 *
 *  Created on: May 29, 2018
 *      Author: sh
 */

// c++
#include <iostream>
#include <math.h>
// opencv
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
// ros
#include "ros/package.h"

#include "utility.h"

/**
 * generates a force map like in the paper:
 * http://page.mi.fu-berlin.de/rojas/2003/matrix.pdf
 * (p.5)
 */
class CForceMapGenerator
{
private:
	// class vars
	cv::Mat m_oImage;
	cv::Mat2d m_oForceMap;
	cv::Mat2d m_oDistanceMap;
	const double EXP_CONST = 10;


	// private methods

	void CreateForceMap()
	{
		cv::Mat oThreshedImg;
		cv::threshold(m_oImage, oThreshedImg, 30, 255, CV_8UC1);
		m_oImage = oThreshedImg;

		// make bigger mat and put map in the center of it
		cv::Mat oWorkingMat = cv::Mat::zeros(m_oImage.size() * 2, CV_8UC1);
		cv::Rect roi(m_oImage.cols / 2, m_oImage.rows / 2, m_oImage.cols, m_oImage.rows);
		m_oImage.copyTo(oWorkingMat(roi));

		// create empty forcemap and distancemap
		m_oForceMap = cv::Mat2d::zeros(MAP_SIZE.height, MAP_SIZE.width);
		m_oDistanceMap = cv::Mat2d::zeros(MAP_SIZE.height, MAP_SIZE.width);

		// get the contours
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours( oWorkingMat, contours, hierarchy, CV_RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point(0, 0) );

		// visualization
	    cv::Mat oContourImg = cv::Mat::zeros( oWorkingMat.size(), CV_8UC1);
//		for( size_t i = 0; i< contours.size(); i++ )
//		{
//			cv::Scalar color = cv::Scalar( 255 );
//			drawContours( oContourImg, contours, (int)i, color, 1, 8, hierarchy, 0, cv::Point() );
//		}

//		cv::namedWindow("kek", cv::WINDOW_NORMAL);
//		imshow("kek", oWorkingMat);
//		cv::waitKey(0);
//		imshow("kek", oContourImg);
//		cv::waitKey(0);
//		exit(0);

		std::cout << "contour count: " << contours.size() << std::endl;
		// create empty weight map
		int aSizes[] = {MAP_SIZE.height, MAP_SIZE.width, contours.size()};
		cv::Mat oWeightMap(3, aSizes, CV_64FC1, cv::Scalar(0));

		int nStartingOffset = 4;
		// the following for loops calculate the weights as in the paper on p.5
		for (int y = 0; y < MAP_SIZE.height; y++)
		{
			// nRow, nCol are pixel coordinate on scaled WorkingMat
			int nRow = nStartingOffset + 10 * y;
			for (int x = 0; x < MAP_SIZE.width; x++)
			{
				int nCol = nStartingOffset + 10 * x;

				// get distances to every contour and sum them up
				for (int nContOuter = 0; nContOuter < contours.size(); nContOuter++)
				{
					double fDividentSum = 0;
					for (int nCont = 0; nCont < contours.size(); nCont++)
					{
						// pixel distance
						double fDistance = fabs(pointPolygonTest(contours[nCont], cv::Point2d(nCol, nRow), true));
						fDividentSum += exp(-fDistance * CM_TO_M / EXP_CONST);
					}
					double fDistanceOuter = fabs(pointPolygonTest(contours[nContOuter], cv::Point2d(nCol, nRow), true));
					double fWeight = exp(fDistanceOuter * CM_TO_M / EXP_CONST) / fDividentSum;
					oWeightMap.at<double>(y, x, nContOuter) = fWeight;
				}
			}
			std::cout << "weightmap (nRow): (" << nRow << ")" << std::endl;
		}

		// these loops calculate the actual force vectors as in the paper p.5
		for (int y = 0; y < MAP_SIZE.height; y++)
		{
			// nRow, nCol are pixel coordinate on scaled WorkingMat
			int nRow = nStartingOffset + 10 * y;
			for (int x = 0; x < MAP_SIZE.width; x++)
			{
				int nCol = nStartingOffset + 10 * x;

				// initialize distancemap on first value for the comparison later
				m_oDistanceMap.at<cv::Point2d>(y, x) = cv::Point2d(contours[0][0] - cv::Point(nCol, nRow)) *  CM_TO_M;
				cv::Point2d oForceVector(0,0);
				// brute-force-find the closest point / shortest vector of a contour:
				// iterate through contours
				for (int nCont = 0; nCont < contours.size(); nCont++)
				{
					// initialize on first number
					double fMinDistance = GetDistance(contours[nCont][0], cv::Point(nCol, nRow));
					cv::Point2d oMinVector = cv::Point2d(contours[nCont][0] - cv::Point(nCol, nRow)) *  CM_TO_M;

					// iterate through pixels in the contour (skip first number)
					for (int nPixelCount = 1; nPixelCount < contours[nCont].size(); nPixelCount++)
					{
						double fDist = GetDistance(contours[nCont][nPixelCount], cv::Point(nCol, nRow));
						if (fDist < fMinDistance)
						{
							fMinDistance = fDist;
							oMinVector = cv::Point2d(contours[nCont][nPixelCount] - cv::Point(nCol, nRow)) *  CM_TO_M;
						}
					}
					if (fMinDistance < GetVectorLength(m_oDistanceMap.at<cv::Point2d>(y, x)))
					{
						m_oDistanceMap.at<cv::Point2d>(y, x) = oMinVector;
					}
					oForceVector += oWeightMap.at<double>(y, x, nCont) * oMinVector;
				}
				m_oForceMap.at<cv::Point2d>(y, x) = oForceVector;

				std::cout << "(x,y): " << x << "," << y << "; m_oDistanceMap.at<Point2d>(y, x) " << m_oDistanceMap.at<cv::Point2d>(y, x) << std::endl;
			}
		}

		double fLongestDistance = 0;
		double fLargestForce = 0;
		// normalize the vectors to the length of the longest one
		for (int y = 0; y < MAP_SIZE.height; y++)
		{
			for (int x = 0; x < MAP_SIZE.width; x++)
			{
				if (GetVectorLength(m_oForceMap[x][y]) > fLargestForce)
					fLargestForce = GetVectorLength(m_oForceMap[x][y]);
				if (GetVectorLength(m_oDistanceMap[x][y]) > fLongestDistance)
					fLongestDistance = GetVectorLength(m_oDistanceMap[x][y]);
			}
		}
		m_oForceMap /= fLargestForce;
		m_oDistanceMap /= fLongestDistance;
	}

public:

	// constructor and callbacks

	CForceMapGenerator(cv::Mat oImg)
	{
		oImg.convertTo(m_oImage, CV_8U);
		CreateForceMap();
	}

	cv::Mat GetForceMap()
	{
		return m_oForceMap;
	}

	cv::Mat GetDistanceMap()
	{
		return m_oDistanceMap;
	}

	void SaveDistanceMapToFile(std::string sFilename)
	{
		if (m_oDistanceMap.empty())
		{
			std::cerr << "No DistanceMap supplied ..." << std::endl;
			return;
		}
		cv::FileStorage file(sFilename, cv::FileStorage::WRITE, "UTF-8");
		file << "DistanceMap" << m_oDistanceMap;
		file.release();
	}

	void SaveForceMapToFile(std::string sFilename)
	{
		if (m_oForceMap.empty())
		{
			std::cerr << "No ForceMap supplied ..." << std::endl;
			return;
		}
		cv::FileStorage file(sFilename, cv::FileStorage::WRITE, "UTF-8");
		file << "ForceMap" << m_oForceMap;
		file.release();
	}

	static void SaveDistanceMapToFile(std::string sFilename, cv::Mat oDistanceMap)
	{
		if (oDistanceMap.empty())
		{
			std::cerr << "No DistanceMap supplied ..." << std::endl;
			return;
		}
		cv::FileStorage file(sFilename, cv::FileStorage::WRITE, "UTF-8");
		file << "DistanceMap" << oDistanceMap;
		file.release();
	}

	static void SaveForceMapToFile(std::string sFilename, cv::Mat oForceMap)
	{
		if (oForceMap.empty())
		{
			std::cerr << "No ForceMap supplied ..." << std::endl;
			return;
		}
		cv::FileStorage file(sFilename, cv::FileStorage::WRITE, "UTF-8");
		file << "ForceMap" << oForceMap;
		file.release();
	}


	static void DrawForceMap()
	{
		std::string sMapPath = ros::package::getPath(PACKAGE_NAME) + "/mapTables/";

		cv::Mat oForceMap;
		cv::FileStorage fs(sMapPath + "forcemap.xml", cv::FileStorage::READ);
		fs["ForceMap"] >> oForceMap;

		cv::RNG rng;

		cv::Mat oDrawMat = cv::Mat::zeros(oForceMap.rows * 101, oForceMap.cols * 101, CV_8UC3);
		for (int nFMapCol = 0; nFMapCol < oForceMap.cols; nFMapCol++)
		{
			int nDMatCol = 50 + nFMapCol * 100;
			for (int nFMapRow = 0; nFMapRow < oForceMap.rows; nFMapRow++)
			{
				int nDMatRow = 50 + nFMapRow * 100;

				cv::Point2d oForceVector = oForceMap.at<cv::Point2d>(nFMapRow, nFMapCol);
				oForceVector = oForceVector / (GetVectorLength(oForceVector) == 0 ? 1 : GetVectorLength(oForceVector)) * 75;
				cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
				cv::Point oCoordinate(nDMatCol, nDMatRow);
				arrowedLine(oDrawMat, oCoordinate, oCoordinate + cv::Point(oForceVector), color, 5, cv::LINE_AA);
			}
		}
		namedWindow("ForceMap", cv::WINDOW_NORMAL);
		imshow("ForceMap", oDrawMat);
		cv::waitKey(0);

	}

	static void DrawDistanceMap()
	{
		std::string sMapPath = ros::package::getPath(PACKAGE_NAME) + "/mapTables/";

		cv::Mat oDistanceMap;
		cv::FileStorage fs2(sMapPath + "distancemap.xml", cv::FileStorage::READ);
		fs2["DistanceMap"] >> oDistanceMap;

		cv::RNG rng;

		cv::Mat oDrawMat = cv::Mat::zeros(oDistanceMap.rows * 101, oDistanceMap.cols * 101, CV_8UC3);
		for (int nDMapCol = 0; nDMapCol < oDistanceMap.cols; nDMapCol++)
		{
			int nDrawMatCol = 50 + nDMapCol * 100;
			for (int nDMapRow = 0; nDMapRow < oDistanceMap.rows; nDMapRow++)
			{
				int nDrawMatRow = 50 + nDMapRow * 100;

				cv::Point2d oDistVector = oDistanceMap.at<cv::Point2d>(nDMapRow, nDMapCol);
				oDistVector = oDistVector / (GetVectorLength(oDistVector) == 0 ? 1 : GetVectorLength(oDistVector)) * 75;
				cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
				cv::Point oCoordinate(nDrawMatCol, nDrawMatRow);
				arrowedLine(oDrawMat, oCoordinate, oCoordinate + cv::Point(oDistVector), color, 5, cv::LINE_AA);
			}
		}
		namedWindow("DistanceMap", cv::WINDOW_NORMAL);
		imshow("DistanceMap", oDrawMat);
		cv::waitKey(0);
	}

};



void CreateMaps()
{
	std::string sImagePath = ros::package::getPath(PACKAGE_NAME) + "/images/";
	std::string sImageName("Lab_map_600x400.png");
	cv::Mat src = imread(sImagePath + sImageName, cv::IMREAD_GRAYSCALE);
	if (src.empty())
	{
		std::cerr << "No image supplied ..." << std::endl;
		exit(-1);
	}

	CForceMapGenerator oForceMapGenerator(src);

	std::string sMapPath = ros::package::getPath(PACKAGE_NAME) + "/mapTables/";
	oForceMapGenerator.SaveDistanceMapToFile(sMapPath + "distancemap.xml");
	oForceMapGenerator.SaveForceMapToFile(sMapPath + "forcemap.xml");
}

int main(int argc, char **argv)
{
	CreateMaps();

//	CForceMapGenerator::DrawForceMap();
//	CForceMapGenerator::DrawDistanceMap();

	return(0);
}
