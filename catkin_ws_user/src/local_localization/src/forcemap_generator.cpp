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
// own
#include "utility.h"

/**
 * this class generates a distance- and a force vector field
 * as described in the following paper:
 * http://page.mi.fu-berlin.de/rojas/2003/matrix.pdf
 * (p.4-5)
 */
class CForceMapGenerator
{
private:
	// class vars
	cv::Mat m_oImage;
	cv::Mat2d m_oForceMap;
	cv::Mat2d m_oDistanceMap;
	const double EXP_CONST = 10;

	cv::Size m_oVectorFieldSize;


	// private methods

/**
 * calculates the shortest distance vector between a point and a line on an image
 * @param oWorkingMat	image the line and point are on
 * @param oPoint		the point
 * @param oLineStart	starting point of the line
 * @param oLineEnd		ending point of the line
 * @return				vector from the given point to the nearest point on the line
 */
	cv::Point2d GetShortestVectorToLine(cv::Mat oImg, cv::Point oPoint, cv::Point2f oLineStart, cv::Point2f oLineEnd)
	{
		cv::LineIterator oIterator(oImg, oLineStart, oLineEnd);
		double fMinLength = GetDistance(oPoint, oIterator.pos());
		cv::Point2d oShortestVector = oIterator.pos() - oPoint;
		oIterator++;
		for (int i = 1; i < oIterator.count; ++i, ++oIterator)
		{
			double fCurrentLength = GetDistance(oPoint, oIterator.pos());
			if (fCurrentLength < fMinLength)
			{
				fMinLength = fCurrentLength;
				oShortestVector = cv::Point2d(oIterator.pos() - oPoint);
			}
		}
		return oShortestVector;
	}



public:

	// constructor and callbacks

	/**
	 * constructor
	 * @param oImg	the image to be turned into a vector field
	 */
	CForceMapGenerator(cv::Mat oImg)
	{
		oImg.convertTo(m_oImage, CV_8U);
		m_oVectorFieldSize.height = m_oImage.rows * 2 / VECTOR_FIELD_DISTANCE;
		m_oVectorFieldSize.width = m_oImage.cols * 2 / VECTOR_FIELD_DISTANCE;
	}


	/**
	 * this method iterates through the given image multiple times to build a distance- and a force-potential field
	 * a scale factor of 1px = 1cm is assumed
	 * !this takes a long time!
	 */
	void CreateVectorFields()
	{
		cv::Mat oThreshedImg;
		cv::threshold(m_oImage, oThreshedImg, 30, 255, CV_8UC1);
		m_oImage = oThreshedImg;

		// make bigger mat and put map in the center of it
		cv::Mat oWorkingMat = cv::Mat::zeros(m_oImage.size() * 2, CV_8UC1);
		cv::Rect roi(m_oImage.cols / 2, m_oImage.rows / 2, m_oImage.cols, m_oImage.rows);
		m_oImage.copyTo(oWorkingMat(roi));

		// create empty forcemap and distancemap
		m_oForceMap = cv::Mat2d::zeros(m_oVectorFieldSize.height, m_oVectorFieldSize.width);
		m_oDistanceMap = cv::Mat2d::zeros(m_oVectorFieldSize.height, m_oVectorFieldSize.width);

	    // Create LineSegmentDetector
	    cv::Ptr<cv::LineSegmentDetector> pLsd = cv::createLineSegmentDetector();
	    std::vector<cv::Vec4f> aLsdLines;
	    pLsd->detect(oWorkingMat, aLsdLines);
		std::cout << "line count: " << aLsdLines.size() << std::endl;


	    // visualization
//	    cv::Mat oLineImg(oWorkingMat.rows, oWorkingMat.cols, CV_8UC3);
//	    pLsd->drawSegments(oLineImg, aLsdLines);
//	    cv::namedWindow("LSD", CV_WINDOW_NORMAL);
//	    imshow("LSD", oLineImg);
//	    cv::waitKey(0);
//	    exit(10);


		// create empty weight map
		int aSizes[] = {m_oVectorFieldSize.height, m_oVectorFieldSize.width, aLsdLines.size()};
		cv::Mat oWeightMap(3, aSizes, CV_64FC1, cv::Scalar(0));

		int nStartingOffset = VECTOR_FIELD_DISTANCE / 2 - 1;
		// the following for loops calculate the weights as in the paper on p.5
		for (int y = 0; y < m_oVectorFieldSize.height; y++)
		{
			// nRow, nCol are pixel coordinate on WorkingMat
			int nRow = nStartingOffset + VECTOR_FIELD_DISTANCE * y; // VECTOR_FIELD_DISTANCE cm steps
			for (int x = 0; x < m_oVectorFieldSize.width; x++)
			{
				int nCol = nStartingOffset + VECTOR_FIELD_DISTANCE * x;

				// for every line segment
				for (int nLineSegment = 0; nLineSegment < aLsdLines.size(); nLineSegment++)
				{
					double fDividentSum = 0;

					// get distances to every line segment and sum them up
					for (int nInnerLineSegment = 0; nInnerLineSegment < aLsdLines.size(); nInnerLineSegment++)
					{
						// pixel distance
						cv::Point2f oInnerLineStart(aLsdLines.at(nInnerLineSegment).val[0], aLsdLines.at(nInnerLineSegment).val[1]);
						cv::Point2f oInnerLineEnd(aLsdLines.at(nInnerLineSegment).val[2], aLsdLines.at(nInnerLineSegment).val[3]);
						double fDistance = GetVectorLength(GetShortestVectorToLine(oWorkingMat, cv::Point(nCol, nRow), oInnerLineStart, oInnerLineEnd));
						fDividentSum += exp(-fDistance / EXP_CONST);
					}
					// and save the formula in the array
					cv::Point2f oLineStart(aLsdLines.at(nLineSegment).val[0], aLsdLines.at(nLineSegment).val[1]);
					cv::Point2f oLineEnd(aLsdLines.at(nLineSegment).val[2], aLsdLines.at(nLineSegment).val[3]);
					double fDistanceOuter = GetVectorLength(GetShortestVectorToLine(oWorkingMat, cv::Point(nCol, nRow), oLineStart, oLineEnd));
					double fWeight = exp(-fDistanceOuter / EXP_CONST) / fDividentSum;
					oWeightMap.at<double>(y, x, nLineSegment) = fWeight;
				}
			}
			std::cout << "weightmap (nRow): (" << nRow << ")" << std::endl;
		}

		// these loops calculate the actual force vectors as in the paper p.5
		for (int y = 0; y < m_oVectorFieldSize.height; y++)
		{
			// nRow, nCol are pixel coordinates on WorkingMat
			int nRow = nStartingOffset + VECTOR_FIELD_DISTANCE * y;
			for (int x = 0; x < m_oVectorFieldSize.width; x++)
			{
				int nCol = nStartingOffset + VECTOR_FIELD_DISTANCE * x;

				// initialize distancemap on first value for the comparison later
				cv::Point2f oLineStart(aLsdLines.at(0).val[0], aLsdLines.at(0).val[1]);
				cv::Point2f oLineEnd(aLsdLines.at(0).val[2], aLsdLines.at(0).val[3]);
				m_oDistanceMap.at<cv::Point2d>(y, x) = GetShortestVectorToLine(oWorkingMat, cv::Point(nCol, nRow), oLineStart, oLineEnd);

				cv::Point2d oForceVector = oWeightMap.at<double>(y, x, 0) * m_oDistanceMap.at<cv::Point2d>(y, x);


				// brute-force-find the closest point / shortest vector of a contour:
				// iterate through line segments, skipping the first
				for (int nLineSegment = 1; nLineSegment < aLsdLines.size(); nLineSegment++)
				{
					oLineStart = cv::Point(aLsdLines.at(nLineSegment).val[0], aLsdLines.at(nLineSegment).val[1]);
					oLineEnd = cv::Point(aLsdLines.at(nLineSegment).val[2], aLsdLines.at(nLineSegment).val[3]);
					cv::Point2d oMinVector = GetShortestVectorToLine(oWorkingMat, cv::Point(nCol, nRow), oLineStart, oLineEnd);
					double fMinDistance = GetVectorLength(oMinVector);

					if (fMinDistance < GetVectorLength(m_oDistanceMap.at<cv::Point2d>(y, x)))
					{
						m_oDistanceMap.at<cv::Point2d>(y, x) = oMinVector;
					}
					oForceVector += oWeightMap.at<double>(y, x, nLineSegment) * oMinVector;
				}
				m_oForceMap.at<cv::Point2d>(y, x) = oForceVector;

				std::cout << "(x,y): " << x << "," << y << "; m_oDistanceMap.at<Point2d>(y, x) " << m_oDistanceMap.at<cv::Point2d>(y, x) << std::endl;
			}
		}

		double fLongestDistance = 0;
		double fLargestForce = 0;
		// normalize the vectors to the length of the longest one
		for (int y = 0; y < m_oVectorFieldSize.height; y++)
		{
			for (int x = 0; x < m_oVectorFieldSize.width; x++)
			{
				if (GetVectorLength(m_oForceMap.at<cv::Point2d>(y, x)) > fLargestForce)
					fLargestForce = GetVectorLength(m_oForceMap.at<cv::Point2d>(y, x));
				if (GetVectorLength(m_oDistanceMap.at<cv::Point2d>(y, x)) > fLongestDistance)
					fLongestDistance = GetVectorLength(m_oDistanceMap.at<cv::Point2d>(y, x));
				if (std::isinf(fLongestDistance) || std::isnan(fLongestDistance))
				{
					std::cout << x << " " << y << " dist " << m_oDistanceMap.at<cv::Point2d>(y, x) << fLongestDistance << std::endl;
					exit(-1);
				}
				if (std::isinf(fLargestForce) || std::isnan(fLargestForce))
				{
					std::cout << x << " " << y << " force " << m_oDistanceMap.at<cv::Point2d>(y, x) << fLargestForce << std::endl;
					exit(-1);
				}
			}
		}
		m_oForceMap /= fLargestForce;
		m_oDistanceMap /= fLongestDistance;
	}


	/**
	 * returns the generated distance vector field
	 * @return	the generated distance vector field
	 */
	cv::Mat2d GetDistanceMap()
	{
		return m_oDistanceMap;
	}

	/**
	 * returns the generated force vector field
	 * @return	the generated force vector field
	 */
	cv::Mat2d GetForceMap()
	{
		return m_oForceMap;
	}

	/**
	 * saves the generated distance vector field to a file
	 * @param sFilename		filepath and -name the distance vector field should be saved to
	 */
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

	/**
	 * saves the generated force vector field to a file
	 * @param sFilename		filepath and -name the force vector field should be saved to
	 */
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

	/**
	 * displays the generated distance vector field as an opencv image
	 */
	void DrawDistanceMap()
	{
		cv::Mat oDrawMat = cv::Mat::zeros(m_oDistanceMap.rows * 101, m_oDistanceMap.cols * 101, CV_8UC3);
		for (int nDMapCol = 0; nDMapCol < m_oDistanceMap.cols; nDMapCol++)
		{
			int nDrawMatCol = 50 + nDMapCol * 100;
			for (int nDMapRow = 0; nDMapRow < m_oDistanceMap.rows; nDMapRow++)
			{
				int nDrawMatRow = 50 + nDMapRow * 100;

				cv::Point2d oDistVector = m_oDistanceMap.at<cv::Point2d>(nDMapRow, nDMapCol) * 1000;
				if (GetVectorLength(oDistVector) < 50)
					oDistVector = oDistVector / (GetVectorLength(oDistVector) == 0 ? 1 : GetVectorLength(oDistVector)) * 50;
				else if (GetVectorLength(oDistVector) > 150)
					oDistVector = oDistVector / (GetVectorLength(oDistVector) == 0 ? 1 : GetVectorLength(oDistVector)) * 150;
				cv::Scalar color = cv::Scalar(0,0,255);
				cv::Point oCoordinate(nDrawMatCol, nDrawMatRow);
				arrowedLine(oDrawMat, oCoordinate, oCoordinate + cv::Point(oDistVector), color, 5, cv::LINE_AA);
			}
		}
		namedWindow("DistanceMap", cv::WINDOW_NORMAL);
		imshow("DistanceMap", oDrawMat);
		cv::waitKey(0);
	}

	/**
	 * displays the generated force vector field as an opencv image
	 */
	void DrawForceMap()
	{
		cv::Mat oDrawMat = cv::Mat::zeros(m_oForceMap.rows * 101, m_oForceMap.cols * 101, CV_8UC3);
		for (int nFMapCol = 0; nFMapCol < m_oForceMap.cols; nFMapCol++)
		{
			int nDMatCol = 50 + nFMapCol * 100;
			for (int nFMapRow = 0; nFMapRow < m_oForceMap.rows; nFMapRow++)
			{
				int nDMatRow = 50 + nFMapRow * 100;

				cv::Point2d oForceVector = m_oForceMap.at<cv::Point2d>(nFMapRow, nFMapCol) * 1000;
				if (GetVectorLength(oForceVector) < 50)
					oForceVector = oForceVector / (GetVectorLength(oForceVector) == 0 ? 1 : GetVectorLength(oForceVector)) * 50;
				else if (GetVectorLength(oForceVector) > 150)
					oForceVector = oForceVector / (GetVectorLength(oForceVector) == 0 ? 1 : GetVectorLength(oForceVector)) * 150;
				cv::Scalar color = cv::Scalar(0,0,255);
				cv::Point oCoordinate(nDMatCol, nDMatRow);
				arrowedLine(oDrawMat, oCoordinate, oCoordinate + cv::Point(oForceVector), color, 5, cv::LINE_AA);
			}
		}
		namedWindow("ForceMap", cv::WINDOW_NORMAL);
		imshow("ForceMap", oDrawMat);
		cv::waitKey(0);
	}

	/**
	 * loads a distance vector field from a file and displays it as an opencv image
	 * @param sMapPath		path to the file the distance vector field is saved in
	 */
	static void DrawDistanceMap(std::string sMapPath)
	{
		cv::Mat oDistanceMap;
		cv::FileStorage fs2(sMapPath, cv::FileStorage::READ);
		fs2["DistanceMap"] >> oDistanceMap;

		cv::RNG rng;

		cv::Mat oDrawMat = cv::Mat::zeros(oDistanceMap.rows * 101, oDistanceMap.cols * 101, CV_8UC3);
		for (int nDMapCol = 0; nDMapCol < oDistanceMap.cols; nDMapCol++)
		{
			int nDrawMatCol = 50 + nDMapCol * 100;
			for (int nDMapRow = 0; nDMapRow < oDistanceMap.rows; nDMapRow++)
			{
				int nDrawMatRow = 50 + nDMapRow * 100;

				cv::Point2d oDistVector = oDistanceMap.at<cv::Point2d>(nDMapRow, nDMapCol) * 1000;
				if (GetVectorLength(oDistVector) < 50)
					oDistVector = oDistVector / (GetVectorLength(oDistVector) == 0 ? 1 : GetVectorLength(oDistVector)) * 50;
				else if (GetVectorLength(oDistVector) > 150)
					oDistVector = oDistVector / (GetVectorLength(oDistVector) == 0 ? 1 : GetVectorLength(oDistVector)) * 150;
				cv::Scalar color = cv::Scalar(0,0,255);
				cv::Point oCoordinate(nDrawMatCol, nDrawMatRow);
				arrowedLine(oDrawMat, oCoordinate, oCoordinate + cv::Point(oDistVector), color, 5, cv::LINE_AA);
			}
		}
		namedWindow("DistanceMap", cv::WINDOW_NORMAL);
		imshow("DistanceMap", oDrawMat);
		cv::waitKey(0);
	}

	/**
	 * loads a force vector field from a file and displays it as an opencv image
	 * @param sMapPath		path to the file the force vector field is saved in
	 */
	static void DrawForceMap(std::string sMapPath)
	{
		cv::Mat oForceMap;
		cv::FileStorage fs(sMapPath, cv::FileStorage::READ);
		fs["ForceMap"] >> oForceMap;

		cv::RNG rng;

		cv::Mat oDrawMat = cv::Mat::zeros(oForceMap.rows * 101, oForceMap.cols * 101, CV_8UC3);
		for (int nFMapCol = 0; nFMapCol < oForceMap.cols; nFMapCol++)
		{
			int nDMatCol = 50 + nFMapCol * 100;
			for (int nFMapRow = 0; nFMapRow < oForceMap.rows; nFMapRow++)
			{
				int nDMatRow = 50 + nFMapRow * 100;

				cv::Point2d oForceVector = oForceMap.at<cv::Point2d>(nFMapRow, nFMapCol) * 1000;
				if (GetVectorLength(oForceVector) < 50)
					oForceVector = oForceVector / (GetVectorLength(oForceVector) == 0 ? 1 : GetVectorLength(oForceVector)) * 50;
				else if (GetVectorLength(oForceVector) > 150)
					oForceVector = oForceVector / (GetVectorLength(oForceVector) == 0 ? 1 : GetVectorLength(oForceVector)) * 150;
				cv::Scalar color = cv::Scalar(0,0,255);
				cv::Point oCoordinate(nDMatCol, nDMatRow);
				arrowedLine(oDrawMat, oCoordinate, oCoordinate + cv::Point(oForceVector), color, 5, cv::LINE_AA);
			}
		}
		namedWindow("ForceMap", cv::WINDOW_NORMAL);
		imshow("ForceMap", oDrawMat);
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
	oForceMapGenerator.CreateVectorFields();

	std::string sMapPath = ros::package::getPath(PACKAGE_NAME) + "/mapTables/";
	oForceMapGenerator.SaveDistanceMapToFile(sMapPath + "distancemap.xml");
	oForceMapGenerator.SaveForceMapToFile(sMapPath + "forcemap.xml");
}

int main(int argc, char **argv)
{
	//CreateMaps();

	std::string sMapPath = ros::package::getPath(PACKAGE_NAME) + "/mapTables/";
	CForceMapGenerator::DrawDistanceMap(sMapPath + "distancemap.xml");
	CForceMapGenerator::DrawForceMap(sMapPath + "forcemap.xml");

	return(0);
}
