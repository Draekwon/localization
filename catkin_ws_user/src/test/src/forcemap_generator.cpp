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

#include "utility.h"

using namespace cv;
using namespace std;

const double SCALE_TO_PX = 1114.0 * 100 / 600.0;
const double SCALE_TO_M = 600.0 / (1114.0 * 100);
const Size MAP_SIZE = Size(80 - 1, 120 - 1);

/**
 * generates a force map like in the paper:
 * http://page.mi.fu-berlin.de/rojas/2003/matrix.pdf
 * (p.5)
 */
class CForceMapGenerator
{
private:
	// class vars
	Mat m_oImage;
	Mat2d m_oForceMap;
	Mat2d m_oDistanceMap;
	const double EXP_CONST = 10;


	// private methods

	void CreateForceMap()
	{
		// border for map border
		Rect border(Point(0,0), m_oImage.size());
		rectangle(m_oImage, border, Scalar(255));

		// make bigger mat and put map in the center of it
		Mat oWorkingMat = Mat::zeros(m_oImage.size() * 2, CV_8UC1);
		Rect roi(m_oImage.cols / 2, m_oImage.rows / 2, m_oImage.cols, m_oImage.rows);
		m_oImage.copyTo(oWorkingMat(roi));

		int nStartingOffset = 0.1/*m*/ * SCALE_TO_PX / 2;

		// create empty forcemap and distancemap
		m_oForceMap = Mat2d::zeros(MAP_SIZE);
		m_oDistanceMap = Mat2d::zeros(MAP_SIZE);

		// get the contours
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours( oWorkingMat, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0) );
		cout << "contour count: " << contours.size() << endl;
		// create empty weight map
		int aSizes[] = {MAP_SIZE.width, MAP_SIZE.height, contours.size()};
		Mat oWeightMap(3, aSizes, CV_64FC1, Scalar(0));

		// the following for loops calculate the weights as in the paper on p.5
		for (int y = 0; y < MAP_SIZE.height; y++)
		{
			// nRow, nCol are pixel coordinate on scaled WorkingMat
			int nRow = nStartingOffset + (int)(0.1 * SCALE_TO_PX * y);
			for (int x = 0; x < MAP_SIZE.width; x++)
			{
				int nCol = nStartingOffset + (int)(0.1 * SCALE_TO_PX * x);

				// get distances to every contour and sum them up
				for (int nContOuter = 0; nContOuter < contours.size(); nContOuter++)
				{
					double fDividentSum = 0;
					for (int nCont = 0; nCont < contours.size(); nCont++)
					{
						// pixel distance
						double fDistance = fabs(pointPolygonTest(contours[nCont], Point2d(nCol, nRow), true));
						fDividentSum += exp(-fDistance * SCALE_TO_M / EXP_CONST);
					}
					double fDistanceOuter = fabs(pointPolygonTest(contours[nContOuter], Point2d(nCol, nRow), true));
					double fWeight = exp(fDistanceOuter * SCALE_TO_M / EXP_CONST) / fDividentSum;
					oWeightMap.at<double>(x, y, nContOuter) = fWeight;
				}
			}
			cout << "(nRow): (" << nRow << ")" << endl;
		}

		// these loops calculate the actual force vectors as in the paper p.5
		for (int y = 0; y < MAP_SIZE.height; y++)
		{
			// nRow, nCol are pixel coordinate on scaled WorkingMat
			int nRow = nStartingOffset + (int)(0.1 * SCALE_TO_PX * y);
			for (int x = 0; x < MAP_SIZE.width; x++)
			{
				int nCol = nStartingOffset + (int)(0.1 * SCALE_TO_PX * x);

				// initialize distancemap with too high value for the comparison later
				// this is kind of a hack :-/
				m_oDistanceMap[x][y] = Point2d(20000,20000);
				Point2d oForceVector(0,0);
				// brute-force-find the closest point / shortest vector of a contour:
				// iterate through contours
				for (int nCont = 0; nCont < contours.size(); nCont++)
				{
					// initialize on needlessly huge number
					double fMinDistance = 12345;
					Point2d oMinVector(0,0);

					// iterate through pixels in the contour
					for (int nPixelCount = 0; nPixelCount < contours[nCont].size(); nPixelCount++)
					{
						Point_<int> keks;
						double fDist = GetDistance(contours[nCont][nPixelCount], Point(nRow, nCol));
						if (fDist < fMinDistance)
						{
							fMinDistance = fDist;
							oMinVector = Point2d(contours[nCont][nPixelCount] - Point(nRow, nCol)) *  SCALE_TO_M;
							oMinVector = Point2d(-oMinVector.y, -oMinVector.x);
						}
					}
					if (GetVectorLength(oMinVector) < GetVectorLength(m_oDistanceMap[x][y]))
						m_oDistanceMap[x][y] = oMinVector;
					oForceVector += oWeightMap.at<double>(x , y, nCont) * oMinVector ;
				}

				m_oForceMap[x][y] = oForceVector;
				cout << "oForceVector: " << m_oForceMap[x][y] << endl;
				cout << "oDistanceVector: " << m_oDistanceMap[x][y] << endl;
			}
		}
	}

public:

	// constructor and callbacks

	CForceMapGenerator(Mat oImg)
	{
		oImg.convertTo(m_oImage, CV_8U);
		CreateForceMap();
	}

	Mat GetForceMap()
	{
		return m_oForceMap;
	}

	Mat GetDistanceMap()
	{
		return m_oDistanceMap;
	}

	static void SaveDistanceMapToFile(String sFilename, Mat oDistanceMap)
	{
		if (oDistanceMap.empty())
		{
			cerr << "No DistanceMap supplied ..." << endl;
			return;
		}
		FileStorage file(sFilename, FileStorage::WRITE, "UTF-8");
		file << "DistanceMap" << oDistanceMap;
		file.release();
	}

	static void SaveForceMapToFile(String sFilename, Mat oForceMap)
	{
		if (oForceMap.empty())
		{
			cerr << "No ForceMap supplied ..." << endl;
			return;
		}
		FileStorage file(sFilename, FileStorage::WRITE, "UTF-8");
		file << "ForceMap" << oForceMap;
		file.release();
	}

};



int main(int argc, char **argv)
{
	String imageName("../../../captures/Lab_map_600x400_scaled.png");
	Mat src = imread(imageName,IMREAD_GRAYSCALE);
	if (src.empty())
	{
		cerr << "No image supplied ..." << endl;
		return -1;
	}
	CForceMapGenerator oForceMapGenerator(src);

	CForceMapGenerator::SaveDistanceMapToFile("../../../distancemap.xml", oForceMapGenerator.GetDistanceMap());
	CForceMapGenerator::SaveForceMapToFile("../../../forcemap.xml", oForceMapGenerator.GetForceMap());

	return(0);
}
