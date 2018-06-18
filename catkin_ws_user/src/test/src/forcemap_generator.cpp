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

		// create empty forcemap and distancemap
		m_oForceMap = Mat2d::zeros(MAP_SIZE.height, MAP_SIZE.width);
		m_oDistanceMap = Mat2d::zeros(MAP_SIZE.height, MAP_SIZE.width);

		// get the contours
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours( oWorkingMat, contours, hierarchy, CV_RETR_LIST, CHAIN_APPROX_NONE, Point(0, 0) );

		// visualization
//	    Mat oContourImg = Mat::zeros( oWorkingMat.size(), CV_8UC1);
//		for( size_t i = 0; i< contours.size(); i++ )
//		{
//			Scalar color = Scalar( 255 );
//			drawContours( oContourImg, contours, (int)i, color, 1, 8, hierarchy, 0, Point() );
//		}
//		namedWindow("kek", WINDOW_NORMAL);
//		imshow("kek", oWorkingMat);
//		waitKey(0);
//		imshow("kek", oContourImg);
//		waitKey(0);
//		return;

		cout << "contour count: " << contours.size() << endl;
		// create empty weight map
		int aSizes[] = {MAP_SIZE.width, MAP_SIZE.height, contours.size()};
		Mat oWeightMap(3, aSizes, CV_64FC1, Scalar(0));

		int nStartingOffset = 4;
		// the following for loops calculate the weights as in the paper on p.5
//		for (int y = 0; y < MAP_SIZE.height; y++)
//		{
//			// nRow, nCol are pixel coordinate on scaled WorkingMat
//			int nRow = nStartingOffset + (int)(10 * y);
//			for (int x = 0; x < MAP_SIZE.width; x++)
//			{
//				int nCol = nStartingOffset + (int)(10 * x);
//
//				// get distances to every contour and sum them up
//				for (int nContOuter = 0; nContOuter < contours.size(); nContOuter++)
//				{
//					double fDividentSum = 0;
//					for (int nCont = 0; nCont < contours.size(); nCont++)
//					{
//						// pixel distance
//						double fDistance = fabs(pointPolygonTest(contours[nCont], Point2d(nCol, nRow), true));
//						fDividentSum += exp(-fDistance * SCALE_TO_M / EXP_CONST);
//					}
//					double fDistanceOuter = fabs(pointPolygonTest(contours[nContOuter], Point2d(nCol, nRow), true));
//					double fWeight = exp(fDistanceOuter * SCALE_TO_M / EXP_CONST) / fDividentSum;
//					oWeightMap.at<double>(y, x, nContOuter) = fWeight;
//				}
//			}
//			cout << "(nRow): (" << nRow << ")" << endl;
//		}

		// for visualization
//		namedWindow("DistanceMapDrawing", WINDOW_NORMAL);
//		RNG rng;
//		Mat oDrawMat = Mat::zeros(MAP_SIZE.height * 101, MAP_SIZE.width * 101, CV_8UC3);

		// these loops calculate the actual force vectors as in the paper p.5
		for (int y = 0; y < MAP_SIZE.height; y++)
		{
			// nRow, nCol are pixel coordinate on scaled WorkingMat
			int nRow = nStartingOffset + 10 * y;
			for (int x = 0; x < MAP_SIZE.width; x++)
			{
				int nCol = nStartingOffset + 10 * x;

				// initialize distancemap with too high value for the comparison later
				// this is kind of a hack :-/
				m_oDistanceMap.at<Point2d>(y, x) = Point2d(20000, 20000);
				Point2d oForceVector(0,0);
				// brute-force-find the closest point / shortest vector of a contour:
				// iterate through contours
				for (int nCont = 0; nCont < contours.size(); nCont++)
				{
					// initialize on needlessly huge number (is hack like above)
					double fMinDistance = 12345;
					Point2d oMinVector(0,0);

					// iterate through pixels in the contour
					for (int nPixelCount = 0; nPixelCount < contours[nCont].size(); nPixelCount++)
					{
						double fDist = GetDistance(contours[nCont][nPixelCount], Point(nCol, nRow));
						if (fDist < fMinDistance)
						{
							fMinDistance = fDist;
							oMinVector = Point2d(contours[nCont][nPixelCount] - Point(nCol, nRow)) *  SCALE_TO_M;
							oMinVector = Point2d(-oMinVector.y, -oMinVector.x);
						}
					}
					if (GetVectorLength(oMinVector) < GetVectorLength(m_oDistanceMap.at<Point2d>(y, x)))
					{
						m_oDistanceMap.at<Point2d>(y, x) = oMinVector;
					}
//					oForceVector += oWeightMap.at<double>(y, x, nCont) * oMinVector;
				}
//				m_oForceMap.at<Point2d>(y, x) = oForceVector;

				cout << "(x,y): " << x << "," << y << "; m_oDistanceMap.at<Point2d>(y, x) " << m_oDistanceMap.at<Point2d>(y, x) << endl;

				// visualization
//				Point oCoordinate(50 + x * 100, 50 + y * 100);
//				Point2d oDistVec = m_oDistanceMap[x][y] / (GetVectorLength(m_oDistanceMap[x][y]) == 0 ? 1 : GetVectorLength(m_oDistanceMap[x][y])) * 75;
//				Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
//				arrowedLine(oDrawMat, oCoordinate, oCoordinate+Point(oDistVec), color, 5, LINE_AA);
//				imshow("DistanceMapDrawing", oDrawMat);
//				waitKey(3);
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

void DrawForceMap()
{
	Mat oForceMap;
	cv::FileStorage fs("../../../forcemap.xml", cv::FileStorage::READ);
	fs["ForceMap"] >> oForceMap;

	RNG rng;

	Mat oDrawMat = Mat::zeros(oForceMap.rows * 101, oForceMap.cols * 101, CV_8UC3);
	cout << oDrawMat.size << endl;
	for (int nFMapCol = 0; nFMapCol < oForceMap.cols; nFMapCol++)
	{
		int nDMatCol = 50 + nFMapCol * 100;
		for (int nFMapRow = 0; nFMapRow < oForceMap.rows; nFMapRow++)
		{
			int nDMatRow = 50 + nFMapRow * 100;

			Point2d oForceVector = oForceMap.at<Point2d>(nFMapRow, nFMapCol);
			oForceVector = oForceVector / (GetVectorLength(oForceVector) == 0 ? 1 : GetVectorLength(oForceVector)) * 75;
			Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
			Point oCoordinate(nDMatCol, nDMatRow);
			arrowedLine(oDrawMat, oCoordinate, oCoordinate+Point(oForceVector), color, 5, LINE_AA);
		}
	}
	namedWindow("ForceMap", WINDOW_NORMAL);
	imshow("ForceMap", oDrawMat);
	waitKey(0);

}

void DrawDistanceMap()
{
	Mat oDistanceMap;
	cv::FileStorage fs2("../../../distancemap.xml", cv::FileStorage::READ);
	fs2["DistanceMap"] >> oDistanceMap;

	RNG rng;

	Mat oDrawMat = Mat::zeros(oDistanceMap.rows * 101, oDistanceMap.cols * 101, CV_8UC3);
	cout << oDrawMat.size << endl;
	for (int nDMapCol = 0; nDMapCol < oDistanceMap.cols; nDMapCol++)
	{
		int nDrawMatCol = 50 + nDMapCol * 100;
		for (int nDMapRow = 0; nDMapRow < oDistanceMap.rows; nDMapRow++)
		{
			int nDrawMatRow = 50 + nDMapRow * 100;

			Point2d oDistVector = oDistanceMap.at<Point2d>(nDMapRow, nDMapCol);
			oDistVector = oDistVector / (GetVectorLength(oDistVector) == 0 ? 1 : GetVectorLength(oDistVector)) * 75;
			Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
			Point oCoordinate(nDrawMatCol, nDrawMatRow);
			arrowedLine(oDrawMat, oCoordinate, oCoordinate+Point(oDistVector), color, 5, LINE_AA);
		}
	}
	namedWindow("DistanceMap", WINDOW_NORMAL);
	imshow("DistanceMap", oDrawMat);
	waitKey(0);
}

void CreateMaps()
{
	String imageName("../../../captures/Lab_map_600x400.png");
	Mat src = imread(imageName, IMREAD_GRAYSCALE);
	if (src.empty())
	{
		cerr << "No image supplied ..." << endl;
		exit(-1);
	}
	CForceMapGenerator oForceMapGenerator(src);

	CForceMapGenerator::SaveDistanceMapToFile("../../../distancemap.xml", oForceMapGenerator.GetDistanceMap());
//	CForceMapGenerator::SaveForceMapToFile("../../../forcemap.xml", oForceMapGenerator.GetForceMap());
}

int main(int argc, char **argv)
{
//	CreateMaps();

	DrawDistanceMap();

	return(0);
}
