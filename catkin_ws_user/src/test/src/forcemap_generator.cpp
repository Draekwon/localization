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

using namespace cv;
using namespace std;

const double SCALE_TO_PX = 1114.0 / 600.0;
const double SCALE_TO_CM = 600.0 / 1114.0;

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
	const double EXP_CONST = 10 * SCALE_TO_PX * 100;


	// private methods

	void CreateForceMap()
	{
		// create empty forcemap
		m_oForceMap = Mat2d::zeros(m_oImage.size());
		m_oDistanceMap = Mat2d::zeros(m_oImage.size());

		// get the contours
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours( m_oImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0) );
		cout << "contour count: " << contours.size() << endl;
		// create empty weight map
		int aSizes[] = {m_oImage.cols, m_oImage.rows, contours.size()};
		Mat oWeightMap(3, aSizes, CV_64FC1, Scalar(0));

		// the following for loops calculate the weights as in the paper on p.5
		for (int nRow = 0; nRow < m_oImage.rows; nRow++)
		{
			for (int nCol = 0; nCol < m_oImage.cols; nCol++)
			{
				// get distances to every contour and sum them up
				for (int nContOuter = 0; nContOuter < contours.size(); nContOuter++)
				{
					double fDividentSum = 0;
					for (int nCont = 0; nCont < contours.size(); nCont++)
					{
						double fDistance = fabs(pointPolygonTest(contours[nCont], Point2d(nCol, nRow), true));
						fDividentSum += exp(-fDistance / EXP_CONST);
					}
					double fDistanceOuter = fabs(pointPolygonTest(contours[nContOuter], Point2d(nCol, nRow), true));
					double fWeight = exp(fDistanceOuter / EXP_CONST) / fDividentSum;
					oWeightMap.at<double>(nCol, nRow, nContOuter) = fWeight;
				}
			}
			cout << "(nRow): (" << nRow << ")" << endl;
		}

		// these loops calculate the actual force vectors as in the paper p.5
		for (int nRow = 0; nRow < m_oImage.rows; nRow++)
		{
			for (int nCol = 0; nCol < m_oImage.cols; nCol++)
			{

				m_oDistanceMap[nCol][nRow] = Point2d(20000,20000);
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
							oMinVector = Point2d(contours[nCont][nPixelCount] - Point(nRow, nCol));
						}
					}
					if (GetDistance(Point2d(0,0), oMinVector) < GetDistance(Point2d(0,0), m_oDistanceMap[nCol][nRow]))
						m_oDistanceMap[nCol][nRow] = oMinVector;
					oForceVector += oWeightMap.at<double>(nCol , nRow, nCont) * oMinVector ;
				}

				m_oForceMap[nCol][nRow] = oForceVector;
				cout << "oForceVector: " << m_oForceMap[nCol][nRow] << endl;
			}
		}
	}

	double GetDistance(Point oP1, Point oP2)
	{
		return sqrt(pow(oP1.x - oP2.x, 2) + pow(oP1.y - oP2.y, 2));
	}
	double GetDistance(Point2d oP1, Point2d oP2)
	{
		return sqrt(pow(oP1.x - oP2.x, 2) + pow(oP1.y - oP2.y, 2));
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
			cerr << "No ForceMap supplied ..." << endl;
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

	FileStorage file("../../../forcemap.xml", FileStorage::READ, "UTF-8");
	Mat oForceMap;
	file["ForceMap"] >> oForceMap;
	file.release();
	cout << "forcemap size " << oForceMap.size();

	return(0);
}