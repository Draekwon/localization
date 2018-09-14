// c++
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <time.h>
// opencv
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
// ros
#include "ros/ros.h"
#include "ros/package.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"


const std::string PACKAGE_NAME = "forward_camera";


class CForwardCameraUndistortion
{

	ros::NodeHandle m_oNodeHandle;
	image_transport::ImageTransport m_oImgTransport;
	image_transport::Subscriber m_oImageSub;
	image_transport::Publisher m_oImagePub;

	bool m_bWithUndistortion	= true;

	const cv::Mat m_oCameraMatrix, m_oDistCoeffs;

public:

	CForwardCameraUndistortion(const std::string& sInputTopic, const std::string& sOutputTopic, const cv::Mat& oCameraMatrix, const cv::Mat& oDistCoeffs)
	: m_oImgTransport(m_oNodeHandle),  m_oCameraMatrix(oCameraMatrix), m_oDistCoeffs(oDistCoeffs)
	{
//		cv::namedWindow("window", cv::WINDOW_NORMAL);

		m_oImagePub = m_oImgTransport.advertise(sOutputTopic, 1);
		m_oImageSub = m_oImgTransport.subscribe(sInputTopic, 1,
				  &CForwardCameraUndistortion::ImageCallback, this, image_transport::TransportHints("compressed"));
	}

	CForwardCameraUndistortion(const std::string& sInputTopic, const std::string& sOutputTopic)
	: m_oImgTransport(m_oNodeHandle), m_bWithUndistortion(false)
	{
		cv::namedWindow("window", cv::WINDOW_NORMAL);
		cv::namedWindow("masked", cv::WINDOW_NORMAL);
//		cv::namedWindow("adaptive", cv::WINDOW_NORMAL);
//		cv::namedWindow("hsv", cv::WINDOW_NORMAL);
//		cv::namedWindow("bitwise", cv::WINDOW_NORMAL);

		m_oImagePub = m_oImgTransport.advertise(sOutputTopic, 1);
		m_oImageSub = m_oImgTransport.subscribe(sInputTopic, 1,
				  &CForwardCameraUndistortion::ImageCallback, this, image_transport::TransportHints("compressed"));
	}

private:

	//! does not work yet
	void FilterGreenLines(cv::Mat& oImg)
	{
		cv::Mat oHsvImg(oImg.size(), CV_8UC3);
		cvtColor(oImg, oHsvImg, CV_BGR2HSV);

		/// green h: 60°-180° => 42.5 - 127.5

//		cv::Mat channels[3], oAdaptiveImg;
//		cv::split(oHsvImg, channels);
//		cv::adaptiveThreshold(channels[1], oAdaptiveImg, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 5, 9);
//		cv::dilate(oAdaptiveImg, oAdaptiveImg, cv::Mat(), cv::Point(-1,-1), 1);

		cv::Mat oRangedImg(oImg.size(), CV_8UC1);
		cv::inRange(oHsvImg, cv::Scalar(0, 100, 75), cv::Scalar(255, 255, 180), oRangedImg);
		cv::erode(oRangedImg, oRangedImg, cv::Mat(), cv::Point(-1, -1), 2);
		cv::dilate(oRangedImg, oRangedImg, cv::Mat(), cv::Point(-1,-1), 2);


//		cv::imshow("H", channels[0]);
//		cv::imshow("S", channels[1]);
//		cv::imshow("V", channels[2]);


//		cv::Mat oBitwiseImg;
//		cv::bitwise_and(oAdaptiveImg, oRangedImg, oBitwiseImg);
//
//
//		cv::imshow("adaptive", oAdaptiveImg);
//		cv::imshow("hsv", oRangedImg);
//		cv::imshow("bitwise", oBitwiseImg);


		// lines to top
//		for (int x = oImg.cols-1; x >= 0; x--)
//		{
//			for (int y = oImg.rows-1; y >= 0; y--)
//			{
//				if (oRangedImg.at<uchar>(y,x) > 0)
//				{
//					cv::line(oImg, cv::Point(x, 0), cv::Point(x, y), cv::Scalar(0,0,0), 1);
//					break;
//				}
//			}
//		}

		// lines from bottom center
		cv::Point oBottomCenter(oImg.cols / 2, oImg.rows);
		for (int nRow = oImg.rows -1; nRow >= 0; nRow--)
		{
			cv::Point oLeft(0, nRow);
			cv::Point oRight(oImg.cols - 1, nRow);

			cv::LineIterator oLeftIt(oRangedImg, oBottomCenter, oLeft);
			cv::LineIterator oRightIt(oRangedImg, oBottomCenter, oRight);

			for(int i = 0; i < oLeftIt.count; i++, ++oLeftIt)
			{
				if (oRangedImg.at<uchar>(oLeftIt.pos()) > 0)
				{
					cv::line(oImg, oLeftIt.pos(), oLeft, cv::Scalar(0,0,0), 2);
					break;
				}
			}
			for(int i = 0; i < oRightIt.count; i++, ++oRightIt)
			{
				if (oRangedImg.at<uchar>(oRightIt.pos()) > 0)
				{
					cv::line(oImg, oRightIt.pos(), oRight, cv::Scalar(0,0,0), 2);
					break;
				}
			}
		}
		for (int nCol = 0; nCol < oImg.cols; nCol++)
		{
			cv::Point oTop(nCol, 0);
			cv::LineIterator oTopIt(oRangedImg, oBottomCenter, oTop);
			for(int i = 0; i < oTopIt.count; i++, ++oTopIt)
			{
				if (oRangedImg.at<uchar>(oTopIt.pos()) > 0)
				{
					cv::line(oImg, oTopIt.pos(), oTop, cv::Scalar(0,0,0), 2);
					break;
				}
			}
		}

		cv::imshow("masked", oImg);
	}


	void FilterGreenAttemptTwo(cv::Mat& oImg)
	{
		cv::Mat oHsvImg(oImg.size(), CV_8UC3);
		cvtColor(oImg, oHsvImg, CV_BGR2HSV);
		/// green h: 60°-180° => 42.5 - 127.5
		cv::Mat oRangedHsv(oImg.size(), CV_8UC1);
		cv::inRange(oHsvImg, cv::Scalar(0, 75, 75), cv::Scalar(255, 255, 200), oRangedHsv);

//		cv::Mat oHslImg(oImg.size(), CV_8UC3);
//		cvtColor(oImg, oHslImg, CV_BGR2HLS);
//		cv::Mat oRangedHsl(oImg.size(), CV_8UC1);
//		cv::inRange(oHslImg, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 50), oRangedHsl);

		cv::Mat oHsvEdges;
		cv::Canny(oRangedHsv, oHsvEdges, 100, 100);
		cv::Mat oDilatedHsvEdges;
		cv::dilate(oHsvEdges, oDilatedHsvEdges, cv::Mat(), cv::Point(-1,-1), 1);
		oHsvEdges = oDilatedHsvEdges;

	    std::vector<cv::Vec4i> lines;
	    HoughLinesP( oHsvEdges, lines, 1, CV_PI/180, 200, 40, 10 );
	    for( size_t i = 0; i < lines.size(); i++ )
	    {
	        line( oImg, cv::Point(lines[i][0], lines[i][1]),
	            cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,255), 3, 8 );
	    }
//
//		std::vector<cv::Vec2f> lines;
//		cv::HoughLines( oHsvEdges, lines, 1, CV_PI/180, 250 );
//		for( size_t i = 0; i < lines.size(); i++ )
//		{
//			float rho = lines[i][0];
//			float theta = lines[i][1];
//			double a = cos(theta), b = sin(theta);
//			double x0 = a*rho, y0 = b*rho;
//			cv::Point pt1(cvRound(x0 + 1000*(-b)),
//					  cvRound(y0 + 1000*(a)));
//			cv::Point pt2(cvRound(x0 - 1000*(-b)),
//					  cvRound(y0 - 1000*(a)));
//			cv::line( oImg, pt1, pt2, cv::Scalar(0,0,255), 3, 8 );
//		}
//
//		cv::Mat oHslEdges;
//		cv::Canny(oRangedHsl, oHslEdges, 100, 100);
//		cv::Mat oDilatedHslEdges;
//		cv::dilate(oHslEdges, oDilatedHslEdges, cv::Mat(), cv::Point(-1,-1), 3);
//		oHslEdges = oDilatedHslEdges;
//
//		cv::Mat oMask;
//		cv::bitwise_and(oHsvEdges, oHslEdges, oMask);
//
//		cv::Mat oWhiteImg(oImg.size(), CV_8UC3, cv::Scalar(255,255,255));
//		oWhiteImg.copyTo(oImg, oHsvEdges/*oMask*/);

//		cv::Mat channels[3];
//		cv::split(oHslImg, channels);
//		cv::imshow("H", channels[0]);
//		cv::imshow("L", channels[1]);
//		cv::imshow("S", channels[2]);

		cv::imshow("hsv", oRangedHsv);
//		cv::imshow("hsl", oRangedHsl);
		cv::imshow("hsvedges", oHsvEdges);
//		cv::imshow("hsledges", oHslEdges);
		cv::imshow("masked", oImg);
		cv::waitKey(1);
	}

	void ApplyThresholding(cv::Mat& oImg)
	{
		cv::Mat oHlsImg;
		cvtColor(oImg, oHlsImg, CV_BGR2HLS);

//		cv::Mat channels[3];
//		cv::split(oHlsImg, channels);
//		cv::adaptiveThreshold(channels[1], oImg, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 3, 8);

		cv::inRange(oHlsImg, cv::Scalar(0, 175, 0), cv::Scalar(255, 255, 255), oImg);

		cv::dilate(oImg, oImg, cv::Mat(), cv::Point(-1, -1), 2);
		cv::erode(oImg, oImg, cv::Mat(), cv::Point(-1,-1), 1);
	}

	void WarpPerspective(cv::Mat& oImg)
	{
		cv::Point2f aImagePoints[4];
		cv::Point2f aObjectPoints[4];

		aImagePoints[0] = cv::Point2f(37, 351);
		aImagePoints[1] = cv::Point2f(603, 366);
		aImagePoints[2] = cv::Point2f(205, 272);
		aImagePoints[3] = cv::Point2f(429, 276);

		aObjectPoints[0] = cv::Point2f(218, 405);
		aObjectPoints[1] = cv::Point2f(422, 405);
		aObjectPoints[2] = cv::Point2f(218, 75);
		aObjectPoints[3] = cv::Point2f(422, 75);

		cv::Mat mPerspectiveTransform = cv::getPerspectiveTransform(aImagePoints, aObjectPoints);
		cv::Mat mRectifiedImg;

		cv::warpPerspective(oImg, mRectifiedImg, mPerspectiveTransform, oImg.size());
		oImg = mRectifiedImg;
	}

	void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr pCvImg;
		try
		{
		  pCvImg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}

		cv::Mat oImg;
		pCvImg->image.copyTo(oImg);

		WarpPerspective(oImg);
		FilterGreenLines(oImg);
		ApplyThresholding(oImg);


		if (m_bWithUndistortion)
		{
			cv::Mat oUndistortedImg;
			cv::undistort(oImg, oUndistortedImg, m_oCameraMatrix, m_oDistCoeffs);
			oImg = oUndistortedImg;
		}

		sensor_msgs::ImagePtr oPubMsg = cv_bridge::CvImage(std_msgs::Header(),
				sensor_msgs::image_encodings::MONO8, oImg).toImageMsg();
		// Output modified video stream
		m_oImagePub.publish(oPubMsg);

		cv::imshow("window", oImg);
		cv::waitKey(1);
	}
};



static bool ReadConfigFile(const std::string& sConfigFile, cv::Mat& cameraMatrix,
	    cv::Mat& distCoeffs)
{
	cv::FileStorage oFilestorage(sConfigFile, cv::FileStorage::READ);

    if (!oFilestorage.isOpened())
    {
      std::cerr << "failed to open " << sConfigFile << std::endl;
      return false;
    }

    double rms;
    oFilestorage["rms"] >> rms;
    oFilestorage["camera_matrix"] >> cameraMatrix;
    oFilestorage["distortion_coefficients"] >> distCoeffs;

    std::string buf;
    oFilestorage["calibration_time"] >> buf;

    std::cout << "timestamp of this calibration: " << buf << std::endl;
    std::cout << "error for this calibration: " << rms << std::endl;

    return true;
}


int main(int argc, char** argv)
{
	// parse command line arguments
    cv::CommandLineParser parser(argc, argv,
                                 "{i|out_camera_params.xml|input file}"
								 "{ti|/app/camera/rgb/image_rect_color|ros image topic}"
			 	 	 	 	 	 "{to|/forward_rectified|rectified image topic}"
                                 "{help||show help}"
                                 );
    parser.about("This is forward camera undistortion. Example command line:\n"
                 "    undistort_camera -i=out_camera_params.xml -ti=/usb_cam/image_raw -to=/omni_undistorted \n");//
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }

    const std::string sConfigFile = ros::package::getPath(PACKAGE_NAME) + "/config/" + parser.get<std::string>("i");
    const std::string sTopicIn = parser.get<std::string>("ti");
    const std::string sTopicOut = parser.get<std::string>("to");

    if (!parser.check())
    {
        parser.printErrors();
        return -1;
    }

    cv::Mat cameraMatrix, distCoeffs;

	ros::init(argc, argv, "forward_undistortion");

//    if (!ReadConfigFile(sConfigFile, cameraMatrix, distCoeffs))
//    	return -1;
    CForwardCameraUndistortion oCalibrate(sTopicIn, sTopicOut);
	//CForwardCameraUndistortion oCalibrate(sTopicIn, sTopicOut, cameraMatrix, distCoeffs);
    ros::spin();

    return 0;
}
