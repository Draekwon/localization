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

		m_oImagePub = m_oImgTransport.advertise(sOutputTopic, 1);
		m_oImageSub = m_oImgTransport.subscribe(sInputTopic, 1,
				  &CForwardCameraUndistortion::ImageCallback, this, image_transport::TransportHints("compressed"));
	}

private:

	int nCounter = 0;

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

		cv::Mat oHsvImg;
		cvtColor(pCvImg->image, oHsvImg, CV_BGR2YUV);
		cv::Mat oRangedImg;
		cv::inRange(oHsvImg, cv::Scalar(155, 0, 0), cv::Scalar(245, 255, 255), oRangedImg);
		pCvImg->image = oRangedImg;

		if (m_bWithUndistortion)
		{
			cv::Mat oUndistortedImg;
			cv::undistort(pCvImg->image, oUndistortedImg, m_oCameraMatrix, m_oDistCoeffs);
			pCvImg->image = oUndistortedImg;
		}

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

		cv::warpPerspective(pCvImg->image, mRectifiedImg, mPerspectiveTransform, pCvImg->image.size());
		pCvImg->image = mRectifiedImg;
/*
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(pCvImg->image, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
		for( size_t i = 0; i< contours.size(); i++ )
		{
			double a = contourArea(contours[i]);
			if (a > 400)
			{
				cv::fillConvexPoly(pCvImg->image, contours[i], 0);
			}
			std::cout << "contour " << i << " size: " << a << std::endl;
		}
		std::cout << std::endl;

*/
		sensor_msgs::ImagePtr oPubMsg = cv_bridge::CvImage(std_msgs::Header(),
				sensor_msgs::image_encodings::MONO8, pCvImg->image).toImageMsg();
		// Output modified video stream
		m_oImagePub.publish(oPubMsg);

		cv::imshow("window", pCvImg->image);
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

    if (!ReadConfigFile(sConfigFile, cameraMatrix, distCoeffs))
    	return -1;
    CForwardCameraUndistortion oCalibrate(sTopicIn, sTopicOut);
	//CForwardCameraUndistortion oCalibrate(sTopicIn, sTopicOut, cameraMatrix, distCoeffs);
    ros::spin();

    return 0;
}
