// c++
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <time.h>
// opencv
#include "opencv2/ccalib/omnidir.hpp"
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


const std::string PACKAGE_NAME = "omni_camera";


class COmniCameraUndistortion
{

	ros::NodeHandle m_oNodeHandle;
	image_transport::ImageTransport m_oImgTransport;
	image_transport::Subscriber m_oImageSub;
	image_transport::Publisher m_oImagePub;

	const cv::Mat m_oCameraMatrix, m_oDistCoeffs, m_oXi;

public:

	COmniCameraUndistortion(const std::string& sInputTopic, const std::string& sOutputTopic, const cv::Mat& oCameraMatrix, const cv::Mat& oDistCoeffs, const cv::Mat& oXi)
	: m_oImgTransport(m_oNodeHandle), m_oXi(oXi), m_oCameraMatrix(oCameraMatrix), m_oDistCoeffs(oDistCoeffs)
	{
		cv::namedWindow("window", cv::WINDOW_NORMAL);

		m_oImagePub = m_oImgTransport.advertise(sOutputTopic, 1);
		m_oImageSub = m_oImgTransport.subscribe(sInputTopic, 1,
				  &COmniCameraUndistortion::ImageCallback, this, image_transport::TransportHints("compressed"));
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

		cv::Mat oUndistortedImg;

		cv::omnidir::undistortImage(pCvImg->image, oUndistortedImg, m_oCameraMatrix, m_oDistCoeffs, m_oXi, cv::omnidir::RECTIFY_PERSPECTIVE);

		sensor_msgs::ImagePtr oPubMsg = cv_bridge::CvImage(std_msgs::Header(),
				sensor_msgs::image_encodings::BGR8, oUndistortedImg).toImageMsg();
		// Output modified video stream
		m_oImagePub.publish(oPubMsg);

		cv::imshow("window", oUndistortedImg);
		cv::waitKey(1);
	}
};



static bool ReadConfigFile(const std::string& sConfigFile, cv::Mat& cameraMatrix,
	    cv::Mat& distCoeffs, cv::Mat& xi)
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
    oFilestorage["xi"] >> xi;

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
								 "{ti|/JaRen/usb_cam/image_raw|ros image topic}"
			 	 	 	 	 	 "{to|/omni_undistorted|undistorted image topic}"
                                 "{help||show help}"
                                 );
    parser.about("This is a sample for omnidirectional camera calibration. Example command line:\n"
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

    cv::Mat cameraMatrix, distCoeffs, xi;

    if (!ReadConfigFile(sConfigFile, cameraMatrix, distCoeffs, xi))
    	return -1;

    // start the live calibration
	ros::init(argc, argv, "omni_undistortion");
	COmniCameraUndistortion oCalibrate(sTopicIn, sTopicOut, cameraMatrix, distCoeffs, xi);
    ros::spin();
    return 0;


    return 0;
}
