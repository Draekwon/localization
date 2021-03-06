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
const bool bIsChessboard = true;


class CCalibrateOmniCamera
{
private:

	ros::NodeHandle m_oNodeHandle;
	image_transport::ImageTransport m_oImgTransport;
	image_transport::Subscriber m_oImageSub;

	const std::string m_sWindowName = "Calibration";

	std::string								m_sOutputFilename;
	std::vector<std::vector<cv::Point2f>> 	m_aImagePoints;
	std::vector<std::vector<cv::Point3f>> 	m_aObjectPoints;
	cv::Size								m_oBoardSize;
	cv::Size								m_oImageSize;

	bool 	m_bIsChessboard;
	bool*	m_pButtonClicked;

public:

	CCalibrateOmniCamera(std::string sOutputFilename, std::vector<std::vector<cv::Point3f>>& aObjectPoints, cv::Size& oBoardSize,
			std::string sImageTopic, bool bIsChessboard)
	: m_oImgTransport(m_oNodeHandle), m_pButtonClicked(new bool(false)), m_bIsChessboard(bIsChessboard)
	{
		m_sOutputFilename	= sOutputFilename;
		m_aObjectPoints 	= aObjectPoints;
		m_oBoardSize 		= oBoardSize;

		cv::namedWindow(m_sWindowName, cv::WINDOW_GUI_EXPANDED);
		cv::createButton("Calibrate now.", CCalibrateOmniCamera::CalibrateButton, m_pButtonClicked, cv::QT_PUSH_BUTTON);
		cv::displayStatusBar(m_sWindowName, "0 valid frames received.");

		m_oImageSub = m_oImgTransport.subscribe(sImageTopic, 5,
				  &CCalibrateOmniCamera::ImageCallback, this, image_transport::TransportHints("compressed"));
	}

	~CCalibrateOmniCamera()
	{
		m_oImageSub.shutdown();
		cv::destroyWindow(m_sWindowName);
		delete m_pButtonClicked;
	}

private:

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
		cv::Mat oGrayImg;
		cv::cvtColor(pCvImg->image, oGrayImg, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> aPoints;
        bool found;
        if (m_bIsChessboard)
        	found = findChessboardCorners(oGrayImg, m_oBoardSize, aPoints, cv::CALIB_CB_ADAPTIVE_THRESH);
        else
        	found = findCirclesGrid(oGrayImg, m_oBoardSize, aPoints, cv::CALIB_CB_ASYMMETRIC_GRID);
        if (found)
        {
            m_aImagePoints.push_back(aPoints);
            std::ostringstream strs;
            strs << m_aImagePoints.size() << " valid frames received.";
    		cv::displayStatusBar(m_sWindowName, strs.str());
        }

        m_oImageSize = oGrayImg.size();

		if (*m_pButtonClicked)
		{
			CalibrateAndSave();
		}

        cv::imshow(m_sWindowName, pCvImg->image);
        cv::waitKey(1);
	}

	void CalibrateAndSave()
	{
		*m_pButtonClicked = false;

		if (m_aImagePoints.size() < 3)
		{
	        std::cout << "Not enough corner detected images" << std::endl;
	        return;
		}

		m_aObjectPoints.resize(m_aImagePoints.size(), m_aObjectPoints[0]);

		// run calibration, some images are discarded in calibration process because they are failed
		// in initialization. Retained image indexes are in idx variable.
		int flags = 0;
		cv::Mat K, D, xi, idx;
		std::vector<cv::Vec3d> rvecs, tvecs;
		double rms;
		cv::TermCriteria criteria(3, 200, 1e-8);
		rms = cv::omnidir::calibrate(m_aObjectPoints, m_aImagePoints, m_oImageSize, K, xi, D, rvecs, tvecs, flags, criteria, idx);
		std::cout << "rms: " << rms << std::endl;
		std::cout << "Saving camera params to " << m_sOutputFilename << std::endl;
		saveCameraParams(m_sOutputFilename, K, D, xi,
			rvecs, tvecs, rms);

		std::ostringstream strs;
		strs << "calibration error: " << rms;
		cv::displayOverlay(m_sWindowName, strs.str(), 5000);
	}

	/**
	 * no idea why this has to be static, bad design IMO.
	 * This is obviously the Callback for pressing the Calibrate Button
	 * @param state			The state of the button. Apparently always -1 for a QPushButton
	 * @param userData		The userData. In this case a pointer to a bool value.
	 */
	static void CalibrateButton(int state, void* userData)
	{
		bool* bButtonClicked = (bool*)userData;
		*bButtonClicked = true;
	}

	static void saveCameraParams( const std::string & filename, const cv::Mat& cameraMatrix,
	    const cv::Mat& distCoeffs, const cv::Mat& xi, const std::vector<cv::Vec3d>& rvecs, const std::vector<cv::Vec3d>& tvecs,
		const double rms)
	{
	    cv::FileStorage fs( filename, cv::FileStorage::WRITE );

	    time_t tt;
	    time( &tt );
	    struct tm *t2 = localtime( &tt );
	    char buf[1024];
	    strftime( buf, sizeof(buf)-1, "%c", t2 );

	    fs << "calibration_time" << buf;
	    fs << "rms" << rms;
	    fs << "camera_matrix" << cameraMatrix;
	    fs << "distortion_coefficients" << distCoeffs;
	    fs << "xi" << xi;

	    if ( !rvecs.empty() && !tvecs.empty() )
	    {
	        cv::Mat rvec_tvec((int)rvecs.size(), 6, CV_64F);
	        for (int i = 0; i < (int)rvecs.size(); ++i)
	        {
	            cv::Mat(rvecs[i]).reshape(1, 1).copyTo(rvec_tvec(cv::Rect(0, i, 3, 1)));
	            cv::Mat(tvecs[i]).reshape(1, 1).copyTo(rvec_tvec(cv::Rect(3, i, 3, 1)));
	        }
	        //cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
	        fs << "extrinsic_parameters" << rvec_tvec;
	    }
	}

};

static void calcChessBoardCorners(const cv::Size &boardSize, const cv::Size2f &squareSize, std::vector<cv::Point3f>& corners)
{
    corners.clear();
	for (int nHeight = 0; nHeight < boardSize.height; ++nHeight)
	{
		for (int nWidth = 0; nWidth < boardSize.width; ++nWidth)
		{
        	// every second point is indented by half the width
        	float dCurrentWidth = nWidth * squareSize.width;
        	float dCurrentHeight = nHeight * squareSize.height;
        	corners.push_back(cv::Point3f(dCurrentWidth, dCurrentHeight, 0.0));
        }
    }
}

/**
 *
 * @param boardSize			size of the async circle grid
 * @param circleDistance	horizontal and vertical distance between circles
 * @param corners			output matrix of corner points
 */
static void calcAsyncCircleCorners(const cv::Size &boardSize, const cv::Size2f &circleDistance, std::vector<cv::Point3f>& corners)
{
    corners.clear();
	for (int nHeight = 0; nHeight < boardSize.height; ++nHeight)
	{
		for (int nWidth = 0; nWidth < boardSize.width; ++nWidth)
		{
        	// every second point is indented by half the width
        	float dCurrentWidth = nWidth * circleDistance.width + (nHeight % 2 * circleDistance.width / 2.0);
        	float dCurrentHeight = nHeight * circleDistance.height / 2.0;
        	corners.push_back(cv::Point3f(dCurrentWidth, dCurrentHeight, 0.0));
        }
    }
}

int main(int argc, char** argv)
{
	// parse command line arguments
    cv::CommandLineParser parser(argc, argv,
                                 "{w|8|board width}"
                                 "{h|6|board height}"
                                 "{sw|0.024|square width}"
                                 "{sh|0.024|square height}"
                                 "{o|out_camera_params.xml|output file}"
    							 "{t|/usb_cam/image_raw|ros image topic}"
                                 "{help||show help}"
                                 );
    parser.about("This is the omnidirectional camera calibration. Example command line:\n"
                 "    omni_calibration -w=4 -h=11 -sw=0.06 -sh=0.06 \n");//
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }

    std::string sTopicName = parser.get<std::string>("t");

    cv::Size boardSize(parser.get<int>("w"), parser.get<int>("h"));
    cv::Size2f squareSize(parser.get<float>("sw"), parser.get<float>("sh"));
    const std::string outputFilename = ros::package::getPath(PACKAGE_NAME) + "/config/" + parser.get<std::string>("o");

    if (!parser.check())
    {
        parser.printErrors();
        return -1;
    }

    // calculate object coordinates
    std::vector<std::vector<cv::Point3f>> objectPoints(1);
    if (bIsChessboard)
    	calcChessBoardCorners(boardSize, squareSize, objectPoints[0]);
    else
    	calcAsyncCircleCorners(boardSize, squareSize, objectPoints[0]);

    // start the live calibration
	ros::init(argc, argv, "live_calibration");
    CCalibrateOmniCamera oCalibrate(outputFilename, objectPoints, boardSize, sTopicName, bIsChessboard);
    ros::spin();
    return 0;

}
