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


static bool detecAsincCircleCorners(const std::vector<std::string>& list, std::vector<std::string>& list_detected,
    std::vector<std::vector<cv::Point2f>>& imagePoints, cv::Size boardSize, cv::Size& imageSize)
{
    imagePoints.resize(0);
    list_detected.resize(0);
    int n_img = (int)list.size();
    cv::Mat img;
    for(int i = 0; i < n_img; ++i)
    {
        std::cout << list[i] << "... ";
        std::vector<cv::Point2f> points;
        img = imread(list[i], cv::IMREAD_GRAYSCALE);
        bool found = findCirclesGrid(img, boardSize, points, cv::CALIB_CB_ASYMMETRIC_GRID);
        if (found)
        {
            imagePoints.push_back(points);
            list_detected.push_back(list[i]);
        }
        std::cout << (found ? "FOUND" : "NO") << std::endl;
    }
    if (!img.empty())
        imageSize = img.size();
    if (imagePoints.size() < 3)
        return false;
    else
        return true;
}

static bool readStringList( const std::string& filename, std::vector<std::string>& l )
{
    std::ifstream infile(filename);
    std::string line;
    while (std::getline(infile, line))
    {
    	l.push_back(line);
    }

    return true;
}

static void saveCameraParams( const std::string & filename, int flags, const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs, const double xi, const std::vector<cv::Vec3d>& rvecs, const std::vector<cv::Vec3d>& tvecs,
    std::vector<std::string> detec_list, const cv::Mat& idx, const double rms, const std::vector<std::vector<cv::Point2f>>& imagePoints)
{
	// show undistorted images
	for (std::string sImage : detec_list)
	{
		cv::namedWindow(sImage, CV_WINDOW_NORMAL);
		cv::Mat distorted = cv::imread(sImage);
		cv::Mat undistorted;
		cv::omnidir::undistortImage(distorted, undistorted, cameraMatrix, distCoeffs, xi, cv::omnidir::RECTIFY_PERSPECTIVE);
		imshow(sImage, undistorted);
	}
	cv::waitKey(0);


    cv::FileStorage fs( filename, cv::FileStorage::WRITE );

    time_t tt;
    time( &tt );
    struct tm *t2 = localtime( &tt );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_time" << buf;

    if ( !rvecs.empty())
        fs << "nFrames" << (int)rvecs.size();

    if ( flags != 0)
    {
        sprintf( buf, "flags: %s%s%s%s%s%s%s%s%s",
            flags & cv::omnidir::CALIB_USE_GUESS ? "+use_intrinsic_guess" : "",
            flags & cv::omnidir::CALIB_FIX_SKEW ? "+fix_skew" : "",
            flags & cv::omnidir::CALIB_FIX_K1 ? "+fix_k1" : "",
            flags & cv::omnidir::CALIB_FIX_K2 ? "+fix_k2" : "",
            flags & cv::omnidir::CALIB_FIX_P1 ? "+fix_p1" : "",
            flags & cv::omnidir::CALIB_FIX_P2 ? "+fix_p2" : "",
            flags & cv::omnidir::CALIB_FIX_XI ? "+fix_xi" : "",
            flags & cv::omnidir::CALIB_FIX_GAMMA ? "+fix_gamma" : "",
            flags & cv::omnidir::CALIB_FIX_CENTER ? "+fix_center" : "");
        //cvWriteComment( *fs, buf, 0 );
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs << "xi" << xi;

    //cvWriteComment( *fs, "names of images that are acturally used in calibration", 0 );
    fs << "used_imgs" << "[";
    for (int i = 0;  i < (int)idx.total(); ++i)
    {
        fs << detec_list[(int)idx.at<int>(i)];
    }
    fs << "]";

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

    fs << "rms" << rms;

    if ( !imagePoints.empty() )
    {
        cv::Mat imageMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_64FC2);
        for (int i = 0; i < (int)imagePoints.size(); ++i)
        {
            cv::Mat r = imageMat.row(i).reshape(2, imageMat.cols);
            cv::Mat imagei(imagePoints[i]);
            imagei.copyTo(r);
        }
        fs << "image_points" << imageMat;
    }
}

int main(int argc, char** argv)
{
    cv::CommandLineParser parser(argc, argv,
                                 "{w|4|board width}"
                                 "{h|11|board height}"
                                 "{sw|0.06|square width}"
                                 "{sh|0.06|square height}"
                                 "{o|out_camera_params.xml|output file}"
                                 "{fs|false|fix skew}"
                                 "{fp|false|fix principal point at the center}"
                                 "{@input||input file - txt file with a list of the images}"
                                 "{help||show help}"
                                 );
    parser.about("This is a sample for omnidirectional camera calibration. Example command line:\n"
                 "    omni_calibration -w=6 -h=9 -sw=80 -sh=80 imagelist.xml \n");//
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }

    cv::Size boardSize(parser.get<int>("w"), parser.get<int>("h"));
    cv::Size2f squareSize(parser.get<float>("sw"), parser.get<float>("sh"));
    int flags = 0;
    if (parser.get<bool>("fs"))
        flags |= cv::omnidir::CALIB_FIX_SKEW;
    if (parser.get<bool>("fp"))
        flags |= cv::omnidir::CALIB_FIX_CENTER;
    const std::string outputFilename = parser.get<std::string>("o");
    const std::string inputFilename = parser.get<std::string>(0);

    if (!parser.check())
    {
        parser.printErrors();
        return -1;
    }

    // get image name list
    std::vector<std::string> image_list, detec_list;
    if(!readStringList(inputFilename, image_list))
    {
        std::cout << "Can not read imagelist" << std::endl;
        return -1;
    }

    // find corners in images
    // some images may be failed in automatic corner detection, passed cases are in detec_list
    std::cout << "Detecting chessboards (" << image_list.size() << ")" << std::endl;
    std::vector<std::vector<cv::Point2f>> imagePoints;
    cv::Size imageSize;
    if(!detecAsincCircleCorners(image_list, detec_list, imagePoints, boardSize, imageSize))
    {
        std::cout << "Not enough corner detected images" << std::endl;
        return -1;
    }

    // calculate object coordinates
    std::vector<std::vector<cv::Point3f>> objectPoints(1);
    calcAsyncCircleCorners(boardSize, squareSize, objectPoints[0]);
    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    // run calibration, some images are discarded in calibration process because they are failed
    // in initialization. Retained image indexes are in idx variable.
    cv::Mat K, D, xi, idx;
    std::vector<cv::Vec3d> rvecs, tvecs;
    double _xi, rms;
    cv::TermCriteria criteria(3, 200, 1e-8);
    std::cout << imagePoints.size() << " " << objectPoints.size() << " " << imageSize << std::endl;
//    cv::Mat camMat, coeffs;
//    rms = calibrateCamera(objectPoints, imagePoints, imageSize, camMat, coeffs, rvecs, tvecs);
    rms = cv::omnidir::calibrate(objectPoints, imagePoints, imageSize, K, xi, D, rvecs, tvecs, flags, criteria, idx);
    std::cout << "rms: " << rms << std::endl;
    _xi = xi.at<double>(0);
    std::cout << "Saving camera params to " << outputFilename << std::endl;
    saveCameraParams(outputFilename, flags, K, D, _xi,
        rvecs, tvecs, detec_list, idx, rms, imagePoints);

}
