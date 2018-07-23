#include <string>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "fisheye_camera_matrix/camera_matrix.hpp"

cv::Mat img;
cv::Mat img_ud;

// intrinsic
int width;
int height;
int cx;
int cy;

// extrinsic
int wz;
int nZRotation;
int nYRotation;
int nXRotation = 180;


int fl              = 320;
int fl_max          = 960;
int ceil_height     = 200;
int ceil_height_max = 350;
int scale           = 10;
int scale_max       = 30;

int wzMax 			= 200;

cv::Mat camMat;
cv::Mat distCoeffs;
std::vector<cv::Mat> rvecs;
std::vector<cv::Mat> tvecs;

std::vector<std::vector<cv::Point3f>> objectPoints = {
		{
		  cv::Point3f(915, 580, 0),
		  cv::Point3f(950, 560, 0),
		  cv::Point3f(920, 610, 0),
		  cv::Point3f(990, 600, 0),
		  cv::Point3f(910, 650, 0),
		  cv::Point3f(1015, 645, 0)
		}
};

std::vector<std::vector<cv::Point2f>> imagePoints = {
		{
		  cv::Point2f(235, 295),
		  cv::Point2f(265, 360),
		  cv::Point2f(215,310),
		  cv::Point2f(255, 370),
		  cv::Point2f(200, 315),
		  cv::Point2f(220, 385)
		}
};


void apply(int, void*)
{
  // convert degrees -> radians
  double dZRotation = nZRotation / 180.0 * M_PI;
  double dYRotation = dYRotation / 180.0 * M_PI;
  double dXRotation = dXRotation / 180.0 * M_PI;

  camMat = cv::Mat1d::eye(3, 3);
  camMat.at<double>(0, 0) = fl;
  camMat.at<double>(1, 1) = fl;
  camMat.at<double>(0, 2) = cx;
  camMat.at<double>(1, 2) = cy;

  cv::Size imageSize(img.cols, img.rows);

  cv::calibrateCamera(objectPoints, imagePoints, imageSize, camMat, distCoeffs, rvecs, tvecs, CV_CALIB_CB_NORMALIZE_IMAGE);

  cv::undistort(img, img_ud, camMat, distCoeffs);

  std::cout << width << " " << height << " " << cx << " " << cy << " "
	      << fl << " " << (ceil_height / 100.0) << " " << (scale / 10.0) << std::endl;

  cv::imshow("undistorted", img_ud);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "calibration");

  if(argc < 3) {
    ROS_ERROR("Please use roslaunch: 'roslaunch fisheye_camera_matrix calibrate_offline_manual.launch "
              "[img:=FILE] [calib:=FILE]'");
    return 1;
  }

  std::string path_img   = ros::package::getPath("fisheye_camera_matrix") + std::string("/../../../captures/") + std::string(argv[1]);
  std::string path_calib = ros::package::getPath("fisheye_camera_matrix") + std::string("/config/") + std::string(argv[2]);

  if(access(path_img.c_str(), R_OK ) == -1) {
    ROS_ERROR("No such file: %s\nPlease give a path relative to catkin_ws/../captures/", path_img.c_str() );
    return 1;
  }

  ROS_INFO("calibration: using imagefile %s", path_img.c_str() );
  ROS_INFO("calibration: using calibrationfile: %s", path_calib.c_str() );

  img    = cv::imread(path_img);
  width  = img.cols;
  height = img.rows;
  cx     = width / 2;
  cy     = height / 2;


  img.copyTo(img_ud);
  cv::namedWindow("undistorted", CV_WINDOW_NORMAL);
  cv::createTrackbar("focal length", "undistorted", &fl, fl_max, apply);
  cv::createTrackbar("z distance", "undistorted", &wz, wzMax, apply);


  apply(0, (void*)0);
  cv::waitKey(0);

  std::ofstream file;
  file.open(path_calib.c_str(), std::ios::out | std::ios::trunc);

  file << width << " " << height << " " << cx << " " << cy << " "
      << fl << " " << (ceil_height / 100.0) << " " << (scale / 10.0) << std::endl;

  file.close();
  ROS_INFO("Calibration written to %s.", path_calib.c_str() );

  return 0;
}
