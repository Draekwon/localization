#include <string>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/core.hpp>
#include <opencv2/ccalib/omnidir.hpp>
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

std::vector<cv::Mat> objectPoints;

std::vector<cv::Mat> imagePoints;


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

  cv::Mat K, xi, D, idx;
  int flags = 0;
  cv::TermCriteria critia(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 0.0001);
  std::vector<cv::Mat> rvecs, tvecs;
  std::cout << "((cv::InputArrayOfArrays)objectPoints).type() " << ((cv::InputArrayOfArrays)objectPoints).type() << std::endl;
  std::cout << "(((6) & ((1 << 3) - 1)) + (((3)-1) << 3))): " << ((((6) & ((1 << 3) - 1)) + (((3)-1) << 3))) << std::endl;
  std::cout << "before calib " << (!((cv::InputArrayOfArrays)objectPoints).empty() && (((cv::InputArrayOfArrays)objectPoints).type() == CV_64FC3)) << std::endl;
  CV_Assert(!((cv::InputArrayOfArrays)objectPoints).empty() && (((cv::InputArrayOfArrays)objectPoints).type() == CV_64FC3));
  std::cout << " after assert " << std::endl;
  double rms = cv::omnidir::calibrate(objectPoints, imagePoints, imageSize, K, xi, D, rvecs, tvecs, flags, critia, idx);
  std::cout << "after calib" << std::endl;
  cv::omnidir::undistortImage(img, img_ud, K, D, xi, cv::omnidir::RECTIFY_PERSPECTIVE);

  std::cout << width << " " << height << " " << cx << " " << cy << " "
	      << fl << " " << (ceil_height / 100.0) << " " << (scale / 10.0) << std::endl;

  cv::imshow("undistorted", img_ud);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "calibration");

  std::cout << argc << std::endl;
  std::cout << argv[0] << " " << argv [argc-1] << std::endl;

  if(argc < 3) {
    ROS_ERROR("Please use roslaunch: 'roslaunch fisheye_camera_matrix calibrate_offline_manual.launch "
              "[img:=FILE] [calib:=FILE]'");
    return 1;
  }
  std::cout << ros::package::getPath("fisheye_camera_matrix") << std::endl;
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

  cv::Mat* objMat = new cv::Mat(6,1,CV_64FC3);
  objMat->at<cv::Point3f>(0,0) = cv::Point3f(915, 580, 0);
  objMat->at<cv::Point3f>(1,0) = cv::Point3f(950, 560, 0);
  objMat->at<cv::Point3f>(2,0) = cv::Point3f(920, 610, 0);
  objMat->at<cv::Point3f>(3,0) = cv::Point3f(990, 600, 0);
  objMat->at<cv::Point3f>(4,0) = cv::Point3f(910, 650, 0);
  objMat->at<cv::Point3f>(5,0) = cv::Point3f(1015, 645, 0);
  objectPoints.push_back(*objMat);

  cv::Mat* imgMat = new cv::Mat(6,1,CV_64FC2);
  imgMat->at<cv::Point2f>(0,0) = cv::Point2f(235, 295);
  imgMat->at<cv::Point2f>(1,0) = cv::Point2f(265, 360);
  imgMat->at<cv::Point2f>(2,0) = cv::Point2f(215, 310);
  imgMat->at<cv::Point2f>(3,0) = cv::Point2f(255, 370);
  imgMat->at<cv::Point2f>(4,0) = cv::Point2f(200, 315);
  imgMat->at<cv::Point2f>(5,0) = cv::Point2f(220, 385);
  imagePoints.push_back(*imgMat);

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
