/*
 * camera_undistort.cpp
 *
 *  Created on: Jun 26, 2018
 *      Author: sh
 */

// c++
#include <stdio.h>
#include <math.h>
#include <iostream>
//ros
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
// opencv
#include "opencv2/core.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgproc.hpp"


#include "utility.h"


class CCameraUndistort
{
	ros::NodeHandle m_oNodeHandle;
	image_transport::ImageTransport m_oImgTransport;
	image_transport::Subscriber m_oImageSub;
	image_transport::Publisher m_oImagePub;


public:

	CCameraUndistort()
	: m_oImgTransport(m_oNodeHandle)
	{
		// publish a position by publishing odometry
		m_oImagePub = m_oImgTransport.advertise("/usb_cam/undistorted", 5);

		// Subscribe to input video feed
		m_oImageSub = m_oImgTransport.subscribe("/usb_cam/image_raw", 5,
		  &CCameraUndistort::ImageCallback, this);
	}


private:

	void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr pCvImg;
		try
		{
		  pCvImg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8 /*BGR8*/);
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}




		sensor_msgs::ImagePtr oPubMsg = cv_bridge::CvImage(std_msgs::Header(),
				sensor_msgs::image_encodings::MONO8, pCvImg->image).toImageMsg();
		// Output modified video stream
		m_oImagePub.publish(oPubMsg);

	}

};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_undistort");
	CCameraUndistort oUndistort;
	ros::spin();
	return 0;
}
