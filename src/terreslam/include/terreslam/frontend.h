/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-18 11:30:37
 *    Last Modified: 2021-02-24 12:03:46
 */

#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "terreslam/features/plane_detector.h"
#include "terreslam/visualizer.h"

namespace terreslam
{

class Frontend : public nodelet::Nodelet
{
public:
	Frontend();

protected:
	// parameters
	std::string frame_id_;
	std::string odom_frame_id_;
	std::string cloud_frame_id_;
	std::string sub_lidar_frame_id_;
	std::string sub_cam_frame_id_;
	std::string sub_cam_depth_frame_id_;
	std::string sub_cam_info_frame_id_;
	ros::Publisher odom_pub_;
	ros::Publisher cloud_pub_;

	// blocks
	PlaneDetector PD_;

	// pcl_viewer
	Visualizer Vis_;

private:
	virtual void onInit();
	virtual void onFrontendInit() = 0;
	
};

}