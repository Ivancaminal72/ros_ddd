/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-18 11:30:37
 */

#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "terreslam/features/plane_detector.h"
#include "terreslam/visualizer.h"
#include "terreslam/io_disk.h"

namespace terreslam
{

class Frontend : public nodelet::Nodelet
{
public:
	Frontend();

protected:
	// Parameters
	std::string frame_id;
	std::string odom_frame_id;
	std::string cloud_frame_id;
	std::string plane_frame_id;
	std::string sub_lidar_frame_id;
	std::string sub_cam_frame_id;
	std::string sub_cam_depth_frame_id;
	std::string sub_cam_info_frame_id;
	std::string logs_dir;
	double max_depth;

	bool PD_debug;
	int PD_theta;
	int PD_phi;
	int PD_d;
	int PD_max_plane;
	int PD_min_plane_size;
	double PD_thres_angle;
	double PD_thres_dist;
	// double PD_thres_color;
	
	/// Variables
	ros::Publisher odom_pub;
	ros::Publisher cloud_pub;
	ros::Publisher plane_pub;

	// blocks
	PlaneDetector *PD;

	// pcl_viewer
	// Visualizer Vis_;

	// read/write to disk
	IoDisk Disk;

private:
	virtual void onInit();
	virtual void onFrontendInit() = 0;
	
};

}