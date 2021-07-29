/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-18 11:30:37
 */

#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <memory>

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
	// General frame_id
	std::string frame_id;
	std::string odom_frame_id;
	std::string cloud_frame_id;
	std::string cloud_xy_frame_id;
	std::string cloud_filtered_frame_id;
	std::string cloud_filtered_blobs_frame_id;
	std::string cloud_plane_frame_id;
	std::string sub_lidar_frame_id;
	std::string sub_cam_frame_id;
	std::string sub_cam_depth_frame_id;
	std::string sub_cam_info_frame_id;
	
	/// General parameters
	int entry_count = 0;
	std::string logs_dir;
	double max_depth;

	/// Constants
	const double depthScale = pow(2,16)/120;
	const float bad_point = std::numeric_limits<float>::quiet_NaN();

	/// Plane Filter
	bool use_normal_integral = false;
	float PF_thresh;
	bool PF_highpass;

	/// Plane Detector
	Scan* scan_;
	bool PD_debug;
	int PD_theta;
	int PD_phi;
	int PD_d;
	int PD_max_plane;
	int PD_min_plane_size;
	double PD_thres_angle;
	double PD_thres_dist;
	// double PD_thres_color;

	//Blob Detector
	float BD_tolerance;
	int BD_min_size;
	int BD_max_size;
	int BD_ppa;
	float BD_alpha;
	float BD_thres_xz;
	float BD_thres_radius;


	// pcl_viewer
	// Visualizer Vis_;

	// read/write to disk
	IoDisk Disk;

private:
	virtual void onInit();
	virtual void onFrontendInit() = 0;
	
};

}