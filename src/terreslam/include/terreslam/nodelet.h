/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-18 11:30:37
 */

#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <memory>

#include "terreslam/visualizer.h"
#include "terreslam/io_disk.h"

namespace terreslam
{

class Nodelet : public nodelet::Nodelet
{
public:
	Nodelet();

protected:
	// General topic
	std::string base_link_topic;
	std::string odom_topic;
	std::string blob_odom_topic;
	std::string cloud_topic;
	std::string cloud_xy_topic;
	std::string cloud_filtered_topic;
	std::string cloud_filtered_blobs_topic;
	std::string cloud_keypoints_topic;
	std::string cloud_plane_topic;
	std::string normal_filtered_topic;
	std::string blob_matches_topic;
	std::string dd_keypoint_matches_topic;
	std::string ddd_keypoint_matches_topic;
	std::string sub_lidar_topic;
	std::string sub_cam_topic;
	std::string sub_cam_depth_topic;
	std::string sub_cam_info_topic;
	std::string pub_cam_topic;
	std::string pub_cam_depth_topic;
	std::string visualization_kps_topic;
	std::string visualization_trajectory_topic;

	// General frame_id
	std::string base_link_frame_id;
	std::string odom_frame_id;
	std::string blob_odom_frame_id;
	std::string cloud_frame_id;
	std::string cloud_xy_frame_id;
	std::string cloud_filtered_frame_id;
	std::string cloud_filtered_blobs_frame_id;
	std::string cloud_keypoints_frame_id;
	std::string cloud_plane_frame_id;
	std::string normal_filtered_frame_id;
	std::string blob_matches_frame_id;
	std::string dd_keypoint_matches_frame_id;
	std::string ddd_keypoint_matches_frame_id;
	std::string sub_lidar_frame_id;
	std::string sub_cam_frame_id;
	std::string sub_cam_depth_frame_id;
	std::string sub_cam_info_frame_id;
	std::string pub_cam_frame_id;
	std::string pub_cam_depth_frame_id;
	std::string visualization_kps_frame_id;
	std::string visualization_trajectory_frame_id;
	
	/// General parameters
	int entry_count = 0;
	bool debug;
	std::string logs_dir;
	double max_depth;
	float max_steering_angle;
	float max_rollaxis_acc;
	float max_pitchaxis_acc;
	float max_rollaxis_vel;
	float max_pitchaxis_vel;

	/// Constants
	const double depthScale = pow(2,16)/120;
	const float bad_point = std::numeric_limits<float>::quiet_NaN();
	const float RAD2DEG = 180.0/M_PI;
	const float DEG2RAD = M_PI/180.0;

	/// Plane Filter
	bool use_normal_integral = false;
	float PF_thresh;
	bool PF_highpass;

	/// Plane Detector
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

	//DD Keypoint
	int DDKP_ws;

	//DDD Keypoint
	float DDDKP_SIFT_min_scale;
	int DDDKP_SIFT_nr_octaves;
	int DDDKP_SIFT_nr_scales_per_octave;
	float DDDKP_SIFT_min_contrast;
	float DDDKP_FPFH_radius;

	//MA
	bool MA_Blobs;
	bool MA_blob_regularisation;
	bool MA_KPs;
	bool MA_joint_KPs;
	bool MA_debug_Blobs_coarse;
	bool MA_debug_KPs;
	int MA_max_blob_heading_vels;
	


	// pcl_viewer
	// Visualizer Vis_;

	// read/write to disk
	IoDisk Disk;

private:
	virtual void onInit();
	virtual void onNodeletInit() = 0;
	
};

}