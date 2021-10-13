/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-14 11:11:27
 */

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include "terreslam/nodelet.h"

namespace terreslam
{

Nodelet::Nodelet()
{
	// std::cout << "Constructor nodelet..." << std::endl;
}

void Nodelet::onInit()
{	
	ros::NodeHandle & nh = getNodeHandle();
	ros::NodeHandle & pnh = getPrivateNodeHandle();
	const nodelet::V_string & str_argv = getMyArgv();

	/// - Comms parameters
	nh.getParam("base_topic/base_link", base_link_topic);
	nh.getParam("base_topic/odom", odom_topic);
	nh.getParam("base_topic/blob_odom", blob_odom_topic);
	nh.getParam("base_topic/cloud", cloud_topic);
	nh.getParam("base_topic/cloud_xy", cloud_xy_topic);
	nh.getParam("base_topic/cloud_filtered", cloud_filtered_topic);
	nh.getParam("base_topic/cloud_filtered_blobs", cloud_filtered_blobs_topic);
	nh.getParam("base_topic/cloud_keypoints", cloud_keypoints_topic);
	nh.getParam("base_topic/cloud_plane", cloud_plane_topic);
	nh.getParam("base_topic/normal_filtered", normal_filtered_topic);
	nh.getParam("base_topic/blob_matches", blob_matches_topic);
	nh.getParam("base_topic/blob_points", blob_points_topic);
	nh.getParam("base_topic/dd_keypoint_matches", dd_keypoint_matches_topic);
	nh.getParam("base_topic/ddd_keypoint_matches", ddd_keypoint_matches_topic);
	nh.getParam("base_topic/sub_lidar", sub_lidar_topic);
	nh.getParam("base_topic/sub_cam", sub_cam_topic);
	nh.getParam("base_topic/sub_cam_depth", sub_cam_depth_topic);
	nh.getParam("base_topic/sub_cam_info", sub_cam_info_topic);
	nh.getParam("base_topic/pub_cam", pub_cam_topic);
	nh.getParam("base_topic/pub_cam_depth", pub_cam_depth_topic);
	nh.getParam("base_topic/visualization_kps", visualization_kps_topic);
	nh.getParam("base_topic/visualization_trajectory", visualization_trajectory_topic);

	base_link_frame_id = (nh.getNamespace()+"/"+base_link_topic).erase(0,1);
	odom_frame_id = (nh.getNamespace()+"/"+odom_topic).erase(0,1);
	blob_odom_frame_id = (nh.getNamespace()+"/"+blob_odom_topic).erase(0,1);
	cloud_frame_id = (nh.getNamespace()+"/"+cloud_topic).erase(0,1);
	cloud_xy_frame_id = (nh.getNamespace()+"/"+cloud_xy_topic).erase(0,1);
	cloud_filtered_frame_id = (nh.getNamespace()+"/"+cloud_filtered_topic).erase(0,1);
	cloud_filtered_blobs_frame_id = (nh.getNamespace()+"/"+cloud_filtered_blobs_topic).erase(0,1);
	cloud_keypoints_frame_id = (nh.getNamespace()+"/"+cloud_keypoints_topic).erase(0,1);
	cloud_plane_frame_id = (nh.getNamespace()+"/"+cloud_plane_topic).erase(0,1);
	normal_filtered_frame_id = (nh.getNamespace()+"/"+normal_filtered_topic).erase(0,1);
	blob_matches_frame_id = (nh.getNamespace()+"/"+blob_matches_topic).erase(0,1);
	blob_points_frame_id = (nh.getNamespace()+"/"+blob_points_topic).erase(0,1);
	dd_keypoint_matches_frame_id = (nh.getNamespace()+"/"+dd_keypoint_matches_topic).erase(0,1);
	ddd_keypoint_matches_frame_id = (nh.getNamespace()+"/"+ddd_keypoint_matches_topic).erase(0,1);
	sub_lidar_frame_id = (nh.getNamespace()+"/"+sub_lidar_topic).erase(0,1);
	sub_cam_frame_id = (nh.getNamespace()+"/"+sub_cam_topic).erase(0,1);
	sub_cam_depth_frame_id = (nh.getNamespace()+"/"+sub_cam_depth_topic).erase(0,1);
	sub_cam_info_frame_id = (nh.getNamespace()+"/"+sub_cam_info_topic).erase(0,1);
	pub_cam_frame_id = (nh.getNamespace()+"/"+pub_cam_topic).erase(0,1);
	pub_cam_depth_frame_id = (nh.getNamespace()+"/"+pub_cam_depth_topic).erase(0,1);
	visualization_kps_frame_id = (nh.getNamespace()+"/"+visualization_kps_topic).erase(0,1);
	visualization_trajectory_frame_id = (nh.getNamespace()+"/"+visualization_trajectory_topic).erase(0,1);
		
	/// - General parameters
	nh.getParam("debug", debug);
	nh.getParam("logsdir", logs_dir);
	nh.getParam("max_depth", max_depth);
	nh.getParam("max_steering_angle", max_steering_angle);
	nh.getParam("max_slope_angle", max_slope_angle);
	nh.getParam("max_rollaxis_acc", max_rollaxis_acc);
	nh.getParam("max_pitchaxis_acc", max_pitchaxis_acc);
	nh.getParam("max_rollaxis_vel", max_rollaxis_vel);
	nh.getParam("max_pitchaxis_vel", max_pitchaxis_vel);
	
	/// - Plane filter parameters
	nh.getParam("PF/threshold", PF_thresh);
	nh.getParam("PF/highpass", PF_highpass);	
	
	/// - Plane detector parameters
	nh.getParam("PD/debug", PD_debug);
	nh.getParam("PD/theta", PD_theta);
	nh.getParam("PD/phi", PD_phi);
	nh.getParam("PD/d", PD_d);
	nh.getParam("PD/max_plane", PD_max_plane);
	nh.getParam("PD/min_plane_size", PD_min_plane_size);
	nh.getParam("PD/thres_angle", PD_thres_angle);
	nh.getParam("PD/thres_dist", PD_thres_dist);
	// nh.getParam("/PD/thres_color", PD_thres_color);

	/// - Blob detector parameters
	nh.getParam("BD/tolerance", BD_tolerance);
	nh.getParam("BD/min_size", BD_min_size);
	nh.getParam("BD/max_size", BD_max_size);
	nh.getParam("BD/ppa", BD_ppa);
	nh.getParam("BD/alpha", BD_alpha);
	nh.getParam("BD/thres_xz", BD_thres_xz);
	nh.getParam("BD/thres_radius", BD_thres_radius);

	/// - DD Keypoint parameters
	nh.getParam("DDKP/window_size", DDKP_ws);

	/// - DDD Keypoint parameters
	nh.getParam("DDDKP/SIFT_min_scale", DDDKP_SIFT_min_scale);
	nh.getParam("DDDKP/SIFT_nr_octaves", DDDKP_SIFT_nr_octaves);
	nh.getParam("DDDKP/SIFT_nr_scales_per_octave", DDDKP_SIFT_nr_scales_per_octave);
	nh.getParam("DDDKP/SIFT_min_contrast", DDDKP_SIFT_min_contrast);
	nh.getParam("DDDKP/FPFH_radius", DDDKP_FPFH_radius);

	/// - MA
	nh.getParam("MA/regularisation", MA_regularisation);
	nh.getParam("MA/Blobs", MA_Blobs);
	nh.getParam("MA/DDKPs", MA_DDKPs);
	nh.getParam("MA/DDDKPs", MA_DDDKPs);
	assert(MA_Blobs || MA_DDKPs || MA_DDDKPs);
	nh.getParam("MA/joint_KPs", MA_joint_KPs);
	nh.getParam("MA/debug_Blobs_coarse", MA_debug_Blobs_coarse);
	nh.getParam("MA/debug_KPs", MA_debug_KPs);
	nh.getParam("MA/max_blob_heading_vels", MA_max_blob_heading_vels);
	

	onNodeletInit();
}

}