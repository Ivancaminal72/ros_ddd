/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-14 11:11:27
 */

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include "terreslam/frontend.h"

namespace terreslam
{

Frontend::Frontend()
{
	// std::cout << "Constructor frontend..." << std::endl;
}

void Frontend::onInit()
{	
	ros::NodeHandle & nh = getNodeHandle();
	ros::NodeHandle & pnh = getPrivateNodeHandle();
	const nodelet::V_string & str_argv = getMyArgv();

	/// - General parameters
	nh.getParam("/terreslam/logsdir", logs_dir);
	nh.getParam("/terreslam/max_depth", max_depth);

	/// - Comms parameters
	nh.getParam("/terreslam/frame_id/base", frame_id);
	nh.getParam("/terreslam/frame_id/odom", odom_frame_id);
	nh.getParam("/terreslam/frame_id/cloud", cloud_frame_id);
	nh.getParam("/terreslam/frame_id/cloud_xy", cloud_xy_frame_id);
	nh.getParam("/terreslam/frame_id/cloud_filtered", cloud_filtered_frame_id);
	nh.getParam("/terreslam/frame_id/cloud_filtered_blobs", cloud_filtered_blobs_frame_id);
	nh.getParam("/terreslam/frame_id/cloud_keypoints", cloud_keypoints_frame_id);
	nh.getParam("/terreslam/frame_id/cloud_plane", cloud_plane_frame_id);
	nh.getParam("/terreslam/frame_id/blob_matches", blob_matches_frame_id);
	nh.getParam("/terreslam/frame_id/keypoint_matches", keypoint_matches_frame_id);
	nh.getParam("/terreslam/frame_id/sub_lidar", sub_lidar_frame_id);
	nh.getParam("/terreslam/frame_id/sub_cam", sub_cam_frame_id);
	nh.getParam("/terreslam/frame_id/sub_cam_depth", sub_cam_depth_frame_id);
	nh.getParam("/terreslam/frame_id/sub_cam_info", sub_cam_info_frame_id);
		
	/// - Plane filter parameters
	nh.getParam("/terreslam/PF/threshold", PF_thresh);
	nh.getParam("/terreslam/PF/highpass", PF_highpass);	
	
	/// - Plane detector parameters
	nh.getParam("/terreslam/PD/debug", PD_debug);
	nh.getParam("/terreslam/PD/theta", PD_theta);
	nh.getParam("/terreslam/PD/phi", PD_phi);
	nh.getParam("/terreslam/PD/d", PD_d);
	nh.getParam("/terreslam/PD/max_plane", PD_max_plane);
	nh.getParam("/terreslam/PD/min_plane_size", PD_min_plane_size);
	nh.getParam("/terreslam/PD/thres_angle", PD_thres_angle);
	nh.getParam("/terreslam/PD/thres_dist", PD_thres_dist);
	// nh.getParam("/PD/thres_color", PD_thres_color);

	/// - Blob detector parameters
	nh.getParam("/terreslam/BD/tolerance", BD_tolerance);
	nh.getParam("/terreslam/BD/min_size", BD_min_size);
	nh.getParam("/terreslam/BD/max_size", BD_max_size);
	nh.getParam("/terreslam/BD/ppa", BD_ppa);
	nh.getParam("/terreslam/BD/alpha", BD_alpha);
	nh.getParam("/terreslam/BD/thres_xz", BD_thres_xz);
	nh.getParam("/terreslam/BD/thres_radius", BD_thres_radius);

	/// - DD Keypoint parameters
	nh.getParam("/terreslam/DDKP/window_size", DDKP_ws);

	onFrontendInit();
}

}