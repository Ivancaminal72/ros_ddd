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
	std::cout << "Constructor frontend..." << std::endl;
}

void Frontend::onInit()
{
	std::cout << "onInit frontend:" << std::endl;
	
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
	nh.getParam("/terreslam/frame_id/cloud_plane", cloud_plane_frame_id);
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

	onFrontendInit();
}

}