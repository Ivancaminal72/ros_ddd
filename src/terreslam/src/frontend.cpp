/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-14 11:11:27
 */

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>

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

	/// Parameters
	nh.getParam("/terreslam/logsdir", logs_dir);
	nh.getParam("/terreslam/max_depth", max_depth);

	/// Comms
	nh.getParam("/terreslam/frame_id", frame_id);
	nh.getParam("/terreslam/frame_id/odom", odom_frame_id);
	nh.getParam("/terreslam/frame_id/cloud", cloud_frame_id);
	nh.getParam("/terreslam/frame_id/plane", plane_frame_id);
	nh.getParam("/terreslam/frame_id/sub_lidar", sub_lidar_frame_id);
	nh.getParam("/terreslam/frame_id/sub_cam", sub_cam_frame_id);
	nh.getParam("/terreslam/frame_id/sub_cam_depth", sub_cam_depth_frame_id);
	nh.getParam("/terreslam/frame_id/sub_cam_info", sub_cam_info_frame_id);

	// Publishers
	odom_pub = nh.advertise<nav_msgs::Odometry>(odom_frame_id, 1);
	cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_frame_id, 10);
	plane_pub = nh.advertise<sensor_msgs::PointCloud2>(plane_frame_id, 10);
	// timer_ = nh.createTimer(ros::Duration(1.0), boost::bind(& NodeletClass::timerCb, this, _1));

	//Publish identity T_cam_cloud
	geometry_msgs::TransformStamped Ts_cloud_lidar;
	static tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;
	Ts_cloud_lidar.transform.rotation.x = 0;
	Ts_cloud_lidar.transform.rotation.y = 0;
	Ts_cloud_lidar.transform.rotation.z = 0;
	Ts_cloud_lidar.transform.rotation.w = 1;
	Ts_cloud_lidar.transform.translation.x = 0;
	Ts_cloud_lidar.transform.translation.y = 0;
	Ts_cloud_lidar.transform.translation.z = 0;
	Ts_cloud_lidar.header.frame_id = sub_lidar_frame_id;
	Ts_cloud_lidar.child_frame_id = cloud_frame_id;
	static_tf_broadcaster.sendTransform(Ts_cloud_lidar);
	Ts_cloud_lidar.child_frame_id = plane_frame_id;
	static_tf_broadcaster.sendTransform(Ts_cloud_lidar);

	onFrontendInit();
}

}