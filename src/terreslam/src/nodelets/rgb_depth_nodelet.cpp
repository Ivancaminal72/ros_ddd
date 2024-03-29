/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-19 11:47:07
 */

#include "terreslam/nodelet.h"
#include "terreslam/camera_model.h"
#include "terreslam/utils/util_types.h"
#include "terreslam/utils/util_map.h"
#include "terreslam/utils/util_pcd.h"
#include "terreslam/utils/util_chrono.h"

#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <Eigen/Geometry>

namespace terreslam
{

class RGBDepthNodelet : public terreslam::Nodelet
{
public:
	RGBDepthNodelet() : 
	Nodelet(),
		queue_size_(10)
		{
			// std::cout << "Constructor rgb_depth_nodelet..." << std::endl;
		}

private:

	virtual void onNodeletInit()
	{
		std::cout << "Initalize rgb_depth_nodelet..." << std::endl;
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();
		
		std::string subscribedTopicsMsg;

		image_transport::ImageTransport rgb_it_sub(nh);
		image_transport::ImageTransport depth_it_sub(nh);
		image_transport::ImageTransport rgb_it_pub(nh);
		image_transport::ImageTransport depth_it_pub(nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), pnh);
		image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), pnh);

		/// Subscribers
		rgb_sub_filter_.subscribe(rgb_it_sub, sub_cam_topic, 1, hintsRgb);
		depth_sub_filter_.subscribe(depth_it_sub, sub_cam_depth_topic, 1, hintsDepth);
		info_sub_filter_.subscribe(nh, sub_cam_info_topic, 1);

		exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>
			(MyExactSyncPolicy(queue_size_), rgb_sub_filter_, depth_sub_filter_, info_sub_filter_);
		exactSync_->registerCallback(boost::bind(&RGBDepthNodelet::callback, this, _1, _2, _3));

		// Publishers
		image_rgb_pub_ = rgb_it_pub.advertiseCamera(pub_cam_topic,1);
		image_depth_pub_ = depth_it_pub.advertiseCamera(pub_cam_depth_topic,1);
		cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic, 10);
		old_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(old_cloud_topic, 10);
		// timer_ = nh.createTimer(ros::Duration(1.0), boost::bind(& NodeletClass::timerCb, this, _1));

		//Publish identity transforms
		static tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;
		geometry_msgs::TransformStamped Ts_identity;
		Ts_identity.transform.rotation.x = 0;
		Ts_identity.transform.rotation.y = 0;
		Ts_identity.transform.rotation.z = 0;
		Ts_identity.transform.rotation.w = 1;
		Ts_identity.transform.translation.x = 0;
		Ts_identity.transform.translation.y = 0;
		Ts_identity.transform.translation.z = 0;
		Ts_identity.header.frame_id = sub_cam_frame_id;
		Ts_identity.child_frame_id = pub_cam_frame_id;
		static_tf_broadcaster.sendTransform(Ts_identity);
		Ts_identity.child_frame_id = pub_cam_depth_frame_id;
		static_tf_broadcaster.sendTransform(Ts_identity);
		Ts_identity.child_frame_id = cloud_frame_id;
		static_tf_broadcaster.sendTransform(Ts_identity);
		Ts_identity.child_frame_id = old_cloud_frame_id;
		static_tf_broadcaster.sendTransform(Ts_identity);
		Ts_identity.child_frame_id = cloud_filtered_high_frame_id;
		static_tf_broadcaster.sendTransform(Ts_identity);
		Ts_identity.child_frame_id = cloud_filtered_low_frame_id;
		static_tf_broadcaster.sendTransform(Ts_identity);
		Ts_identity.child_frame_id = old_cloud_filtered_low_frame_id;
		static_tf_broadcaster.sendTransform(Ts_identity);
		Ts_identity.child_frame_id = cloud_plane_frame_id;
		static_tf_broadcaster.sendTransform(Ts_identity);
		Ts_identity.child_frame_id = cloud_filtered_blobs_frame_id;
		static_tf_broadcaster.sendTransform(Ts_identity);
		Ts_identity.child_frame_id = cloud_keypoints_frame_id;
		static_tf_broadcaster.sendTransform(Ts_identity);
		Ts_identity.child_frame_id = visualization_kps_frame_id;
		static_tf_broadcaster.sendTransform(Ts_identity);

		tf2::Quaternion quat;
		quat.setEulerZYX(-M_PI_2, 0, -M_PI_2);
		geometry_msgs::TransformStamped Ts_odom_vehicle;
		Ts_odom_vehicle.transform.rotation.x = quat.x();
		Ts_odom_vehicle.transform.rotation.y = quat.y();
		Ts_odom_vehicle.transform.rotation.z = quat.z();
		Ts_odom_vehicle.transform.rotation.w = quat.w();
		Ts_odom_vehicle.transform.translation.x = 0;
		Ts_odom_vehicle.transform.translation.y = 0;
		Ts_odom_vehicle.transform.translation.z = 0;
		Ts_odom_vehicle.header.frame_id = base_link_frame_id;
		Ts_odom_vehicle.child_frame_id = odom_frame_id;
		static_tf_broadcaster.sendTransform(Ts_odom_vehicle);

		static tf2_ros::TransformBroadcaster odom_broadcaster;
		Ts_identity.header.frame_id = odom_frame_id;
		Ts_identity.child_frame_id = sub_cam_frame_id;
		odom_broadcaster.sendTransform(Ts_identity);
		Ts_identity.child_frame_id = visualization_trajectory_frame_id;
		static_tf_broadcaster.sendTransform(Ts_identity);
	} 

	void callback(
		const sensor_msgs::Image::ConstPtr& rgb_msg,
		const sensor_msgs::Image::ConstPtr& depth_msg,
		const sensor_msgs::CameraInfo::ConstPtr& info_msg)
	{
		if(debug) std::cout << "Entry: " << entry_count << std::endl;
		// ///Start chrono ticking
		// std::chrono::duration<double> tick;
		// std::chrono::high_resolution_clock::time_point end_t, start_t;
		// start_t = std::chrono::high_resolution_clock::now();
		// end_t = std::chrono::high_resolution_clock::now();
		// tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);

		cv_bridge::CvImage::ConstPtr ptr_msg_rgb = cv_bridge::toCvShare(rgb_msg);
		cv_bridge::CvImage::ConstPtr ptr_msg_depth = cv_bridge::toCvShare(depth_msg);
		sensor_msgs::CameraInfo info = *info_msg;

		/// INITALIZATION
		uint32_t height, width;
		if (rgb_msg->height == depth_msg->height) height = rgb_msg->height; else {skipFrame("Different img height"); return;}
		if (rgb_msg->width == depth_msg->width) width = rgb_msg->width; else {skipFrame("Different img width"); return;}
		cv::Mat img_rgb = cv::Mat(rgb_msg->height, rgb_msg->width, ptr_msg_rgb->image.type());
		cv::Mat img_depth = cv::Mat(depth_msg->height, depth_msg->width, ptr_msg_depth->image.type());
		ptr_msg_rgb->image.copyTo(cv::Mat(img_rgb, cv::Rect(0, 0, rgb_msg->width, rgb_msg->height)));
		ptr_msg_depth->image.copyTo(cv::Mat(img_depth, cv::Rect(0, 0, depth_msg->width, depth_msg->height)));

		/// PRE-BACKPROJECTION
		CameraModel cam_model(info);
		// cam_model.printModel();

		/// Pointer to the Mat data
		uint8_t *rgb_ptr;
		rgb_ptr=img_rgb.data;

		cur_points = ptrPointCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		Eigen::Vector4d point_eigen;
		Eigen::Vector4d point_eigen_backproj;
		Eigen::Matrix4d P_inv = cam_model.P().inverse().matrix();
		pcl::PointXYZRGBA point_pcl;
		pcl::PointXY tmp_pointxy;
		/// clear the pointcloud 
		/// the allocated memory does not release
		/// the newly pushed elements cover the old ones

		/// BACKPROJECTION
		for (int v = 0; v < img_depth.rows; ++v)
			for (int u = 0; u < img_depth.cols; ++u)
			{
				double depth_yx = (double) img_depth.at<ushort>(v, u) / depthScale;
				if(depth_yx != 0)
				{
					/// 3 channels for one pixel in rgb image
					point_pcl.b=*rgb_ptr;
					rgb_ptr++;
					point_pcl.g=*rgb_ptr;
					rgb_ptr++;
					point_pcl.r=*rgb_ptr;
					rgb_ptr++;
					point_eigen << (double) u * depth_yx, (double) v * depth_yx, (double) depth_yx, 1;
					point_eigen_backproj = P_inv * point_eigen;
					point_pcl.x = (float) point_eigen_backproj(0);
					point_pcl.y = (float) point_eigen_backproj(1);
					point_pcl.z = (float) point_eigen_backproj(2);
					cur_points->push_back(point_pcl);
				}
				else if (use_normal_integral)
				{
					/// 3 channels for one pixel in rgb image
					point_pcl.b=*rgb_ptr;
					rgb_ptr++;
					point_pcl.g=*rgb_ptr;
					rgb_ptr++;
					point_pcl.r=*rgb_ptr;
					rgb_ptr++;
					point_pcl.x = point_pcl.y = point_pcl.z = bad_point;
					cur_points->push_back(point_pcl);
				}
				else rgb_ptr+=3;
			}

		// std::cout<<"Number of valid depth points: "<<cur_points->size()<<std::endl;

		/// PUBLISH
		/// - Cloud
		sensor_msgs::PointCloud2 msg_pcd;
		pcl::toROSMsg(*cur_points, msg_pcd);
		msg_pcd.header.frame_id = cloud_frame_id;
		msg_pcd.header.stamp = info.header.stamp;
		cloud_pub_.publish(msg_pcd);

		if(entry_count > 0)
		{
			pcl::toROSMsg(*old_points, msg_pcd);
			msg_pcd.header.frame_id = old_cloud_frame_id;
			msg_pcd.header.stamp = info.header.stamp;
			old_cloud_pub_.publish(msg_pcd);
		}
		
		/// - Images
		sensor_msgs::Image rgb_msg_pub = *rgb_msg;
		sensor_msgs::Image depth_msg_pub = *depth_msg;
		sensor_msgs::CameraInfo info_msg_pub = *info_msg;
		rgb_msg_pub.header.frame_id = pub_cam_frame_id;
		info_msg_pub.header.frame_id = pub_cam_frame_id;
		image_rgb_pub_.publish(rgb_msg_pub, info_msg_pub);
		depth_msg_pub.header.frame_id = pub_cam_depth_frame_id;
		info_msg_pub.header.frame_id = pub_cam_depth_frame_id;
		image_depth_pub_.publish(depth_msg_pub, info_msg_pub);
		
		//Update old data
		old_points = cur_points;
		
		entry_count++;

		// util::tick_high_resolution(start_t, tick, elapsed);
		// util::printElapsed(elapsed, "Callback rgb_depth: ");
	}

	void skipFrame(std::string msg)
	{
		std::cerr<<msg<<std::endl;
		entry_count++;
	}

private:
	/// Variables
	int queue_size_;

	/// Chrono timmings
	// std::vector<double> elapsed;

	/// Comms
	ros::Publisher cloud_pub_;
	ros::Publisher old_cloud_pub_;
  image_transport::CameraPublisher image_rgb_pub_;
  image_transport::CameraPublisher image_depth_pub_;
	image_transport::SubscriberFilter rgb_sub_filter_;
	image_transport::SubscriberFilter depth_sub_filter_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_filter_;
	
	typedef message_filters::sync_policies::ExactTime
		<sensor_msgs::Image,
		sensor_msgs::Image,
		sensor_msgs::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;

	/// Types
	ptrPointCloud cur_points;
	ptrPointCloud old_points;

};

PLUGINLIB_EXPORT_CLASS(terreslam::RGBDepthNodelet, nodelet::Nodelet);

}