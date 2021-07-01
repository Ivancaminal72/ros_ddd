/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-19 11:47:07
 */

#include "terreslam/frontend.h"
#include "terreslam/camera_model.h"
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
#include <tf2_ros/static_transform_broadcaster.h>

#include <Eigen/Geometry>

namespace terreslam
{

class RGBDepthFrontend : public terreslam::Frontend
{
public:
	RGBDepthFrontend() : 
	Frontend(),
		queue_size_(10)
		{
			std::cout << "Constructor rgb_depth_frontend..." << std::endl;
		}

private:

	virtual void onFrontendInit()
	{
		std::cout << "Initalize rgb_depth_frontend..." << std::endl;
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();
		
		std::string subscribedTopicsMsg;

		ros::NodeHandle rgb_nh(nh, "rgb_img");
		ros::NodeHandle depth_nh(nh, "depth_img");
		ros::NodeHandle rgb_pnh(pnh, "rgb_img");
		ros::NodeHandle depth_pnh(pnh, "depth_img");
		image_transport::ImageTransport rgb_it(rgb_nh);
		image_transport::ImageTransport depth_it(depth_nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
		image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

		/// Subscribers
		rgb_sub_filter_.subscribe(rgb_it, sub_cam_frame_id, 1, hintsRgb);
		depth_sub_filter_.subscribe(depth_it, sub_cam_depth_frame_id, 1, hintsDepth);
		info_sub_filter_.subscribe(rgb_nh, sub_cam_info_frame_id, 1);

		exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>
			(MyExactSyncPolicy(queue_size_), rgb_sub_filter_, depth_sub_filter_, info_sub_filter_);
		exactSync_->registerCallback(boost::bind(&RGBDepthFrontend::callback, this, _1, _2, _3));

		// Publishers
		cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cloud_frame_id, 10);
		cloud_xy_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cloud_xy_frame_id, 10);
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
		Ts_cloud_lidar.child_frame_id = cloud_filtered_frame_id;
		static_tf_broadcaster.sendTransform(Ts_cloud_lidar);
		Ts_cloud_lidar.child_frame_id = cloud_plane_frame_id;
		static_tf_broadcaster.sendTransform(Ts_cloud_lidar);
		Ts_cloud_lidar.child_frame_id = cloud_filtered_blobs_frame_id;
		static_tf_broadcaster.sendTransform(Ts_cloud_lidar);
	} 

	void callback(
		const sensor_msgs::ImageConstPtr& rgb_msg,
		const sensor_msgs::ImageConstPtr& depth_msg,
		const sensor_msgs::CameraInfoConstPtr& info_msg)
	{
		std::cout << "Entry: " << entry_count << std::endl;
		// ///Start chrono ticking
		// std::chrono::duration<double> tick;
		// std::chrono::high_resolution_clock::time_point end_t, start_t;
		// start_t = std::chrono::high_resolution_clock::now();
		// end_t = std::chrono::high_resolution_clock::now();
		// tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);

		cv_bridge::CvImageConstPtr ptr_msg_rgb = cv_bridge::toCvShare(rgb_msg);
		cv_bridge::CvImageConstPtr ptr_msg_depth = cv_bridge::toCvShare(depth_msg);
		sensor_msgs::CameraInfo info = *info_msg;

		/// INITALIZATION
		uint32_t height, width;
		if (rgb_msg->height == depth_msg->height) height = rgb_msg->height; else {skipFrame("Different img height"); return;}
		if (rgb_msg->width == depth_msg->width) width = rgb_msg->width; else {skipFrame("Different img width"); return;}
		cv::Mat img_rgb = cv::Mat(rgb_msg->height, rgb_msg->width, ptr_msg_rgb->image.type());
		cv::Mat img_depth = cv::Mat(depth_msg->height, depth_msg->width, ptr_msg_depth->image.type());
		ptr_msg_rgb->image.copyTo(cv::Mat(img_rgb, cv::Rect(0, 0, rgb_msg->width, rgb_msg->height)));
		ptr_msg_depth->image.copyTo(cv::Mat(img_depth, cv::Rect(0, 0, depth_msg->width, depth_msg->height)));

		/// BACKPROJECTION
		CameraModel cam_model(info);
		// cam_model.printModel();

		/// Pointer to the Mat data
		uint8_t *rgb_ptr;
		rgb_ptr=img_rgb.data;

		scan_ = new Scan();
		scan_->points() = ptrPointCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		scan_->normals() = ptrNormalCloud (new pcl::PointCloud<pcl::Normal>);
		scan_->pixels() = ptrPixelCloud (new pcl::PointCloud<pcl::PointXY>);
		scan_->imgRGB() = img_rgb;
		scan_->imgDepth() = img_depth;
		Eigen::Vector4d point_eigen;
		Eigen::Vector4d point_eigen_backproj;
		Eigen::Matrix4d P_inv = cam_model.P().inverse().matrix();
		pcl::PointXYZRGBA point_pcl;
		pcl::PointXY tmp_pointxy;
		/// clear the pointcloud 
		/// the allocated memory does not release
		/// the newly pushed elements cover the old ones
		scan_->points()->clear();
		scan_->normals()->clear();
		scan_->pixels()->clear();

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
					tmp_pointxy.x=u;
					tmp_pointxy.y=v;
					point_eigen << (double) u * depth_yx, (double) v * depth_yx, (double) depth_yx, 1;
					point_eigen_backproj = P_inv * point_eigen;
					point_pcl.x = (float) point_eigen_backproj(0);
					point_pcl.y = (float) point_eigen_backproj(1);
					point_pcl.z = (float) point_eigen_backproj(2);
					scan_->points()->push_back(point_pcl);
					scan_->pixels()->push_back(tmp_pointxy);
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
					scan_->points()->push_back(point_pcl);
				}
				else rgb_ptr+=3;
			}

		if(use_normal_integral)
		{
			/// Organize the point_cloud for the normal estimation
			scan_->points()->width=width;
			scan_->points()->height=height;
		}


		/// PUBLISH
		/// - Cloud
		sensor_msgs::PointCloud2 msg_pcd;
		pcl::toROSMsg(*scan_->points(), msg_pcd);
		msg_pcd.header.frame_id = cloud_frame_id;
		msg_pcd.header.stamp = info.header.stamp;
		cloud_pub_.publish(msg_pcd);

		/// - Cloud XY
		pcl::toROSMsg(*scan_->pixels(), msg_pcd);
		msg_pcd.header.frame_id = cloud_xy_frame_id;
		msg_pcd.header.stamp = info.header.stamp;
		cloud_xy_pub_.publish(msg_pcd);

		scan_->release();
		entry_count++;

		// tick_high_resolution(start_t, tick, elapsed);
		// printElapsed(elapsed, "Callback rgb_depth: ");
	}

	void skipFrame(std::string msg)
	{
		std::cerr<<msg<<std::endl;
		entry_count++;
	}

private:
	/// Variables
	int queue_size_;

	///Chrono timmings
	// std::vector<double> elapsed;

	/// Comms
	ros::Publisher cloud_pub_;
	ros::Publisher cloud_xy_pub_;
	image_transport::SubscriberFilter rgb_sub_filter_;
	image_transport::SubscriberFilter depth_sub_filter_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_filter_;
	
	typedef message_filters::sync_policies::ExactTime
		<sensor_msgs::Image,
		sensor_msgs::Image,
		sensor_msgs::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;
};

PLUGINLIB_EXPORT_CLASS(terreslam::RGBDepthFrontend, nodelet::Nodelet);

}