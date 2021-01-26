/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-19 11:47:07
 *    Last Modified: 2021-01-26 14:05:12
 */

#include "terreslam/frontend.h"
#include "terreslam/camera_model.h"
#include "terreslam/comms/util_msg.h"

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

#include <Eigen/Geometry>
#include <terreslam/util_algebra.h>

namespace terreslam
{

class RGBDepthFrontend : public terreslam::Frontend
{
public:
	RGBDepthFrontend() :
		Frontend(),
		queue_size_(10)
		{

		}

private:

	virtual void onFrontendInit()
	{
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

		//Subscribers
		rgb_sub_.subscribe(rgb_it, sub_cam_frame_id_, 1, hintsRgb);
		depth_sub_.subscribe(depth_it, sub_cam_depth_frame_id_, 1, hintsDepth);
		info_sub_.subscribe(rgb_nh, sub_cam_info_frame_id_, 1);

		exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>
			(MyExactSyncPolicy(queue_size_), rgb_sub_, depth_sub_, info_sub_);
		exactSync_->registerCallback(boost::bind(&RGBDepthFrontend::callback, this, _1, _2, _3));
	} 

	void callback(
		const sensor_msgs::ImageConstPtr& rgb_msg,
		const sensor_msgs::ImageConstPtr& depth_msg,
		const sensor_msgs::CameraInfoConstPtr& info_msg)
	{
		std::cout << "Entry: " << entry_count_ << std::endl;
		// std::cout << rgb_msg->header.stamp << std::endl;
		// std::cout << depth_msg->header.stamp << std::endl;
		// std::cout << info_msg->header.stamp << std::endl;

		cv_bridge::CvImageConstPtr ptr_rgb = cv_bridge::toCvShare(rgb_msg);
		cv_bridge::CvImageConstPtr ptr_depth = cv_bridge::toCvShare(depth_msg);
		sensor_msgs::CameraInfo info = *info_msg;

		// initialize
		cv::Mat rgb_img = cv::Mat(rgb_msg->height, rgb_msg->width, ptr_rgb->image.type());
		cv::Mat depth_img = cv::Mat(depth_msg->height, depth_msg->width, ptr_depth->image.type());
		ptr_rgb->image.copyTo(cv::Mat(rgb_img, cv::Rect(0, 0, rgb_msg->width, rgb_msg->height)));
		ptr_depth->image.copyTo(cv::Mat(depth_img, cv::Rect(0, 0, depth_msg->width, depth_msg->height)));

		// backprojection
		CameraModel cam_model(info);
		// cam_model.printModel();

		pcl::PointCloud<pcl::PointXYZ> pcd;
		Eigen::Vector4d point_eigen;
		Eigen::Vector4d point_eigen_backproj;
		pcl::PointXYZ point_pcl_backproj;
		Eigen::Matrix4d backproj_mat = cam_model.P().inverse().matrix();
		for (int y = 0; y < depth_img.rows; ++y)
			for (int x = 0; x < depth_img.cols; ++x)
			{
				double depth_yx = (double) depth_img.at<ushort>(y, x) / depthScale;
				if(depth_yx != 0)
				{
					point_eigen << (double) x * depth_yx, (double) y * depth_yx, (double) depth_yx, 1;
					point_eigen_backproj = backproj_mat * point_eigen;
					point_pcl_backproj.x = (float) point_eigen_backproj(0);
					point_pcl_backproj.y = (float) point_eigen_backproj(1);
					point_pcl_backproj.z = (float) point_eigen_backproj(2);
					pcd.push_back(point_pcl_backproj);
				}
			}

		sensor_msgs::PointCloud2 msg_pcd;
		pcl::toROSMsg(pcd, msg_pcd);
		msg_pcd.header.frame_id = "/terreslam/cloud";
		cloud_pub_.publish(msg_pcd);

		entry_count_++;
	}

private:
	image_transport::SubscriberFilter rgb_sub_;
	image_transport::SubscriberFilter depth_sub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
	int entry_count_ = 0;

	typedef message_filters::sync_policies::ExactTime
		<sensor_msgs::Image,
		sensor_msgs::Image,
		sensor_msgs::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;

	//Constants
	const double depthScale = pow(2,16)/120;

	int queue_size_;
};

PLUGINLIB_EXPORT_CLASS(terreslam::RGBDepthFrontend, nodelet::Nodelet);

}