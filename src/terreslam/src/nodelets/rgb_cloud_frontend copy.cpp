/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-26 19:35:22
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
		ros::NodeHandle lidar_nh(nh, "lidar_img");
		ros::NodeHandle rgb_pnh(pnh, "rgb_img");
		ros::NodeHandle lidar_pnh(pnh, "lidar_img");
		image_transport::ImageTransport rgb_it(rgb_nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);

		//Subscribers
		rgb_sub_.subscribe(rgb_it, sub_cam_frame_id_, 1, hintsRgb);
		lidar_sub_.subscribe(lidar_nh, sub_lidar_frame_id_, 1);
		info_sub_.subscribe(rgb_nh, sub_cam_info_frame_id_, 1);

		exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>
			(MyExactSyncPolicy(queue_size_), rgb_sub_, lidar_sub_, info_sub_);
		exactSync_->registerCallback(boost::bind(&RGBDepthFrontend::callback, this, _1, _2, _3));
	} 

	void callback(
		const sensor_msgs::ImageConstPtr& rgb_msg,
		const sensor_msgs::PointCloud2& lidar_msg,
		const sensor_msgs::CameraInfoConstPtr& info_msg)
	{


		entry_count_++;
	}

private:
	image_transport::SubscriberFilter rgb_sub_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
	int entry_count_ = 0;

	typedef message_filters::sync_policies::ExactTime
		<sensor_msgs::Image,
		sensor_msgs::PointCloud2,
		sensor_msgs::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;

	int queue_size_;
};

PLUGINLIB_EXPORT_CLASS(terreslam::RGBDepthFrontend, nodelet::Nodelet);

}