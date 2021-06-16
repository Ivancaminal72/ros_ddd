/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-06-15 11:37:46
 */

#include "terreslam/frontend.h"
#include "terreslam/utils/util_chrono.h"

#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <nav_msgs/Odometry.h>

namespace terreslam
{

class MetricAlignmentFrontend : public terreslam::Frontend
{
public:
	MetricAlignmentFrontend() :
		queue_size_(10)
		{
			std::cout << "Constructor metric_alignment_frontend..." << std::endl;
		}

private:

	void onFrontendInit()
	{
		std::cout << "Initalize metric_alignment_frontend..." << std::endl;
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();
		ros::NodeHandle cf_nh(nh, "cloud_filtered");

		/// Subscribers
		cloud_filtered_sub_ = cf_nh.subscribe(cloud_filtered_frame_id, queue_size_, &MetricAlignmentFrontend::callback, this);

		// Publishers
		odom_pub_ = nh.advertise<nav_msgs::Odometry>(odom_frame_id, 1);
	} 

	void callback(
		const sensor_msgs::PointCloud2ConstPtr& cf_msg_ptr)
	{
		std::cout << "Entry plane: " << entry_count << std::endl;
		// ///Start chrono ticking
		// std::chrono::duration<double> tick;
		// std::chrono::high_resolution_clock::time_point end_t, start_t;
		// start_t = std::chrono::high_resolution_clock::now();
		// end_t = std::chrono::high_resolution_clock::now();
		// tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr points (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::fromROSMsg(*cf_msg_ptr, *points);

		///METRIC ALIGNMENT
		

		/// Pre-PUBLISH
		// // Extract points beloging to a plane
		// pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp_plane (new pcl::PointCloud<pcl::PointXYZRGBA>);
		// pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scan_planes (new pcl::PointCloud<pcl::PointXYZRGBA>);
		// for(iterFeature it=scan_->beginFeature();it!=scan_->endFeature();it++)
		// {
		// 	if(it->second->Type()!=PLANE) continue;
		// 	pcl::copyPointCloud(*it->second->ptrPoints(),*it->second->ptrIndices(),*tmp_plane);
		// 	// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(tmp_plane,0,255,0);
		// 	// uint8_t r = 255;
		// 	// uint8_t g = 0;
		// 	// uint8_t b = 0;
		// 	// int32_t rgb = (r << 16) | (g << 8) | b; 
		// 	// for(auto &p: tmp_plane->points) p.rgb=rgb;
		// 	*scan_planes += *tmp_plane;
		// }

		/// PUBLISH
		/// - Cloud Filtered
		sensor_msgs::PointCloud2 msg_pcd;
		pcl::toROSMsg(*points, msg_pcd);
		msg_pcd.header.frame_id = cloud_filtered_frame_id;
		msg_pcd.header.stamp = cf_msg_ptr->header.stamp;
		odom_pub_.publish(msg_pcd);

		entry_count++;

		// tick_high_resolution(start_t, tick, elapsed);
		// printElapsed(elapsed, "Callback blob detector: ");
	}

	void skipFrame(std::string msg)
	{
		std::cerr<<msg<<std::endl;
		entry_count++;
	}

private:

	/// General variables
	int queue_size_;

	///Comms
	ros::Publisher odom_pub_;
	ros::Subscriber cloud_filtered_sub_;

	///Chrono timmings
	std::vector<double> elapsed;

};

PLUGINLIB_EXPORT_CLASS(terreslam::MetricAlignmentFrontend, nodelet::Nodelet);

}