/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-08-19 17:43:50
 */

#include "terreslam/frontend.h"
#include "terreslam/utils/util_pcd.h"
#include "terreslam/utils/util_chrono.h"
#include "terreslam/utils/util_general.h"

#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <math.h>
#include <fstream>

#include <Eigen/Dense>

//msgs
#include <terreslam/KeyPointMatches.h>

namespace terreslam
{

class DDDKeypointFrontend : public terreslam::Frontend
{
public:
	DDDKeypointFrontend() :
		queue_size_(10)
		{
			// std::cout << "Constructor ddd_keypoint_frontend..." << std::endl;
		}

private:

	void onFrontendInit()
	{
		std::cout << "Initalize ddd_keypoint_frontend..." << std::endl;
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();
		ros::NodeHandle cf_nh(nh, "cloud_filtered");

		/// Subscribers
		cloud_filtered_sub_ = cf_nh.subscribe(cloud_filtered_frame_id, queue_size_, &DDDKeypointFrontend::callback, this);

		// Publishers

	} 

	void callback(
		const sensor_msgs::PointCloud2::ConstPtr& cf_msg_ptr)
	{
		std::cout << "Entry DDD KP: " << entry_count << std::endl;
		// ///Start chrono ticking
		// std::chrono::duration<double> tick;
		// std::chrono::high_resolution_clock::time_point end_t, start_t;
		// start_t = std::chrono::high_resolution_clock::now();
		// end_t = std::chrono::high_resolution_clock::now();
		// tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr points (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::fromROSMsg(*cf_msg_ptr, *points);


		/// PUBLISH
		
		
		entry_count++;

		// util::tick_high_resolution(start_t, tick, elapsed);
		// util::printElapsed(elapsed, "Callback ddd_keypoint: ");
	}

private:

	/// General variables
	int queue_size_;

	/// Comms
	ros::Subscriber cloud_filtered_sub_;

	/// Chrono timmings
	std::vector<double> elapsed;

	///Kpts

};

PLUGINLIB_EXPORT_CLASS(terreslam::DDDKeypointFrontend, nodelet::Nodelet);

}