/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-06-15 10:37:02
 */

#include "terreslam/frontend.h"
#include "terreslam/utils/util_pcd.h"
#include "terreslam/utils/util_chrono.h"

#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>

namespace terreslam
{

class BlobDetectorFrontend : public terreslam::Frontend
{
public:
	BlobDetectorFrontend() :
		queue_size_(10)
		{
			std::cout << "Constructor blob_detector_frontend..." << std::endl;
		}

private:

	void onFrontendInit()
	{
		std::cout << "Initalize blob_detector_frontend..." << std::endl;
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();
		ros::NodeHandle cf_nh(nh, "cloud_filtered");

		/// Subscribers
		cloud_filtered_sub_ = cf_nh.subscribe(cloud_filtered_frame_id, queue_size_, &BlobDetectorFrontend::callback, this);

		// Publishers
		blob_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cloud_filtered_blobs_frame_id, 10);
	} 

	void callback(
		const sensor_msgs::PointCloud2ConstPtr& cf_msg_ptr)
	{
		std::cout << "Entry blob: " << entry_count << std::endl;
		// ///Start chrono ticking
		// std::chrono::duration<double> tick;
		// std::chrono::high_resolution_clock::time_point end_t, start_t;
		// start_t = std::chrono::high_resolution_clock::now();
		// end_t = std::chrono::high_resolution_clock::now();
		// tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr points (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::fromROSMsg(*cf_msg_ptr, *points);

		///BLOB DETECTION
		// Creating the KdTree object for the search method of the extraction
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
		tree->setInputCloud (points);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
		ec.setClusterTolerance (0.3);
		ec.setMinClusterSize (50);
		ec.setMaxClusterSize (700);
		ec.setSearchMethod (tree);
		ec.setInputCloud (points);
		ec.extract (cluster_indices);

		std::cout<<"Cluster size: "<<cluster_indices.size()<<std::endl;

		double j = 0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			uint32_t rgb = util::rgb_palette(j/cluster_indices.size());
			for (const auto& idx : it->indices)
				(*points)[idx].rgb=rgb;
			j+=1;
		}

		/// PUBLISH
		/// - Cloud Filtered Blobs
		sensor_msgs::PointCloud2 msg_pcd;
		pcl::toROSMsg(*points, msg_pcd);
		msg_pcd.header.frame_id = cloud_filtered_blobs_frame_id;
		msg_pcd.header.stamp = cf_msg_ptr->header.stamp;
		blob_pub_.publish(msg_pcd);

		/// - Blobs
		// pcl::toROSMsg(*scan_planes, msg_pcd);
		// msg_pcd.header.frame_id = cloud_plane_frame_id;
		// plane_pub.publish(msg_pcd);

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
	ros::Publisher blob_pub_;
	ros::Subscriber cloud_filtered_sub_;

	///Chrono timmings
	std::vector<double> elapsed;

};

PLUGINLIB_EXPORT_CLASS(terreslam::BlobDetectorFrontend, nodelet::Nodelet);

}