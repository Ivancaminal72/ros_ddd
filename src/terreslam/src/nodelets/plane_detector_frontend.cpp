/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-19 11:47:07
 */

#include "terreslam/frontend.h"
#include "terreslam/camera_model.h"
#include "terreslam/utils/util_msg.h"
#include "terreslam/utils/util_map.h"
#include "terreslam/utils/util_pcd.h"
#include "terreslam/utils/util_algebra.h"

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

namespace terreslam
{

class PlaneDetectorFrontend : public terreslam::Frontend
{
public:
	PlaneDetectorFrontend() : 
	Frontend(),
		queue_size_(1)
		{
			std::cout << "Constructor plane_detector_frontend..." << std::endl;
		}

private:

	void onInit()
	{
		std::cout << "Initalize plane_detector_frontend..." << std::endl;
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();
		
		ros::NodeHandle cloud_nh(nh, "cloud");
		std::string subscribedTopicsMsg;

		/// Subscribers
		cloud_nh.subscribe(cloud_frame_id, queue_size_, &PlaneDetectorFrontend::callback, this);
	} 

	void callback(
		const sensor_msgs::PointCloud2ConstPtr& cloud_msg_ptr)
	{
		std::cout << "Entry plane: " << entry_count_ << std::endl;
		// std::cout << cloud_msg_ptr->header.stamp << std::endl;

		ptrPointCloud points = ptrPointCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		ptrNormalCloud normals = ptrNormalCloud (new pcl::PointCloud<pcl::Normal>);
		sensor_msgs::PointCloud2 cloud_msg = *cloud_msg_ptr;
		pcl::fromROSMsg(cloud_msg, *points);

		///NORMALS
		if(use_normal_integral_)
		{
			/// Generate the normal_cloud
			/// More methods --> AVERAGE_3D_GRADIENT; AVERAGE_DEPTH_CHANGE; COVARIANCE_MATRIX
			pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne_integral;
			ne_integral.setNormalEstimationMethod(pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA,pcl::Normal>::AVERAGE_DEPTH_CHANGE);
			ne_integral.setDepthDependentSmoothing(true);
			ne_integral.setNormalSmoothingSize(40.0);
			ne_integral.setInputCloud(points);
			ne_integral.compute(*normals);
		}
		else
		{
			pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
			pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree;
			ne.setInputCloud(points);
			tree=pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGBA>());
			ne.setSearchMethod(tree);
			ne.setRadiusSearch(1);
			ne.compute(*normals);
		}

		/// Eliminate points with low curvature
		util::curvatureFilter(scan_, 0.08);

		/// PLANE DETECTOR
		// PD->detectPlanes(scan_);

		/// Visualize normals
		// Vis_.NormalView1(points, normals);

		/// Write normals
		// if(entry_count_ == 0) Disk.WriteNormals(points, normals);		

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
		msg_pcd.header.frame_id = cloud_frame_id+"/filtered";
		msg_pcd.header.stamp = cloud_msg.header.stamp;
		cloud_pub.publish(msg_pcd);

		// /// - Planes
		// pcl::toROSMsg(*scan_planes, msg_pcd);
		// msg_pcd.header.frame_id = cloud_frame_id+"/plane";
		// plane_pub.publish(msg_pcd);

		entry_count_++;
	}

	void skipFrame(std::string msg)
	{
		std::cerr<<msg<<std::endl;
		entry_count_++;
	}

private:
	/// Constants
	const double depthScale = pow(2,16)/120;
	const float bad_point = std::numeric_limits<float>::quiet_NaN();

	/// Variables
	int queue_size_;
	int entry_count_ = 0;
	bool use_normal_integral_ = false;

};

PLUGINLIB_EXPORT_CLASS(terreslam::PlaneDetectorFrontend, nodelet::Nodelet);

}