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
		queue_size_(10)
		{
			std::cout << "Constructor plane_detector_frontend..." << std::endl;
		}

private:

	void onFrontendInit()
	{
		std::cout << "Initalize plane_detector_frontend..." << std::endl;
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();
		ros::NodeHandle cloud_nh(nh, "cloud");

		PD = std::make_unique<PlaneDetector> (PD_debug,
																					PD_theta,
																					PD_phi,
																					PD_d,
																					PD_max_plane,
																					PD_min_plane_size,
																					PD_thres_angle,
																					PD_thres_dist,
																					max_depth,
																					logs_dir);

		/// Subscribers
		cloud_sub = cloud_nh.subscribe(cloud_frame_id, queue_size_, &PlaneDetectorFrontend::callback, this);

		// Publishers
		cloud_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_filtered_frame_id, 10);
		plane_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_plane_frame_id, 10);
	} 

	void callback(
		const sensor_msgs::PointCloud2ConstPtr& cloud_msg_ptr)
	{
		std::cout << "Entry plane: " << entry_count_ << std::endl;
		// std::cout << cloud_msg_ptr->header.stamp << std::endl;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr points (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		const sensor_msgs::PointCloud2 cloud_msg = *cloud_msg_ptr;
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
		util::curvatureFilter(points, normals, PF_thresh, PF_highpass);

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
		msg_pcd.header.frame_id = cloud_filtered_frame_id;
		msg_pcd.header.stamp = cloud_msg.header.stamp;
		cloud_filtered_pub.publish(msg_pcd);

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

	/// General variables
	int queue_size_;
	int entry_count_ = 0;
	bool use_normal_integral_ = false;

	// blocks
	std::unique_ptr<PlaneDetector> PD;

};

PLUGINLIB_EXPORT_CLASS(terreslam::PlaneDetectorFrontend, nodelet::Nodelet);

}