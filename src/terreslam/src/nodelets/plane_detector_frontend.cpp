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
		// cloud_sub = cloud_nh.subscribe(cloud_frame_id, queue_size_, &PlaneDetectorFrontend::callback, this);
		
		cloud_sub_filter_.subscribe(cloud_nh, cloud_frame_id, 1);
		cloud_xy_sub_filter_.subscribe(cloud_nh, cloud_xy_frame_id, 1);

		exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>
			(MyExactSyncPolicy(queue_size_), cloud_sub_filter_, cloud_xy_sub_filter_);
		exactSync_->registerCallback(boost::bind(&PlaneDetectorFrontend::callback, this, _1, _2));

		// Publishers
		cloud_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_filtered_frame_id, 10);
		plane_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_plane_frame_id, 10);
	} 

	void callback(
		const sensor_msgs::PointCloud2ConstPtr& cloud_msg_ptr,
		const sensor_msgs::PointCloud2ConstPtr& cloud_xy_msg_ptr)
	{
		std::cout << "Entry plane: " << entry_count_ << std::endl;
		// ///Start chrono ticking
		// std::chrono::duration<double> tick;
		// std::chrono::high_resolution_clock::time_point end_t, start_t;
		// start_t = std::chrono::high_resolution_clock::now();
		// end_t = std::chrono::high_resolution_clock::now();
		// tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr points (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::PointXY>::Ptr pixels (new pcl::PointCloud<pcl::PointXY>);
		pcl::fromROSMsg(*cloud_msg_ptr, *points);
		pcl::fromROSMsg(*cloud_xy_msg_ptr, *pixels);

		///NORMALS
		if(use_normal_integral)
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

		// tick_high_resolution(start_t, tick, elapsed_normal);

		/// Eliminate points with low curvature
		util::curvatureFilter(points, normals, PF_thresh, PF_highpass);

		/// PLANE DETECTOR
		// scan_ = new Scan();
		// scan_->points()->clear();
		// scan_->normals()->clear();
		// scan_->pixels()->clear();
		// scan_->points() = points;
		// scan_->normals() = normals;
		// scan_->pixels() = pixels;
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
		msg_pcd.header.stamp = cloud_msg_ptr->header.stamp;
		cloud_filtered_pub.publish(msg_pcd);

		/// - Planes
		// pcl::toROSMsg(*scan_planes, msg_pcd);
		// msg_pcd.header.frame_id = cloud_plane_frame_id;
		// plane_pub.publish(msg_pcd);

		entry_count_++;

		// tick_high_resolution(start_t, tick, elapsed_filter);
		// printElapsed(elapsed_normal, "Callback plane normal: ");
		// printElapsed(elapsed_filter, "Callback plane filter: ");
	}

	void skipFrame(std::string msg)
	{
		std::cerr<<msg<<std::endl;
		entry_count_++;
	}

private:

	/// General variables
	int queue_size_;
	int entry_count_ = 0;

	///Chrono timmings
	std::vector<double> elapsed_normal;
	std::vector<double> elapsed_filter;

	// blocks
	std::unique_ptr<PlaneDetector> PD;

	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_filter_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_xy_sub_filter_;

	typedef message_filters::sync_policies::ExactTime
		<sensor_msgs::PointCloud2,
		sensor_msgs::PointCloud2> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;

};

PLUGINLIB_EXPORT_CLASS(terreslam::PlaneDetectorFrontend, nodelet::Nodelet);

}