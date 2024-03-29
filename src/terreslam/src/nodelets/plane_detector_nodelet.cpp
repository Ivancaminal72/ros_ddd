/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-19 11:47:07
 */

#include "terreslam/nodelet.h"
#include "terreslam/features/plane_detector.h"
#include "terreslam/utils/util_pcd.h"
#include "terreslam/utils/util_chrono.h"

#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/normal_space.h>

namespace terreslam
{

class PlaneDetectorNodelet : public terreslam::Nodelet
{
public:
	PlaneDetectorNodelet() :
		queue_size_(10)
		{
			// std::cout << "Constructor plane_detector_nodelet..." << std::endl;
		}

private:

	void onNodeletInit()
	{
		std::cout << "Initalize plane_detector_nodelet..." << std::endl;
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

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
		cloud_sub = nh.subscribe(cloud_topic, queue_size_, &PlaneDetectorNodelet::callback, this);

		// Publishers
		cloud_filtered_high_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cloud_filtered_high_topic, 10);
		old_cloud_filtered_low_pub_ = nh.advertise<sensor_msgs::PointCloud2>(old_cloud_filtered_low_topic, 10);
		cloud_filtered_low_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cloud_filtered_low_topic, 10);
		normal_filtered_high_pub_ = nh.advertise<sensor_msgs::PointCloud2>(normal_filtered_high_topic, 10);
		plane_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cloud_plane_topic, 10);
	} 

	void callback(
		const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr)
	{
		if(debug) std::cout << "Entry plane: " << entry_count << std::endl;
		// ///Start chrono ticking
		// std::chrono::duration<double> tick;
		// std::chrono::high_resolution_clock::time_point end_t, start_t;
		// start_t = std::chrono::high_resolution_clock::now();
		// end_t = std::chrono::high_resolution_clock::now();
		// tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr points (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		// pcl::PointCloud<pcl::PointXY>::Ptr pixels (new pcl::PointCloud<pcl::PointXY>);
		pcl::fromROSMsg(*cloud_msg_ptr, *points);

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
			pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> ne;
			ne.setNumberOfThreads(6);
			pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree;
			ne.setInputCloud(points);
			tree=pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGBA>());
			ne.setSearchMethod(tree);
			ne.setRadiusSearch(1);
			ne.compute(*normals);
		}

		// util::tick_high_resolution(start_t, tick, elapsed_normal);

		/// Eliminate points with low curvature
		cur_high_points = ptrPointCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		cur_high_normals = ptrNormalCloud (new pcl::PointCloud<pcl::Normal>);
		cur_low_points = ptrPointCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		cur_low_normals = ptrNormalCloud (new pcl::PointCloud<pcl::Normal>);
		size_t fivepercent = points->size() * 0.05f;
		float min_step = 0.01;
		float increment = min_step;
		float PF_thresh_high_opt = PF_thresh_high;
		size_t old_size, reduction, remain;
		
		util::curvatureFilter(points, normals, cur_high_points, cur_high_normals, NULL, NULL, PF_thresh_high);
		util::curvatureFilter(points, normals, NULL, NULL, cur_low_points, cur_low_normals, PF_thresh_low);
		// util::distanceFilter(cur_low_points, cur_low_normals, 20);

		/// Optimize elimination
		if(cur_high_points->size() >= fivepercent)
		{
			old_size = cur_high_points->size();
			PF_thresh_high_opt += increment;
			util::curvatureFilter(points, normals, cur_high_points, cur_high_normals, NULL, NULL, PF_thresh_high_opt);
		}

		while(cur_high_points->size() >= fivepercent)
		{
			remain = cur_high_points->size() - fivepercent;
			reduction = old_size - cur_high_points->size();
			increment = remain * increment / reduction;
			increment = (increment > min_step) ? increment : min_step;
			old_size = cur_high_points->size();
			PF_thresh_high_opt += increment;
			util::curvatureFilter(points, normals, cur_high_points, cur_high_normals, NULL, NULL, PF_thresh_high_opt);
		}

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
		// if(entry_count == 0) Disk.WriteNormals(points, normals);		

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
		/// - Pairwise scans
		if(entry_count > 0)
		{
			//subsample old
			// pcl::NormalSpaceSampling<pcl::PointXYZRGBA, pcl::Normal> nss;
			// nss.setInputCloud(old_low_points);
			// nss.setNormals(old_low_normals);
			// nss.setSample((unsigned int) points->size() * 0.005f);
			// nss.setSeed(std::rand());
			// nss.setBins(2,2,2);
			// nss.filter(*old_low_points);

			pcl::toROSMsg(*old_low_points, msg_pcd);
			msg_pcd.header.frame_id = old_cloud_filtered_low_frame_id;
			msg_pcd.header.stamp = cloud_msg_ptr->header.stamp;
			old_cloud_filtered_low_pub_.publish(msg_pcd);

			pcl::toROSMsg(*cur_low_points, msg_pcd);
			msg_pcd.header.frame_id = cloud_filtered_low_frame_id;
			msg_pcd.header.stamp = cloud_msg_ptr->header.stamp;
			cloud_filtered_low_pub_.publish(msg_pcd);
		} 

		/// - Cloud Filtered
		pcl::toROSMsg(*cur_high_points, msg_pcd);
		msg_pcd.header.frame_id = cloud_filtered_high_frame_id;
		msg_pcd.header.stamp = cloud_msg_ptr->header.stamp;
		cloud_filtered_high_pub_.publish(msg_pcd);

		/// - Normal Filtered
		pcl::toROSMsg(*cur_high_normals, msg_pcd);
		msg_pcd.header.frame_id = normal_filtered_high_frame_id;
		msg_pcd.header.stamp = cloud_msg_ptr->header.stamp;
		normal_filtered_high_pub_.publish(msg_pcd);

		/// - Planes
		// pcl::toROSMsg(*scan_planes, msg_pcd);
		// msg_pcd.header.frame_id = cloud_plane_frame_id;
		// plane_pub.publish(msg_pcd);

		//Update old data
		old_high_points = cur_high_points;
		old_low_points = cur_low_points;
		old_high_normals = cur_high_normals;
		old_low_normals = cur_low_normals;

		entry_count++;

		// util::tick_high_resolution(start_t, tick, elapsed_filter);
		// util::printElapsed(elapsed_normal, "Callback plane normal: ");
		// util::printElapsed(elapsed_filter, "Callback plane filter: ");
	}

	void skipFrame(std::string msg)
	{
		std::cerr<<msg<<std::endl;
		entry_count++;
	}

private:

	/// General variables
	int queue_size_;

	/// Comms
	sensor_msgs::PointCloud2 msg_pcd;
	ros::Subscriber cloud_sub;
	ros::Publisher cloud_filtered_high_pub_;
	ros::Publisher old_cloud_filtered_low_pub_;
	ros::Publisher cloud_filtered_low_pub_;
	ros::Publisher normal_filtered_high_pub_;
	ros::Publisher plane_pub_;

	/// Chrono timmings
	std::vector<double> elapsed_normal;
	std::vector<double> elapsed_filter;

	// blocks
	std::unique_ptr<PlaneDetector> PD;

	/// Types
	Scan* scan_;

	/// Plane Filtering
	ptrPointCloud old_high_points;
	ptrPointCloud old_low_points;
	ptrPointCloud cur_high_points;
	ptrPointCloud cur_low_points;
	ptrNormalCloud old_high_normals;
	ptrNormalCloud old_low_normals;
	ptrNormalCloud cur_high_normals;
	ptrNormalCloud cur_low_normals;
};

PLUGINLIB_EXPORT_CLASS(terreslam::PlaneDetectorNodelet, nodelet::Nodelet);

}