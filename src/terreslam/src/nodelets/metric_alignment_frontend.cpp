/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-06-15 11:37:46
 */

#include "terreslam/frontend.h"
#include "terreslam/utils/util_chrono.h"
#include "terreslam/registrations/dd_coarse_alignment.h"

#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <nav_msgs/Odometry.h>

//msgs
#include <terreslam/BlobMatches.h>
#include <terreslam/KeyPointMatches.h>

namespace terreslam
{

class MetricAlignmentFrontend : public terreslam::Frontend
{
public:
	MetricAlignmentFrontend() :
		queue_size_(10)
		{
			// std::cout << "Constructor metric_alignment_frontend..." << std::endl;
		}

private:

	void onFrontendInit()
	{
		std::cout << "Initalize metric_alignment_frontend..." << std::endl;
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();
		ros::NodeHandle bm_nh(nh, "blob_matches");
		ros::NodeHandle kpm_nh(nh, "keypoint_matches");

		/// Subscribers
		blob_matches_sub_filter_.subscribe(bm_nh, blob_matches_frame_id, 1);
		dd_keypoint_matches_sub_filter_.subscribe(kpm_nh, dd_keypoint_matches_frame_id, 1);
		ddd_keypoint_matches_sub_filter_.subscribe(kpm_nh, ddd_keypoint_matches_frame_id, 1);

		exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>
			(MyExactSyncPolicy(queue_size_), blob_matches_sub_filter_, dd_keypoint_matches_sub_filter_, ddd_keypoint_matches_sub_filter_);
		exactSync_->registerCallback(boost::bind(&MetricAlignmentFrontend::callback, this, _1, _2,_3));

		// Publishers
		cloud_keypoints_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cloud_keypoints_frame_id, 10);
		odom_pub_ = nh.advertise<nav_msgs::Odometry>(odom_frame_id, 1);
	} 

	void callback(
		const terreslam::BlobMatches::ConstPtr& bm_msg_ptr,
		const terreslam::KeyPointMatches::ConstPtr& dd_kpm_msg_ptr,
		const terreslam::KeyPointMatches::ConstPtr& ddd_kpm_msg_ptr)
	{
		std::cout << "Entry MA: " << entry_count << std::endl;
		// ///Start chrono ticking
		// std::chrono::duration<double> tick;
		// std::chrono::high_resolution_clock::time_point end_t, start_t;
		// start_t = std::chrono::high_resolution_clock::now();
		// end_t = std::chrono::high_resolution_clock::now();
		// tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);

		// pcl::PointCloud<pcl::PointXYZRGBA>::Ptr points (new pcl::PointCloud<pcl::PointXYZRGBA>);
		// pcl::fromROSMsg(*cf_msg_ptr, *points);
		size_t sm = bm_msg_ptr->x_cur.size();
		assert(sm == bm_msg_ptr->stability.size() &&
					 sm == bm_msg_ptr->z_cur.size() &&
					 sm == bm_msg_ptr->radius_cur.size() &&
					 sm == bm_msg_ptr->height_cur.size() &&
					 sm == bm_msg_ptr->x_old.size() &&
					 sm == bm_msg_ptr->z_old.size() &&
					 sm == bm_msg_ptr->radius_old.size() &&
					 sm == bm_msg_ptr->height_old.size());
		std::vector<uint8_t> stability(sm);
		std::vector<float> radius_cur(sm), height_cur(sm);
		std::vector<float> radius_old(sm), height_old(sm);
		std::vector<cv::Point2f> bm_cur(sm), bm_old(sm);
		stability = bm_msg_ptr->stability;
		radius_cur = bm_msg_ptr->radius_cur;
		height_cur = bm_msg_ptr->height_cur;
		radius_old = bm_msg_ptr->radius_old;
		height_old = bm_msg_ptr->height_old;
		for(i=0; i<sm; ++i)
		{
			bm_cur.at(i) = cv::Point2f(bm_msg_ptr->x_cur.at(i), bm_msg_ptr->z_cur.at(i));
			bm_old.at(i) = cv::Point2f(bm_msg_ptr->x_old.at(i), bm_msg_ptr->z_old.at(i));
		}

		size_t dd_sm  = dd_kpm_msg_ptr->x_cur.size();
		assert(dd_sm == dd_kpm_msg_ptr->y_cur.size() &&
					 dd_sm == dd_kpm_msg_ptr->z_cur.size() &&
					 dd_sm == dd_kpm_msg_ptr->x_old.size() &&
					 dd_sm == dd_kpm_msg_ptr->y_old.size() &&
					 dd_sm == dd_kpm_msg_ptr->z_old.size());
		std::vector<cv::Point3f> dd_kpm_cur(dd_sm), dd_kpm_old(dd_sm);
		for(i=0; i<dd_sm; ++i)
		{
			dd_kpm_cur.at(i) = cv::Point3f(dd_kpm_msg_ptr->x_cur.at(i), dd_kpm_msg_ptr->y_cur.at(i), dd_kpm_msg_ptr->z_cur.at(i));
			dd_kpm_old.at(i) = cv::Point3f(dd_kpm_msg_ptr->x_old.at(i), dd_kpm_msg_ptr->y_old.at(i), dd_kpm_msg_ptr->z_old.at(i));
		}

		size_t ddd_sm = ddd_kpm_msg_ptr->x_cur.size();
		assert(ddd_sm == ddd_kpm_msg_ptr->y_cur.size() &&
					 ddd_sm == ddd_kpm_msg_ptr->z_cur.size() &&
					 ddd_sm == ddd_kpm_msg_ptr->x_old.size() &&
					 ddd_sm == ddd_kpm_msg_ptr->y_old.size() &&
					 ddd_sm == ddd_kpm_msg_ptr->z_old.size());
		std::vector<cv::Point3f> ddd_kpm_cur(ddd_sm), ddd_kpm_old(ddd_sm);
		for(i=0; i<ddd_sm; ++i)
		{
			ddd_kpm_cur.at(i) = cv::Point3f(ddd_kpm_msg_ptr->x_cur.at(i), ddd_kpm_msg_ptr->y_cur.at(i), ddd_kpm_msg_ptr->z_cur.at(i));
			ddd_kpm_old.at(i) = cv::Point3f(ddd_kpm_msg_ptr->x_old.at(i), ddd_kpm_msg_ptr->y_old.at(i), ddd_kpm_msg_ptr->z_old.at(i));
		}


		///METRIC ALIGNMENT
		float best_params[3] = {0.0f};
		bool inliers[sm] = {true};
		fit3DofRANSAC(bm_old, bm_cur, best_params, inliers, cv::Point2d(0,0), 0.1, 2*sm);

		/// Pre-PUBLISH
		pcl::PointCloud<pcl::PointXYZRGBA> cloud;
		cloud.points.resize (dd_kpm_cur.size()+dd_kpm_old.size()+ddd_kpm_cur.size()+ddd_kpm_old.size());
		uint8_t a=255,r,g,b;
		uint32_t rgba;
		r = 0; g = 255; b = 255;
		rgba = (a << 24 | r << 16 | g << 8 | b);
		for (size_t i=0; i<dd_kpm_cur.size(); ++i) {
			cloud.points[i].x = dd_kpm_cur[i].x;
			cloud.points[i].y = dd_kpm_cur[i].y;
			cloud.points[i].z = dd_kpm_cur[i].z;
			cloud.points[i].rgba = rgba;
		}
		r = 0; g = 255; b = 0;
		rgba = (a << 24 | r << 16 | g << 8 | b);
		for (size_t i=0; i<dd_kpm_old.size(); ++i) {
			cloud.points[i+dd_sm].x = dd_kpm_old[i].x;
			cloud.points[i+dd_sm].y = dd_kpm_old[i].y;
			cloud.points[i+dd_sm].z = dd_kpm_old[i].z;
			cloud.points[i+dd_sm].rgba = rgba;
		}
		r = 255; g = 255; b = 0;
		rgba = (a << 24 | r << 16 | g << 8 | b);
		for (size_t i=0; i<ddd_kpm_cur.size(); ++i) {
			cloud.points[i+2*dd_sm].x = ddd_kpm_cur[i].x;
			cloud.points[i+2*dd_sm].y = ddd_kpm_cur[i].y;
			cloud.points[i+2*dd_sm].z = ddd_kpm_cur[i].z;
			cloud.points[i+2*dd_sm].rgba = rgba;
		}
		r = 255; g = 0; b = 0;
		rgba = (a << 24 | r << 16 | g << 8 | b);
		for (size_t i=0; i<ddd_kpm_old.size(); ++i) {
			cloud.points[i+2*dd_sm+ddd_sm].x = ddd_kpm_old[i].x;
			cloud.points[i+2*dd_sm+ddd_sm].y = ddd_kpm_old[i].y;
			cloud.points[i+2*dd_sm+ddd_sm].z = ddd_kpm_old[i].z;
			cloud.points[i+2*dd_sm+ddd_sm].rgba = rgba;
		}

		/// PUBLISH
		/// - Cloud Filtered
		sensor_msgs::PointCloud2 msg_pcd;
		pcl::toROSMsg(cloud, msg_pcd);
		msg_pcd.header.frame_id = cloud_keypoints_frame_id;
		msg_pcd.header.stamp = dd_kpm_msg_ptr->header.stamp;
		cloud_keypoints_pub_.publish(msg_pcd);

		entry_count++;

		// util::tick_high_resolution(start_t, tick, elapsed);
		// util::printElapsed(elapsed, "Callback blob detector: ");
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
	ros::Publisher cloud_keypoints_pub_;
	ros::Publisher odom_pub_;
	message_filters::Subscriber<terreslam::BlobMatches> blob_matches_sub_filter_;
	message_filters::Subscriber<terreslam::KeyPointMatches> dd_keypoint_matches_sub_filter_;
	message_filters::Subscriber<terreslam::KeyPointMatches> ddd_keypoint_matches_sub_filter_;
	
	typedef message_filters::sync_policies::ExactTime
		<terreslam::BlobMatches,
		terreslam::KeyPointMatches,
		terreslam::KeyPointMatches> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy>* exactSync_;

	///Chrono timmings
	std::vector<double> elapsed;

	/// Blobs
	unsigned int i,j;
};

PLUGINLIB_EXPORT_CLASS(terreslam::MetricAlignmentFrontend, nodelet::Nodelet);

}