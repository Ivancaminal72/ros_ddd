/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-08-19 17:43:50
 */

#include "terreslam/nodelet.h"
#include "terreslam/features/ddd_keypoint_detector.h"
#include "terreslam/processings/ddd_keypoint_processor.h"
#include "terreslam/utils/util_pcd.h"
#include "terreslam/utils/util_chrono.h"
#include "terreslam/utils/util_general.h"

#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <math.h>
#include <fstream>

#include <Eigen/Dense>

//msgs
#include <terreslam/KeyPointMatches.h>

namespace terreslam
{

class DDDKeypointNodelet : public terreslam::Nodelet
{
public:
	DDDKeypointNodelet() :
		queue_size_(10)
		{
			// std::cout << "Constructor ddd_keypoint_nodelet..." << std::endl;
		}

private:

	void onNodeletInit()
	{
		std::cout << "Initalize ddd_keypoint_nodelet..." << std::endl;
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		/// Subscribers
		cloud_filtered_sub_filter_.subscribe(nh, cloud_filtered_topic, 1);
		normal_filtered_sub_filter_.subscribe(nh, normal_filtered_topic, 1);

		exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>
			(MyExactSyncPolicy(queue_size_), cloud_filtered_sub_filter_, normal_filtered_sub_filter_);
		exactSync_->registerCallback(boost::bind(&DDDKeypointNodelet::callback, this, _1, _2));

		// Publishers
		ddd_keypoint_matches_pub_ = nh.advertise<terreslam::KeyPointMatches>(ddd_keypoint_matches_topic, 1);

	} 

	void callback(
		const sensor_msgs::PointCloud2::ConstPtr& cf_msg_ptr,
		const sensor_msgs::PointCloud2::ConstPtr& nf_msg_ptr)
	{
		if(debug) std::cout << "Entry DDD KP: " << entry_count << std::endl;
		// ///Start chrono ticking
		// std::chrono::duration<double> tick;
		// std::chrono::high_resolution_clock::time_point end_t, start_t;
		// start_t = std::chrono::high_resolution_clock::now();
		// end_t = std::chrono::high_resolution_clock::now();
		// tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr points (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		pcl::fromROSMsg(*cf_msg_ptr, *points);
		pcl::fromROSMsg(*nf_msg_ptr, *normals);
		// util::tick_high_resolution(start_t, tick, elapsed_initialization);

		/// 3D KEYPOINT DETECTION & DESCRIPTION
		// std::cout<<"Number of points: "<<points->size()<<std::endl;
		cur_kpts = detectSIFTKeypoints(points, normals, DDDKP_SIFT_min_scale, DDDKP_SIFT_nr_octaves, DDDKP_SIFT_nr_scales_per_octave, DDDKP_SIFT_min_contrast);
		// util::tick_high_resolution(start_t, tick, elapsed_KP_detection);
		cur_desc = computeFPFHDescriptors(points, normals, cur_kpts, DDDKP_FPFH_radius);
		// util::tick_high_resolution(start_t, tick, elapsed_KP_description);

		// std::cout<<"Number of current kps: "<<cur_kpts->size()<<std::endl;

		///MATCHING
		if(entry_count == 0 || reset) //First time initialize map
		{
			old_kpts=cur_kpts;
			old_desc=cur_desc;
			reset=false;
			entry_count++;
			return;
		}
		else //Attempt matching
		{
			pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
			matches_tmp = pcl::CorrespondencesPtr(new pcl::Correspondences());
			est.setInputSource(old_desc);
			est.setInputTarget(cur_desc);
			est.determineCorrespondences(*matches_tmp);
			// est.determineReciprocalCorrespondences(*matches_tmp);

			// std::cout<<"Number of matches: "<<matches_tmp->size()<<std::endl;

			// Delete one to many matches
			matches = pcl::CorrespondencesPtr(new pcl::Correspondences());
			pcl::registration::CorrespondenceRejectorOneToOne rejector_one_to_one;
			rejector_one_to_one.setInputCorrespondences(matches_tmp);
			rejector_one_to_one.getCorrespondences(*matches);

			// std::cout<<"Number of filtered matches: "<<matches->size()<<std::endl;
		}
		// util::tick_high_resolution(start_t, tick, elapsed_matching);
		
		/// PRE-PUBLISH
		size_t sm = matches->size();
		std::vector<float> x_cur(sm), y_cur(sm), z_cur(sm);
		std::vector<float> x_old(sm), y_old(sm), z_old(sm);
		for(int i=0; i<sm; ++i)
		{
			x_cur[i] = cur_kpts->at(matches->at(i).index_match).x;
			y_cur[i] = cur_kpts->at(matches->at(i).index_match).y;
			z_cur[i] = cur_kpts->at(matches->at(i).index_match).z;

			x_old[i] = old_kpts->at(matches->at(i).index_query).x;
			y_old[i] = old_kpts->at(matches->at(i).index_query).y;
			z_old[i] = old_kpts->at(matches->at(i).index_query).z;
		}
		
		/// PUBLISH
		/// - KeyPoint Matches
		terreslam::KeyPointMatchesPtr kpm_msg_ptr(new terreslam::KeyPointMatches);
		kpm_msg_ptr->header.frame_id = ddd_keypoint_matches_frame_id;
		kpm_msg_ptr->header.stamp = cf_msg_ptr->header.stamp;
		kpm_msg_ptr->x_cur = x_cur;
		kpm_msg_ptr->y_cur = y_cur;
		kpm_msg_ptr->z_cur = z_cur;
		kpm_msg_ptr->x_old = x_old;
		kpm_msg_ptr->y_old = y_old;
		kpm_msg_ptr->z_old = z_old;
		ddd_keypoint_matches_pub_.publish(kpm_msg_ptr);

		//Update old data
		old_kpts=cur_kpts;
		old_desc=cur_desc;		
		
		entry_count++;

		// util::tick_high_resolution(start_t, tick, elapsed);
		// util::printElapsed(elapsed, "Callback ddd_keypoint: ");
		// util::tick_high_resolution(start_t, tick, elapsed_publish);
		// util::printElapsed(elapsed_initialization, "Initialization: ");
		// util::printElapsed(elapsed_KP_detection, "KP_detection: ");
		// util::printElapsed(elapsed_KP_description, "KP_description: ");
		// util::printElapsed(elapsed_matching, "Matching: ");
		// util::printElapsed(elapsed_publish, "Publish: ");
	}

private:

	/// General variables
	int queue_size_;

	/// Comms
	ros::Publisher ddd_keypoint_matches_pub_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_filtered_sub_filter_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> normal_filtered_sub_filter_;

	typedef message_filters::sync_policies::ExactTime
		<sensor_msgs::PointCloud2,
		sensor_msgs::PointCloud2> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy>* exactSync_;

	/// Chrono timmings
	// std::vector<double> elapsed;
	std::vector<double> elapsed_initialization;
	std::vector<double> elapsed_KP_detection;
	std::vector<double> elapsed_KP_description;
	std::vector<double> elapsed_matching;
	std::vector<double> elapsed_publish;

	///Kpts
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cur_kpts;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr old_kpts;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cur_desc;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr old_desc;
	pcl::CorrespondencesPtr matches;
	pcl::CorrespondencesPtr matches_tmp;
	bool reset=false;

};

PLUGINLIB_EXPORT_CLASS(terreslam::DDDKeypointNodelet, nodelet::Nodelet);

}