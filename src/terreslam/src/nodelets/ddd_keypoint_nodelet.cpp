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

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

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

		blob_odom_sub_ = nh.subscribe(blob_odom_topic, queue_size_, &DDDKeypointNodelet::updateBlobHeadingVel, this);

		/// Publishers
		ddd_keypoint_matches_pub_ = nh.advertise<terreslam::KeyPointMatches>(ddd_keypoint_matches_topic, 1);

		/// Motion filter
		possible_blob_heading_vel = -max_rollaxis_vel;

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

		ros::Time cur_stamp = cf_msg_ptr->header.stamp;

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
			old_time = cur_stamp.toSec();
			old_kpts=cur_kpts;
			old_desc=cur_desc;
			reset=false;
			entry_count++;
			return;
		}
		//Attempt matching
		delta_time = cur_stamp.toSec() - old_time;
		old_time = cur_stamp.toSec();

		///Motion Filter
		valid_blob_velocity = forward ? std::max(possible_blob_heading_vel, -max_rollaxis_vel)
																	: std::min(possible_blob_heading_vel, max_rollaxis_vel);
		if(!forward) {forward=true; valid_blob_velocity*=-1;} //Force forward
		valid_abs_displacement = abs(valid_blob_velocity*delta_time);
		// cout<<"Velocite: "<<valid_blob_velocity<<endl;
		// cout<<"Displacement: "<<valid_abs_displacement<<endl;

		//For each src kpt find valid dest kpts
		// - filter by forward or backward direction
		// - filter by distance
		// - filter by max steering angle
		// - apply correspondence estimator
		// - append correspondences with point idx
		matches_tmp_lot = pcl::CorrespondencesPtr(new pcl::Correspondences());
		matches_tmp = pcl::CorrespondencesPtr(new pcl::Correspondences());
		old_valid_desc = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>());
		cur_valid_desc = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>());
		int cur_idx, old_idx = -1;
		for(const auto& old_kpt : *old_kpts)
		{
			old_idx++;
			old_valid_desc->points.clear();
			cur_valid_desc->points.clear();
			cur_valid_idx.clear();
			matches_tmp_lot->clear();
			cur_idx = -1;
			old_valid_desc->points.push_back(old_desc->points.at(old_idx));
			for(const auto& cur_kpt : *cur_kpts)
			{
				cur_idx++;
				float direction = -1*(cur_kpt.z-old_kpt.z);
				if(forward && direction < 0) continue;
				if(!forward && direction > 0) continue;
				float displacement = sqrt(pow(old_kpt.x-cur_kpt.x,2)+pow(old_kpt.y-cur_kpt.y,2)+pow(old_kpt.z-cur_kpt.z,2));
				if(displacement > valid_abs_displacement) continue;
				float angle;
				if(forward) angle = atan2(old_kpt.z-cur_kpt.z,old_kpt.x-cur_kpt.x);
				if(!forward) angle = atan2(cur_kpt.z-old_kpt.z,cur_kpt.x-old_kpt.x);
				if(angle < DEG2RAD*(90-max_steering_angle) || angle > DEG2RAD*(90+max_steering_angle)) continue;
				cur_valid_desc->points.push_back(cur_desc->points.at(cur_idx));
				cur_valid_idx.push_back(cur_idx);
			}
			if(cur_valid_desc->points.size() < 1) continue;
			pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
			est.setInputSource(old_valid_desc);
			est.setInputTarget(cur_valid_desc);
			est.determineCorrespondences(*matches_tmp_lot);
			// est.determineReciprocalCorrespondences(*matches_tmp_lot);

			for(const auto& match : *matches_tmp_lot)
				matches_tmp->push_back(pcl::Correspondence(old_idx,cur_valid_idx.at(match.index_match),match.distance));
		}

		// std::cout<<"Number of matches: "<<matches_tmp->size()<<std::endl;

		// Delete one to many matches
		matches = pcl::CorrespondencesPtr(new pcl::Correspondences());
		pcl::registration::CorrespondenceRejectorOneToOne rejector_one_to_one;
		rejector_one_to_one.setInputCorrespondences(matches_tmp);
		rejector_one_to_one.getCorrespondences(*matches);

		// std::cout<<"Number of filtered matches: "<<matches->size()<<std::endl;
		// util::tick_high_resolution(start_t, tick, elapsed_matching);
		
		/// PRE-PUBLISH
		size_t sm = matches->size();
		std::vector<float> x_old(sm), y_old(sm), z_old(sm);
		std::vector<float> x_cur(sm), y_cur(sm), z_cur(sm);
		for(int i=0; i<sm; ++i)
		{
			x_old[i] = old_kpts->at(matches->at(i).index_query).x;
			y_old[i] = old_kpts->at(matches->at(i).index_query).y;
			z_old[i] = old_kpts->at(matches->at(i).index_query).z;
			
			x_cur[i] = cur_kpts->at(matches->at(i).index_match).x;
			y_cur[i] = cur_kpts->at(matches->at(i).index_match).y;
			z_cur[i] = cur_kpts->at(matches->at(i).index_match).z;
		}
		
		/// PUBLISH
		/// - KeyPoint Matches
		terreslam::KeyPointMatchesPtr kpm_msg_ptr(new terreslam::KeyPointMatches);
		kpm_msg_ptr->header.frame_id = ddd_keypoint_matches_frame_id;
		kpm_msg_ptr->header.stamp = cf_msg_ptr->header.stamp;
		kpm_msg_ptr->x_old = x_old;
		kpm_msg_ptr->y_old = y_old;
		kpm_msg_ptr->z_old = z_old;
		kpm_msg_ptr->x_cur = x_cur;
		kpm_msg_ptr->y_cur = y_cur;
		kpm_msg_ptr->z_cur = z_cur;
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

	void updateBlobHeadingVel(
		const nav_msgs::Odometry::ConstPtr& o_msg_ptr)
	{
		float blob_vel_x = o_msg_ptr->twist.twist.linear.x;
		float blob_vel_z = o_msg_ptr->twist.twist.linear.z;
		float blob_vel_x_var = o_msg_ptr->twist.covariance.at(0);
		float blob_vel_z_var = o_msg_ptr->twist.covariance.at(14);
		
		float possible_blob_vel_x = abs(blob_vel_x)+2*sqrt(blob_vel_x_var)+max_pitchaxis_acc*delta_time;
		float possible_blob_vel_z = abs(blob_vel_z)+2*sqrt(blob_vel_z_var)+max_rollaxis_acc*delta_time;
		
		forward = blob_vel_z <= 0 ? true : false;
		possible_blob_heading_vel = sqrt(pow(possible_blob_vel_x,2)+pow(possible_blob_vel_z,2));
		if(forward) possible_blob_heading_vel *= -1;

		// cout<<"UPDATE --> blob_vel_x: "<<blob_vel_x<<endl;
		// cout<<"UPDATE --> blob_vel_z: "<<blob_vel_z<<endl;
		// cout<<"UPDATE --> blob_vel_x_var: "<<blob_vel_x_var<<endl;
		// cout<<"UPDATE --> blob_vel_z_var: "<<blob_vel_z_var<<endl;
		// cout<<"UPDATE --> possible_blob_vel_x: "<<possible_blob_vel_x<<endl;
		// cout<<"UPDATE --> possible_blob_vel_z: "<<possible_blob_vel_z<<endl;
		// cout<<"UPDATE --> max_blob_vel_x: "<<max_pitchaxis_acc*delta_time<<endl;
		// cout<<"UPDATE --> max_blob_vel_z: "<<max_rollaxis_acc*delta_time<<endl;
		// cout<<"UPDATE --> Velocity: "<<possible_blob_heading_vel<<endl;
		// cout<<endl;
	}

private:

	/// General variables
	int queue_size_;

	/// Comms
	ros::Subscriber blob_odom_sub_;
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

	/// Blob motion filter
	bool forward = true;
	float valid_blob_velocity;
	float valid_abs_displacement;
	float possible_blob_heading_vel;

	/// Timming
	float delta_time = -1;
	float old_time = -1;
	
	/// Kpts
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr old_kpts;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cur_kpts;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr old_desc;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cur_desc;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr old_valid_desc;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cur_valid_desc;
	std::vector<uint> cur_valid_idx;
	pcl::CorrespondencesPtr matches;
	pcl::CorrespondencesPtr matches_tmp;
	pcl::CorrespondencesPtr matches_tmp_lot;
	bool reset=false;

};

PLUGINLIB_EXPORT_CLASS(terreslam::DDDKeypointNodelet, nodelet::Nodelet);

}