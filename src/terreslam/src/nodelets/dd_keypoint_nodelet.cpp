/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-08-16 16:38:16
 */

#include "terreslam/nodelet.h"
#include "terreslam/camera_model.h"
#include "terreslam/features/dd_keypoint_detector.h"
#include "terreslam/processings/dd_keypoint_processor.h"
#include "terreslam/utils/util_map.h"
#include "terreslam/utils/util_pcd.h"
#include "terreslam/utils/util_chrono.h"

#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <Eigen/Geometry>

//msgs
#include <terreslam/KeyPointMatches.h>

namespace terreslam
{

class DDKeypointNodelet : public terreslam::Nodelet
{
public:
	DDKeypointNodelet() : 
	Nodelet(),
		queue_size_(10)
		{
			// std::cout << "Constructor dd_keypoint_nodelet..." << std::endl;
		}

private:

	virtual void onNodeletInit()
	{
		std::cout << "Initalize dd_keypoint_nodelet..." << std::endl;
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();
		
		std::string subscribedTopicsMsg;

		image_transport::ImageTransport rgb_it(nh);
		image_transport::ImageTransport depth_it(nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), pnh);
		image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), pnh);

		/// Subscribers
		rgb_sub_filter_.subscribe(rgb_it, sub_cam_topic, 1, hintsRgb);
		depth_sub_filter_.subscribe(depth_it, sub_cam_depth_topic, 1, hintsDepth);
		info_sub_filter_.subscribe(nh, sub_cam_info_topic, 1);

		exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>
			(MyExactSyncPolicy(queue_size_), rgb_sub_filter_, depth_sub_filter_, info_sub_filter_);
		exactSync_->registerCallback(boost::bind(&DDKeypointNodelet::callback, this, _1, _2, _3));

		blob_odom_sub_ = nh.subscribe(blob_odom_topic, queue_size_, &DDKeypointNodelet::updateBlobHeadingVel, this);

		/// Publishers
		dd_keypoint_matches_pub_ = nh.advertise<terreslam::KeyPointMatches>(dd_keypoint_matches_topic, 1);

		/// Motion filter
		max_possible_blob_heading_vel = -max_rollaxis_vel;
		min_possible_blob_heading_vel = 0;

	} 

	void callback(
		const sensor_msgs::Image::ConstPtr& rgb_msg,
		const sensor_msgs::Image::ConstPtr& depth_msg,
		const sensor_msgs::CameraInfo::ConstPtr& info_msg)
	{
		if(debug) std::cout << "Entry DD KP: " << entry_count << std::endl;
		///Start chrono ticking
		// std::chrono::duration<double> tick;
		// std::chrono::high_resolution_clock::time_point end_t, start_t;
		// start_t = std::chrono::high_resolution_clock::now();
		// end_t = std::chrono::high_resolution_clock::now();
		// tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);

		ros::Time cur_stamp = rgb_msg->header.stamp;

		cv_bridge::CvImage::ConstPtr ptr_msg_rgb = cv_bridge::toCvShare(rgb_msg);
		cv_bridge::CvImage::ConstPtr ptr_msg_depth = cv_bridge::toCvShare(depth_msg);
		sensor_msgs::CameraInfo info = *info_msg;

		/// INITALIZATION
		uint32_t height, width;
		if (rgb_msg->height == depth_msg->height) height = rgb_msg->height; else {skipFrame("Different img height"); return;}
		if (rgb_msg->width == depth_msg->width) width = rgb_msg->width; else {skipFrame("Different img width"); return;}
		cv::Mat img_rgb = cv::Mat(rgb_msg->height, rgb_msg->width, ptr_msg_rgb->image.type());
		cv::Mat img_depth = cv::Mat(depth_msg->height, depth_msg->width, ptr_msg_depth->image.type());
		ptr_msg_rgb->image.copyTo(cv::Mat(img_rgb, cv::Rect(0, 0, rgb_msg->width, rgb_msg->height)));
		ptr_msg_depth->image.copyTo(cv::Mat(img_depth, cv::Rect(0, 0, depth_msg->width, depth_msg->height)));


		/// 2D KEYPOINT DETECTION & DESCRIPTION
		cv::Mat img_gray;
		cv::cvtColor(img_rgb, img_gray, CV_BGR2GRAY);
		// util::tick_high_resolution(start_t, tick, elapsed_initialization);
		cur_kpts = detectGFTTKeyPoints(img_gray);
		// util::tick_high_resolution(start_t, tick, elapsed_KP_detection);
		cur_desc = computeBriefDescriptors(img_rgb, cur_kpts);
		// util::tick_high_resolution(start_t, tick, elapsed_KP_description);
		// std::cout<<"Original Keypoint size: "<<cur_kpts.size()<<std::endl;
		// std::cout<<"Original Descriptors size: "<<cur_desc.size()<<std::endl;

		/// PRE-BACKPROJECTION
		CameraModel cam_model(info);
		// cam_model.printModel();

		cur_P_inv = cam_model.P().inverse().matrix();
		// util::tick_high_resolution(start_t, tick, elapsed_cam_model);
		

		///MATCHING
		if(entry_count == 0 || reset) //First time initialize map
		{
			old_time = cur_stamp.toSec();
			old_P_inv=cur_P_inv;
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
		max_valid_blob_velocity = forward ? std::max(max_possible_blob_heading_vel, -max_rollaxis_vel)
																			: std::min(max_possible_blob_heading_vel, max_rollaxis_vel);
		min_valid_blob_velocity = forward ? std::min(min_possible_blob_heading_vel, 0.0f)
																			: std::max(min_possible_blob_heading_vel, 0.0f);
		if(!forward) {forward=true; max_valid_blob_velocity*=-1; min_valid_blob_velocity*=-1;} //Force forward
		max_valid_abs_displacement = abs(max_valid_blob_velocity*delta_time);
		min_valid_abs_displacement = abs(min_valid_blob_velocity*delta_time);
		// cout<<"Max velocite: "<<max_valid_blob_velocity<<endl;
		// cout<<"Min velocite: "<<min_valid_blob_velocity<<endl;
		// cout<<"Max displacement: "<<max_valid_abs_displacement<<endl;
		// cout<<"Min displacement: "<<min_valid_abs_displacement<<endl;
		
		/// BACKPROJECTION
		/// Backproject, if possible, all keypoints with depth value with lowest distance
		old_kpts_bkpj = backprojectKeypoints(old_kpts, old_P_inv, depthScale, img_depth, DDKP_ws);
		cur_kpts_bkpj = backprojectKeypoints(cur_kpts, old_P_inv, depthScale, img_depth, DDKP_ws);

		//For each src kpt find valid dest kpts
		// - filter by valid backprojection
		// - filter by forward or backward direction
		// - filter by distance
		// - filter by max steering angle
		// - apply correspondence estimator
		// - append correspondences with point idx

		matches.clear();
		cv::Mat old_valid_desc;
		cv::Mat cur_valid_desc;
		int cur_idx, old_idx = -1;
		for(const auto& old_kpt : old_kpts)
		{
			old_idx++;
			float old_kpt_x = old_kpts_bkpj.at(old_idx).x;
			float old_kpt_y = old_kpts_bkpj.at(old_idx).y;
			float old_kpt_z = old_kpts_bkpj.at(old_idx).z;
			if(old_kpt_x == 0 
			&& old_kpt_y == 0 
			&& old_kpt_z == 0) continue;

			old_valid_desc = cv::Mat();
			cur_valid_desc = cv::Mat();
			cur_valid_idx.clear();
			matches_lot.clear();
			cur_idx = -1;
			old_valid_desc.push_back(old_desc.row(old_idx));
			for(const auto& cur_kpt : cur_kpts)
			{
				cur_idx++;
				float cur_kpt_x = cur_kpts_bkpj.at(cur_idx).x;
				float cur_kpt_y = cur_kpts_bkpj.at(cur_idx).y;
				float cur_kpt_z = cur_kpts_bkpj.at(cur_idx).z;
				if(cur_kpt_x == 0 
				&& cur_kpt_y == 0 
				&& cur_kpt_z == 0) continue;

				float direction = -1*(cur_kpt_z-old_kpt_z);
				if(forward && direction < 0) continue;
				if(!forward && direction > 0) continue;
				float displacement = sqrt(pow(old_kpt_x-cur_kpt_x,2)+pow(old_kpt_y-cur_kpt_y,2)+pow(old_kpt_z-cur_kpt_z,2));
				if(displacement > max_valid_abs_displacement) continue;
				if(displacement < min_valid_abs_displacement) continue;
				float angle;
				if(forward) angle = atan2(old_kpt_x-cur_kpt_x, old_kpt_z-cur_kpt_z);
				if(!forward) angle = atan2(cur_kpt_x-old_kpt_x, cur_kpt_z-old_kpt_z);
				if(abs(angle) > DEG2RAD*max_steering_angle) continue;
				if(forward) angle = atan2(old_kpt_y-cur_kpt_y, old_kpt_z-cur_kpt_z);
				if(!forward) angle = atan2(cur_kpt_y-old_kpt_y, cur_kpt_z-old_kpt_z);
				if(abs(angle) > DEG2RAD*max_slope_angle) continue;
				cur_valid_desc.push_back(cur_desc.row(cur_idx));
				cur_valid_idx.push_back(cur_idx);
			}
			if(cur_valid_desc.rows < 1) continue;
			matches_lot = matchTwoImage(old_valid_desc, cur_valid_desc);

			for(const auto& match : matches_lot)
				matches.push_back(cv::DMatch(old_idx, cur_valid_idx.at(match.trainIdx), match.imgIdx, match.distance));
		}

		// Delete one to many matches
		matches = matchesRejectorOneToOne(matches);

		// std::cout<<"Matches size: "<<matches.size()<<std::endl;

		// util::tick_high_resolution(start_t, tick, elapsed_matching);

		/// PRE-PUBLISH
		std::vector<float> x_old, y_old, z_old;
		std::vector<float> x_cur, y_cur, z_cur;
		for (const auto& match : matches)
		{
			x_old.emplace_back(old_kpts_bkpj.at(match.queryIdx).x);
			y_old.emplace_back(old_kpts_bkpj.at(match.queryIdx).y);
			z_old.emplace_back(old_kpts_bkpj.at(match.queryIdx).z);
			x_cur.emplace_back(cur_kpts_bkpj.at(match.trainIdx).x);
			y_cur.emplace_back(cur_kpts_bkpj.at(match.trainIdx).y);
			z_cur.emplace_back(cur_kpts_bkpj.at(match.trainIdx).z);
		}

		/// PUBLISH
		/// - KeyPoint Matches
		terreslam::KeyPointMatchesPtr kpm_msg_ptr(new terreslam::KeyPointMatches);
		kpm_msg_ptr->header.frame_id = dd_keypoint_matches_frame_id;
		kpm_msg_ptr->header.stamp = info_msg->header.stamp;
		kpm_msg_ptr->x_cur = x_cur;
		kpm_msg_ptr->y_cur = y_cur;
		kpm_msg_ptr->z_cur = z_cur;
		kpm_msg_ptr->x_old = x_old;
		kpm_msg_ptr->y_old = y_old;
		kpm_msg_ptr->z_old = z_old;
		dd_keypoint_matches_pub_.publish(kpm_msg_ptr);

		//Update old data
		old_P_inv=cur_P_inv;
		old_kpts=cur_kpts;
		old_desc=cur_desc;

		entry_count++;

		// util::tick_high_resolution(start_t, tick, elapsed);
		// util::printElapsed(elapsed, "Callback dd_keypoint: ");
		// util::tick_high_resolution(start_t, tick, elapsed_publish);
		// util::printElapsed(elapsed_initialization, "Initialization: ");
		// util::printElapsed(elapsed_KP_detection, "KP_detection: ");
		// util::printElapsed(elapsed_KP_description, "KP_description: ");
		// util::printElapsed(elapsed_cam_model, "Cam_model: ");
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
		
		float max_possible_blob_vel_x = abs(blob_vel_x)+2*sqrt(blob_vel_x_var)+max_pitchaxis_acc*delta_time;
		float max_possible_blob_vel_z = abs(blob_vel_z)+2*sqrt(blob_vel_z_var)+max_rollaxis_acc*delta_time;
		float min_possible_blob_vel_x = abs(blob_vel_x)-2*sqrt(blob_vel_x_var)-max_pitchaxis_acc*delta_time;
		float min_possible_blob_vel_z = abs(blob_vel_z)-2*sqrt(blob_vel_z_var)-max_rollaxis_acc*delta_time;
		
		forward = blob_vel_z <= 0 ? true : false;
		max_possible_blob_heading_vel = sqrt(pow(max_possible_blob_vel_x,2)+pow(max_possible_blob_vel_z,2));
		min_possible_blob_heading_vel = sqrt(pow(min_possible_blob_vel_x,2)+pow(min_possible_blob_vel_z,2));
		if(forward) max_possible_blob_heading_vel *= -1;
		if(forward) min_possible_blob_heading_vel *= -1;

		// cout<<"UPDATE --> blob_vel_x: "<<blob_vel_x<<endl;
		// cout<<"UPDATE --> blob_vel_z: "<<blob_vel_z<<endl;
		// cout<<"UPDATE --> blob_vel_x_var: "<<blob_vel_x_var<<endl;
		// cout<<"UPDATE --> blob_vel_z_var: "<<blob_vel_z_var<<endl;
		// cout<<"UPDATE --> max_blob_vel_x: "<<max_pitchaxis_acc*delta_time<<endl;
		// cout<<"UPDATE --> max_blob_vel_z: "<<max_rollaxis_acc*delta_time<<endl;
		// cout<<"UPDATE --> max_possible_blob_vel_x: "<<max_possible_blob_vel_x<<endl;
		// cout<<"UPDATE --> max_possible_blob_vel_z: "<<max_possible_blob_vel_z<<endl;
		// cout<<"UPDATE --> min_possible_blob_vel_x: "<<min_possible_blob_vel_x<<endl;
		// cout<<"UPDATE --> min_possible_blob_vel_z: "<<min_possible_blob_vel_z<<endl;
		// cout<<"UPDATE --> Max velocity: "<<max_possible_blob_heading_vel<<endl;
		// cout<<"UPDATE --> Min velocity: "<<min_possible_blob_heading_vel<<endl;
		// cout<<endl;
	}

	void skipFrame(std::string msg)
	{
		std::cerr<<msg<<std::endl;
		entry_count++;
	}

private:
	/// Variables
	int queue_size_;

	/// Comms
	ros::Subscriber blob_odom_sub_;
	ros::Publisher dd_keypoint_matches_pub_;
	image_transport::SubscriberFilter rgb_sub_filter_;
	image_transport::SubscriberFilter depth_sub_filter_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_filter_;
	
	typedef message_filters::sync_policies::ExactTime
		<sensor_msgs::Image,
		sensor_msgs::Image,
		sensor_msgs::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;
	
	/// Chrono timmings
	// std::vector<double> elapsed;
	std::vector<double> elapsed_initialization;
	std::vector<double> elapsed_KP_detection;
	std::vector<double> elapsed_KP_description;
	std::vector<double> elapsed_cam_model;
	std::vector<double> elapsed_matching;
	std::vector<double> elapsed_publish;

	/// Blob motion filter
	bool forward = true;
	float max_valid_blob_velocity;
	float min_valid_blob_velocity;
	float max_valid_abs_displacement;
	float min_valid_abs_displacement;
	float max_possible_blob_heading_vel;
	float min_possible_blob_heading_vel;

	/// Timming
	float delta_time = -1;
	float old_time = -1;

	/// Kpts
	std::vector<cv::KeyPoint> old_kpts;
	std::vector<cv::KeyPoint> cur_kpts;
	cv::Mat old_desc;
	cv::Mat cur_desc;
	std::vector<cv::Point3f> old_kpts_bkpj;
	std::vector<cv::Point3f> cur_kpts_bkpj;
	std::vector<uint> cur_valid_idx;
	std::vector<cv::DMatch> matches;
	std::vector<cv::DMatch> matches_lot;
	Eigen::Matrix4d old_P_inv;
	Eigen::Matrix4d cur_P_inv;
	bool reset=false;

	
};

PLUGINLIB_EXPORT_CLASS(terreslam::DDKeypointNodelet, nodelet::Nodelet);

}
