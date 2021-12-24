/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-06-15 11:37:46
 */

#include "terreslam/nodelet.h"
#include "terreslam/utils/util_chrono.h"
#include "terreslam/utils/util_general.h"
#include "terreslam/registrations/dd_coarse_alignment.h"
#include "terreslam/registrations/ddd_coarse_alignment.h"

#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <deque>
#include <algorithm>

#include <visualization_msgs/Marker.h>

//msgs
#include <terreslam/BlobMatches.h>
#include <terreslam/BlobPoints.h>
#include <terreslam/KeyPointMatches.h>

namespace terreslam
{

class MetricAlignmentNodelet : public terreslam::Nodelet
{
public:
	MetricAlignmentNodelet() :
		queue_size_(10)
		{
			// std::cout << "Constructor metric_alignment_nodelet..." << std::endl;
		}

private:

	void onNodeletInit()
	{
		std::cout << "Initalize metric_alignment_nodelet..." << std::endl;
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		/// Subscribers
		blob_matches_sub_filter_.subscribe(nh, blob_matches_topic, 3);
		blob_points_sub_filter_.subscribe(nh, blob_points_topic, 3);
		dd_keypoint_matches_sub_filter_.subscribe(nh, dd_keypoint_matches_topic, 3);
		ddd_keypoint_matches_sub_filter_.subscribe(nh, ddd_keypoint_matches_topic, 3);
		old_cloud_filtered_low_sub_filter_.subscribe(nh, old_cloud_filtered_low_topic, 3);
		cloud_filtered_low_sub_filter_.subscribe(nh, cloud_filtered_low_topic, 3);
		cloud_sub_filter_.subscribe(nh, cloud_topic, 3);

		exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>
			(MyExactSyncPolicy(queue_size_), 
			blob_matches_sub_filter_, 
			blob_points_sub_filter_, 
			dd_keypoint_matches_sub_filter_, 
			ddd_keypoint_matches_sub_filter_,
			old_cloud_filtered_low_sub_filter_,
			cloud_filtered_low_sub_filter_,
			cloud_sub_filter_);
		exactSync_->registerCallback(boost::bind(&MetricAlignmentNodelet::callback, this, _1, _2,_3,_4,_5,_6,_7));

		/// Publishers
		cloud_keypoints_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cloud_keypoints_topic, 10);
		visualization_kps_pub_ = nh.advertise<visualization_msgs::Marker>(visualization_kps_topic, 3);
		visualization_trajectory_pub_ = nh.advertise<visualization_msgs::Marker>(visualization_trajectory_topic, 1);
		odom_pub_ = nh.advertise<nav_msgs::Odometry>(odom_topic, 1);
		blob_odom_pub_ = nh.advertise<nav_msgs::Odometry>(blob_odom_topic, 1);
	} 

	void callback(
		const terreslam::BlobMatches::ConstPtr& bm_msg_ptr,
		const terreslam::BlobPoints::ConstPtr& bp_msg_ptr,
		const terreslam::KeyPointMatches::ConstPtr& dd_kpm_msg_ptr,
		const terreslam::KeyPointMatches::ConstPtr& ddd_kpm_msg_ptr,
		const sensor_msgs::PointCloud2::ConstPtr& old_cloud_filtered_low_msg_ptr,
		const sensor_msgs::PointCloud2::ConstPtr& cloud_filtered_low_msg_ptr,
		const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr)
	{
		if(debug) std::cout << "Entry MA: " << entry_count << std::endl;
		///Start chrono ticking
		std::chrono::duration<double> tick;
		std::chrono::high_resolution_clock::time_point end_t, start_t;
		start_t = std::chrono::high_resolution_clock::now();
		end_t = std::chrono::high_resolution_clock::now();
		tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);

		size_t sm = bm_msg_ptr->x_old.size();
		assert(sm == bm_msg_ptr->stability.size() &&
					 sm == bm_msg_ptr->ppa.size() &&
					 sm == bm_msg_ptr->z_old.size() &&
					 sm == bm_msg_ptr->radius_old.size() &&
					 sm == bm_msg_ptr->height_old.size() &&
					 sm == bm_msg_ptr->x_cur.size() &&
					 sm == bm_msg_ptr->z_cur.size() &&
					 sm == bm_msg_ptr->radius_cur.size() &&
					 sm == bm_msg_ptr->height_cur.size());
		std::vector<uint8_t> stability(sm);
		std::vector<uint32_t> ppa(sm);
		std::vector<float> radius_old(sm), height_old(sm);
		std::vector<float> radius_cur(sm), height_cur(sm);
		std::vector<cv::Point2f> bm_old(sm), bm_cur(sm);
		delta_time = bm_msg_ptr->delta_time;
		stability = bm_msg_ptr->stability;
		ppa = bm_msg_ptr->ppa;
		radius_old = bm_msg_ptr->radius_old;
		height_old = bm_msg_ptr->height_old;
		radius_cur = bm_msg_ptr->radius_cur;
		height_cur = bm_msg_ptr->height_cur;
		for(size_t i=0; i<sm; ++i)
		{
			bm_old.at(i) = cv::Point2f(bm_msg_ptr->x_old.at(i), bm_msg_ptr->z_old.at(i));
			bm_cur.at(i) = cv::Point2f(bm_msg_ptr->x_cur.at(i), bm_msg_ptr->z_cur.at(i));
		}

		size_t sp_old = bp_msg_ptr->s_old.size();
		size_t sp_cur = bp_msg_ptr->s_cur.size();
		assert(sp_old == bp_msg_ptr->x_old.size() &&
					 sp_old == bp_msg_ptr->y_old.size() &&
					 sp_old == bp_msg_ptr->z_old.size());
		assert(sp_cur == bp_msg_ptr->x_cur.size() &&
					 sp_cur == bp_msg_ptr->y_cur.size() &&
					 sp_cur == bp_msg_ptr->z_cur.size());
		pcl::PointXYZ point_pcl;
		std::vector<pcl::PointCloud<pcl::PointXYZ>> old_blobs_pts;
		pcl::PointCloud<pcl::PointXYZ>::Ptr old_blob_pts_ptr (new pcl::PointCloud<pcl::PointXYZ>);
		unsigned int blob_seg = bp_msg_ptr->s_old.at(0);
		for(size_t i=0; i<sp_old; ++i)
		{
			if(bp_msg_ptr->s_old.at(i) != blob_seg)
			{
				blob_seg = bp_msg_ptr->s_old.at(i);
				old_blobs_pts.push_back(*old_blob_pts_ptr);
				old_blob_pts_ptr->clear();
			}
			point_pcl.x = bp_msg_ptr->x_old.at(i);
			point_pcl.y = bp_msg_ptr->y_old.at(i);
			point_pcl.z = bp_msg_ptr->z_old.at(i);
			old_blob_pts_ptr->points.push_back(point_pcl);
		}

		size_t dd_sm  = dd_kpm_msg_ptr->x_old.size();
		assert(dd_sm == dd_kpm_msg_ptr->y_old.size() &&
					 dd_sm == dd_kpm_msg_ptr->z_old.size() &&
					 dd_sm == dd_kpm_msg_ptr->x_cur.size() &&
					 dd_sm == dd_kpm_msg_ptr->y_cur.size() &&
					 dd_sm == dd_kpm_msg_ptr->z_cur.size());
		std::vector<cv::Point3f> dd_kpm_old(dd_sm), dd_kpm_cur(dd_sm);
		for(size_t i=0; i<dd_sm; ++i)
		{
			dd_kpm_old.at(i) = cv::Point3f(dd_kpm_msg_ptr->x_old.at(i), dd_kpm_msg_ptr->y_old.at(i), dd_kpm_msg_ptr->z_old.at(i));
			dd_kpm_cur.at(i) = cv::Point3f(dd_kpm_msg_ptr->x_cur.at(i), dd_kpm_msg_ptr->y_cur.at(i), dd_kpm_msg_ptr->z_cur.at(i));
		}
		// cout<<"Size dd_sm: "<<dd_sm<<endl;

		size_t ddd_sm = ddd_kpm_msg_ptr->x_old.size();
		assert(ddd_sm == ddd_kpm_msg_ptr->y_old.size() &&
					 ddd_sm == ddd_kpm_msg_ptr->z_old.size() &&
					 ddd_sm == ddd_kpm_msg_ptr->x_cur.size() &&
					 ddd_sm == ddd_kpm_msg_ptr->y_cur.size() &&
					 ddd_sm == ddd_kpm_msg_ptr->z_cur.size());
		std::vector<cv::Point3f> ddd_kpm_old(ddd_sm), ddd_kpm_cur(ddd_sm);
		for(size_t i=0; i<ddd_sm; ++i)
		{
			ddd_kpm_old.at(i) = cv::Point3f(ddd_kpm_msg_ptr->x_old.at(i), ddd_kpm_msg_ptr->y_old.at(i), ddd_kpm_msg_ptr->z_old.at(i));
			ddd_kpm_cur.at(i) = cv::Point3f(ddd_kpm_msg_ptr->x_cur.at(i), ddd_kpm_msg_ptr->y_cur.at(i), ddd_kpm_msg_ptr->z_cur.at(i));
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr old_filtered_low_pts (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*old_cloud_filtered_low_msg_ptr, *old_filtered_low_pts);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cur_filtered_low_pts (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*cloud_filtered_low_msg_ptr, *cur_filtered_low_pts);
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cur_pts (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*cloud_msg_ptr, *cur_pts);

		ros::Time cur_stamp = bm_msg_ptr->header.stamp;
		assert(cur_stamp == dd_kpm_msg_ptr->header.stamp &&
					 cur_stamp == ddd_kpm_msg_ptr->header.stamp);

		// util::tick_high_resolution(start_t, tick, elapsed_load_msg);

		///METRIC ALIGNMENT
		cv::Mat RTr = RTr_identity.clone();
		
		/// - MA Coarse Blobs
		if(MA_Blobs)
		{
		float best_param[3] = {0.0f};
		bool inliers[sm] = {true};
		fit3DofRANSAC(bm_old, bm_cur, best_param, inliers, cv::Point2f(0,0), 0.1, 2*sm, MA_debug_Blobs_coarse);

		/// Transformation
		float theta = best_param[0];
		float tx = best_param[1];
		float tz = best_param[2];
		RTr = cv::Mat(cv::Matx44f(cos(theta), 0, -sin(theta), tx, 
															0         , 1, 0 			    , 0 , 
															sin(theta), 0, cos(theta) , tz,
															0         , 0, 0          , 1));
		
		}

		// util::tick_high_resolution(start_t, tick, elapsed_blob_coarse);
		
		
		/// - MA Coarse Keypoints
		if(cv::countNonZero(RTr != RTr_identity)) trajectory.color.b = 0.0;
		else{ //Compute KPs MA
		if(MA_joint_KPs && MA_DDKPs && MA_DDDKPs) //Joint KPs
		{
			cv::Mat RTr_KPs = RTr_identity.clone();

			ddd_kpm_old.insert(
				ddd_kpm_old.end(), 
				std::make_move_iterator(dd_kpm_old.begin()),
				std::make_move_iterator(dd_kpm_old.end()));

			ddd_kpm_cur.insert(
				ddd_kpm_cur.end(), 
				std::make_move_iterator(dd_kpm_cur.begin()),
				std::make_move_iterator(dd_kpm_cur.end()));

			
			size_t joint_sm = dd_sm+ddd_sm;

			// float best_param[6] = {0.0f};
			// bool inliers[joint_sm] = {true};
			// fit6DofRANSAC(ddd_kpm_old, ddd_kpm_cur, best_param, RTr_KPs, inliers, cv::Point3f(0,0,0), 0.1, joint_sm, MA_debug_KPs);

			float best_param[3] = {0.0f};
			bool inliers[joint_sm] = {true};
			fit3DofRANSAC(ddd_kpm_old, ddd_kpm_cur, best_param, inliers, cv::Point2f(0,0), 0.1, joint_sm, MA_debug_KPs);
			float theta = best_param[0];
			float tx = best_param[1];
			float tz = best_param[2];
			RTr_KPs = cv::Mat(cv::Matx44f(cos(theta), 0, -sin(theta), tx, 
																		0         , 1, 0 			    , 0 , 
																		sin(theta), 0, cos(theta) , tz,
																		0         , 0, 0          , 1));

			RTr = RTr_KPs.clone();
		}
		else //separated KPs
		{
			if(MA_DDKPs)
			{
				cv::Mat RTr_2DKPs = RTr_identity.clone();

				// float best_param_2D[6] = {0.0f};
				// bool inliers_2D[dd_sm] = {true};
				// fit6DofRANSAC(dd_kpm_old, dd_kpm_cur, best_param_2D, RTr_2DKPs, inliers_2D, cv::Point3f(0,0,0), 0.1, dd_sm, MA_debug_KPs);

				float best_param_2D[3] = {0.0f};
				bool inliers_2D[dd_sm] = {true};
				fit3DofRANSAC(dd_kpm_old, dd_kpm_cur, best_param_2D, inliers_2D, cv::Point2f(0,0), 0.1, dd_sm, MA_debug_KPs);
				float theta = best_param_2D[0];
				float tx = best_param_2D[1];
				float tz = best_param_2D[2];
				RTr_2DKPs = cv::Mat(cv::Matx44f(cos(theta), 0, -sin(theta), tx, 
																				0         , 1, 0 			    , 0 , 
																				sin(theta), 0, cos(theta) , tz,
																				0         , 0, 0          , 1));

				RTr = RTr_2DKPs.clone();
			}

			if(MA_DDDKPs)
			{
				cv::Mat RTr_3DKPs = RTr_identity.clone();

				if(MA_DDKPs) cv::transform(ddd_kpm_old, ddd_kpm_old, RTr(cv::Rect( 0, 0, 4, 3 )));

				// float best_param_3D[6] = {0.0f};
				// bool inliers_3D[ddd_sm] = {true};
				// fit6DofRANSAC(ddd_kpm_old, ddd_kpm_cur, best_param_3D, RTr_3DKPs, inliers_3D, cv::Point3f(0,0,0), 0.1, ddd_sm, MA_debug_KPs);

				float best_param_3D[3] = {0.0f};
				bool inliers_3D[ddd_sm] = {true};
				fit3DofRANSAC(ddd_kpm_old, ddd_kpm_cur, best_param_3D, inliers_3D, cv::Point2f(0,0), 0.1, ddd_sm, MA_debug_KPs);
				float theta = best_param_3D[0];
				float tx = best_param_3D[1];
				float tz = best_param_3D[2];
				RTr_3DKPs = cv::Mat(cv::Matx44f(cos(theta), 0, -sin(theta), tx, 
																				0         , 1, 0 			    , 0 , 
																				sin(theta), 0, cos(theta) , tz,
																				0         , 0, 0          , 1));

				if(MA_DDKPs) RTr = RTr * RTr_3DKPs;
				else RTr = RTr_3DKPs.clone();
			}
		}
		trajectory.color.b = 1.0;
		}
		// util::tick_high_resolution(start_t, tick, elapsed_kps);

		
		/// - MA Coarse Regularization
		if(MA_regularisation){
		blob_d_cur = cv::Point2f(RTr.at<float>(0,3), RTr.at<float>(2,3)); // x & z components
		blob_v_cur = blob_d_cur / delta_time;
		
		if(entry_count < 3)
		{
			if(abs(blob_v_cur.x) > max_pitchaxis_vel 
			|| abs(blob_v_cur.y) > max_rollaxis_vel)
			{
				RTr = RTr_blob_old.clone(); //Repeat transform
				trajectory.color.g = 1.0; //yellow (repeat)
			}
			else //Valid transform
			{
				RTr_blob_old = RTr.clone();
				blob_vels_x.push_back(blob_v_cur.x);
				blob_vels_z.push_back(blob_v_cur.y);
				blob_vels2_x.push_back(pow(blob_v_cur.x,2));
				blob_vels2_z.push_back(pow(blob_v_cur.y,2));
				trajectory.color.g = 0.0; //red (default)
			}
		}
		else
		{
			cv::Point2f blob_v_old = cv::Point2f(util::calculateMedian(blob_v_old_x), util::calculateMedian(blob_v_old_z));
			blob_a_cur = (blob_v_cur - blob_v_old) / delta_time;
			// cout<<"deltaTime: "<<delta_time<<endl;
			// cout<<"Dis Z: "<<blob_d_cur.y<<endl;
			// cout<<"Vel Z: "<<blob_v_cur.y<<endl;
			// cout<<"Vel Old Z: "<<blob_v_old.y<<endl;
			// cout<<"Acc Z: "<<blob_a_cur.y<<endl;
			// cout<<"Dis X: "<<blob_d_cur.x<<endl;
			// cout<<"Vel X: "<<blob_v_cur.x<<endl;
			// cout<<"Vel Old X: "<<blob_v_old.x<<endl;
			// cout<<"Acc X: "<<blob_a_cur.x<<endl;
			// cout<<endl;
		 
			if(abs(blob_a_cur.x) > max_pitchaxis_acc 
			|| abs(blob_a_cur.y) > max_rollaxis_acc)
			{
				RTr = RTr_blob_old.clone(); //Repeat transform
				trajectory.color.g = 1.0; //yellow (repeat)
			}
			else //Valid transform
			{
				RTr_blob_old = RTr.clone();
				blob_vels_x.push_back(blob_v_cur.x);
				blob_vels_z.push_back(blob_v_cur.y);
				blob_vels2_x.push_back(pow(blob_v_cur.x,2));
				blob_vels2_z.push_back(pow(blob_v_cur.y,2));
				trajectory.color.g = 0.0; //red (default)
			}
		}

		if(blob_vels_x.size() > MA_max_blob_heading_vels) 
		{
			blob_vels_x.pop_front();
			blob_vels_z.pop_front();
			blob_vels2_x.pop_front();
			blob_vels2_z.pop_front();

			float vx_avg = accumulate(blob_vels_x.begin(), blob_vels_x.end(), 0.0f) / blob_vels_x.size();
			float vz_avg = accumulate(blob_vels_z.begin(), blob_vels_z.end(), 0.0f) / blob_vels_z.size();
			float vx2_avg = accumulate(blob_vels2_x.begin(), blob_vels2_x.end(), 0.0f) / blob_vels2_x.size();
			float vz2_avg = accumulate(blob_vels2_z.begin(), blob_vels2_z.end(), 0.0f) / blob_vels2_z.size();

			nav_msgs::Odometry odom;
			odom.header.stamp = cur_stamp;
			odom.header.frame_id = blob_odom_frame_id;
			odom.child_frame_id = sub_cam_frame_id;
			odom.twist.twist.linear.x = vx_avg;
			odom.twist.twist.linear.z = vz_avg;
			odom.twist.covariance.at(0) = vx2_avg - pow(vx_avg,2);
			odom.twist.covariance.at(14) = vz2_avg - pow(vz_avg,2);
			blob_odom_pub_.publish(odom);
		}

		if(blob_v_old_x.size() > MA_max_blob_heading_vels) 
		{
			blob_v_old_x.pop_front();
			blob_v_old_z.pop_front();
		}

		blob_v_old_x.push_back(blob_v_cur.x);
		blob_v_old_z.push_back(blob_v_cur.y);
		}
		else trajectory.color.g = 0.0; //default

		// util::tick_high_resolution(start_t, tick, elapsed_regularization);

		/// - MA Plannar Fine
		// Eigen::Matrix4f ei_RTr;
		// cv::cv2eigen(RTr, ei_RTr);
		// pcl::transformPointCloud(*old_filtered_low_pts, *old_blob_pts_ptr, ei_RTr);
		// pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
		// gicp.setInputSource(old_blob_pts_ptr);
		// gicp.setInputTarget(cur_filtered_low_pts);
		// gicp.setMaxCorrespondenceDistance(0.2);
		// gicp.setMaximumIterations(50);
		// gicp.align(*old_blob_pts_ptr);
		
		// Eigen::Matrix4f gicp_trans = gicp.getFinalTransformation();
		// cv::Mat RTr_plannar_fine;
		// cv::eigen2cv(gicp_trans, RTr_plannar_fine);
		// RTr = RTr * RTr_plannar_fine;

		// util::tick_high_resolution(start_t, tick, elapsed_plannar_fine);

		/// - MA Blob Fine separated
		// if(old_blobs_pts.size() >= 3){
		// // pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); //Suppress PCL_ERRORs
		// std::vector<cv::Mat> RTr_blob_fines;
		// cv::Mat pts_trans(4,old_blobs_pts.size(), CV_32FC1);
		// Eigen::Matrix4f ei_RTr;
		// cv::cv2eigen(RTr, ei_RTr);
		// for(size_t x=0; x<old_blobs_pts.size(); ++x)
		// {
		// 	pcl::transformPointCloud(old_blobs_pts.at(x), *old_blob_pts_ptr, ei_RTr);
		// 	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		// 	icp.setInputSource(old_blob_pts_ptr);
		// 	icp.setInputTarget(cur_pts);
		// 	icp.setMaxCorrespondenceDistance(0.05);
		// 	icp.setMaximumIterations(50);
		// 	icp.setTransformationEpsilon(1e-8);
		// 	icp.setEuclideanFitnessEpsilon(1);
		// 	icp.align(*old_blob_pts_ptr);

		// 	Eigen::Matrix4f icp_trans = icp.getFinalTransformation();
		// 	cv::Mat RTr_blob_fine;
		// 	cv::eigen2cv(icp_trans, RTr_blob_fine);
		// 	RTr_blob_fines.push_back(RTr_blob_fine);

		// 	pts_trans(cv::Rect( x, 0, 1, 3 )) = RTr_blob_fine(cv::Rect( 0, 0, 4, 3 )) * cv::Mat(cv::Matx41f(0, 0, 0, 1));
			
		// }
		// cv::Mat col_avg, row_sum;
		// reduce(pts_trans,col_avg, 1, cv::REDUCE_AVG);
		// cv::Mat RTr_subtract_avg = RTr_identity.clone();
		// RTr_subtract_avg(cv::Rect( 3, 0, 1, 3 )) = col_avg*-1;
		// cv::Mat pts_trans_dev = RTr_subtract_avg(cv::Rect( 0, 0, 4, 3 ))*pts_trans;
		// reduce(cv::abs(pts_trans_dev),row_sum, 0, cv::REDUCE_SUM);
		// cv::Point min_loc;
		// cv::minMaxLoc(row_sum, NULL, NULL, &min_loc, NULL);
		// RTr = RTr * RTr_blob_fines.at(min_loc.x);
		
		// // util::tick_high_resolution(start_t, tick, elapsed_blob_fine_separate);
		// }

		/// - MA Blob Fine joint
		// if(old_blobs_pts.size() >= 3){
		// // pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); //Suppress PCL_ERRORs
		// std::vector<cv::Mat> RTr_blob_fines;
		// cv::Mat pts_trans(4,old_blobs_pts.size(), CV_32FC1);
		// pcl::PointCloud<pcl::PointXYZ> joint_old_blobs_pts;
		// Eigen::Matrix4f ei_RTr;
		// cv::cv2eigen(RTr, ei_RTr);
		// // joint_old_blobs_pts += (*old_filtered_low_pts);
		// for(size_t x=0; x<old_blobs_pts.size(); ++x)
		// {
		// 	joint_old_blobs_pts += old_blobs_pts.at(x);
		// }
		// pcl::transformPointCloud(joint_old_blobs_pts, *old_blob_pts_ptr, ei_RTr);
		// pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		// icp.setInputSource(old_blob_pts_ptr);
		// icp.setInputTarget(cur_pts);
		// icp.setMaxCorrespondenceDistance(0.05);
		// icp.setMaximumIterations(50);
		// icp.setTransformationEpsilon(1e-8);
		// icp.setEuclideanFitnessEpsilon(1);
		// icp.align(*old_blob_pts_ptr);

		// Eigen::Matrix4f icp_trans = icp.getFinalTransformation();
		// cv::Mat RTr_blob_fine;
		// cv::eigen2cv(icp_trans, RTr_blob_fine);
		// RTr = RTr * RTr_blob_fine;
		
		// // util::tick_high_resolution(start_t, tick, elapsed_blob_fine_joint);
		// }

		/// - MA Blob Fine joint submap
		if(old_blobs_pts.size() >= 3){
		// pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); //Suppress PCL_ERRORs
		std::vector<cv::Mat> RTr_blob_fines;
		cv::Mat pts_trans(4,old_blobs_pts.size(), CV_32FC1);
		pcl::PointCloud<pcl::PointXYZ> joint_old_blobs_pts;
		Eigen::Matrix4f ei_RTr;
		cv::cv2eigen(RTr, ei_RTr);
		// joint_old_blobs_pts += (*old_filtered_low_pts);
		for(size_t x=0; x<old_blobs_pts.size(); ++x)
		{
			joint_old_blobs_pts += old_blobs_pts.at(x);
		}
		pcl::transformPointCloud(joint_old_blobs_pts, *old_blob_pts_ptr, ei_RTr);
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputSource(old_blob_pts_ptr);
		icp.setInputTarget(cur_pts);
		icp.setMaxCorrespondenceDistance(0.05);
		icp.setMaximumIterations(50);
		icp.setTransformationEpsilon(1e-8);
		icp.setEuclideanFitnessEpsilon(1);
		icp.align(*old_blob_pts_ptr);

		Eigen::Matrix4f icp_trans = icp.getFinalTransformation();
		cv::Mat RTr_blob_fine;
		cv::eigen2cv(icp_trans, RTr_blob_fine);
		RTr = RTr * RTr_blob_fine;
		
		// util::tick_high_resolution(start_t, tick, elapsed_blob_fine_joint);
		}


		/// PRE-PUBLISH
		/// - Transformations
		trajectory.header.frame_id = visualization_trajectory_frame_id;
		trajectory.header.stamp = cur_stamp;
		trajectory.action = visualization_msgs::Marker::ADD;
		trajectory.id = entry_count;
		trajectory.type = visualization_msgs::Marker::LINE_LIST;
		trajectory.pose.orientation.w = 1.0;
		trajectory.scale.x = 0.05; //width
		trajectory.color.r = 1.0;
		trajectory.color.a = 1.0;
		trajectory.points.clear();
		geometry_msgs::Point p;
		p.x = RTr_acum.at<float>(0,3);
		p.y = RTr_acum.at<float>(1,3);
		p.z = RTr_acum.at<float>(2,3);
		trajectory.points.push_back(p);

		cv::Mat R_inv = RTr(cv::Rect( 0, 0, 3, 3 )).t();
		cv::Mat RTr_inv = cv::Mat(cv::Matx44f(R_inv.at<float>(0,0), R_inv.at<float>(0,1), R_inv.at<float>(0,2), -RTr.at<float>(0,3), 
																					R_inv.at<float>(1,0), R_inv.at<float>(1,1), R_inv.at<float>(1,2), -RTr.at<float>(1,3), 
																					R_inv.at<float>(2,0), R_inv.at<float>(2,1), R_inv.at<float>(2,2), -RTr.at<float>(2,3),
																					0										, 0										, 0										,	1									));
		 
		RTr_acum = RTr_acum * RTr_inv;

		// cout<<"Tx: "<<RTr.at<float>(0,3)<<endl;
		// cout<<"Ty: "<<RTr.at<float>(1,3)<<endl;
		// cout<<"Tz: "<<RTr.at<float>(2,3)<<endl;
		// cout<<"INVERSE"<<endl;
		// cout<<"Tx: "<<RTr_inv.at<float>(0,3)<<endl;
		// cout<<"Ty: "<<RTr_inv.at<float>(1,3)<<endl;
		// cout<<"Tz: "<<RTr_inv.at<float>(2,3)<<endl;
		// cout<<"ACCUMULATE"<<endl;
		// cout<<"Tx: "<<RTr_acum.at<float>(0,3)<<endl;
		// cout<<"Ty: "<<RTr_acum.at<float>(1,3)<<endl;
		// cout<<"Tz: "<<RTr_acum.at<float>(2,3)<<endl<<endl;

		tf2::Matrix3x3 R_acum_tf2_m(RTr_acum.at<float>(0,0), RTr_acum.at<float>(0,1), RTr_acum.at<float>(0,2),
															 RTr_acum.at<float>(1,0), RTr_acum.at<float>(1,1), RTr_acum.at<float>(1,2),
															 RTr_acum.at<float>(2,0), RTr_acum.at<float>(2,1), RTr_acum.at<float>(2,2)); 
		tf2::Quaternion R_acum_tf2_q;
		R_acum_tf2_m.getRotation(R_acum_tf2_q);

		p.x = RTr_acum.at<float>(0,3);
		p.y = RTr_acum.at<float>(1,3);
		p.z = RTr_acum.at<float>(2,3);
		trajectory.points.push_back(p);
		
		/// - Keypoints & Matching lines colorized
		pcl::PointCloud<pcl::PointXYZRGBA> kpm_cloud;
		kpm_cloud.points.resize(dd_kpm_cur.size()+dd_kpm_old.size()+ddd_kpm_cur.size()+ddd_kpm_old.size());
		
		visualization_msgs::Marker rm_line_matches, dd_line_matches, ddd_line_matches;
		rm_line_matches.header.frame_id = visualization_kps_frame_id;
		rm_line_matches.header.stamp = cur_stamp;
		rm_line_matches.action = visualization_msgs::Marker::DELETEALL;
		dd_line_matches = ddd_line_matches = rm_line_matches;
		dd_line_matches.ns = "dd_line_matches";
		ddd_line_matches.ns = "ddd_line_matches";
		dd_line_matches.id = ddd_line_matches.id = 0;
		dd_line_matches.type = ddd_line_matches.type = visualization_msgs::Marker::LINE_LIST;
		dd_line_matches.action = ddd_line_matches.action = visualization_msgs::Marker::ADD;
		dd_line_matches.pose.orientation.w = ddd_line_matches.pose.orientation.w = 1.0;
		dd_line_matches.scale.x = ddd_line_matches.scale.x = 0.02; //width
		uint8_t a=255,r,g,b;
		uint32_t rgba_cur, rgba_old;

		dd_line_matches.color.r = 0.0;
		dd_line_matches.color.g = 1.0;
		dd_line_matches.color.b = 0.0;
		dd_line_matches.color.a = 1.0;
		r = 0; g = 255; b = 255; rgba_cur = (a << 24 | r << 16 | g << 8 | b);
		r = 0; g = 255; b = 0; rgba_old = (a << 24 | r << 16 | g << 8 | b);
		for (size_t i=0; i<dd_sm; ++i) {
			kpm_cloud.points[i*2].x = dd_kpm_cur[i].x;
			kpm_cloud.points[i*2].y = dd_kpm_cur[i].y;
			kpm_cloud.points[i*2].z = dd_kpm_cur[i].z;
			kpm_cloud.points[i*2].rgba = rgba_cur;
			kpm_cloud.points[i*2+1].x = dd_kpm_old[i].x;
			kpm_cloud.points[i*2+1].y = dd_kpm_old[i].y;
			kpm_cloud.points[i*2+1].z = dd_kpm_old[i].z;
			kpm_cloud.points[i*2+1].rgba = rgba_old;
			p.x = dd_kpm_cur[i].x;
			p.y = dd_kpm_cur[i].y;
			p.z = dd_kpm_cur[i].z;
			dd_line_matches.points.push_back(p);
			p.x = dd_kpm_old[i].x;
			p.y = dd_kpm_old[i].y;
			p.z = dd_kpm_old[i].z;
			dd_line_matches.points.push_back(p);
		}

		ddd_line_matches.color.r = 1.0;
		ddd_line_matches.color.g = 0.0;
		ddd_line_matches.color.b = 0.0;
		ddd_line_matches.color.a = 1.0;
		r = 255; g = 255; b = 0; rgba_cur = (a << 24 | r << 16 | g << 8 | b);
		r = 255; g = 0; b = 0; rgba_old = (a << 24 | r << 16 | g << 8 | b);
		for (size_t i=0; i<ddd_sm; ++i) {
			kpm_cloud.points[i*2+dd_sm*2].x = ddd_kpm_cur[i].x;
			kpm_cloud.points[i*2+dd_sm*2].y = ddd_kpm_cur[i].y;
			kpm_cloud.points[i*2+dd_sm*2].z = ddd_kpm_cur[i].z;
			kpm_cloud.points[i*2+dd_sm*2].rgba = rgba_cur;
			kpm_cloud.points[i*2+1+dd_sm*2].x = ddd_kpm_old[i].x;
			kpm_cloud.points[i*2+1+dd_sm*2].y = ddd_kpm_old[i].y;
			kpm_cloud.points[i*2+1+dd_sm*2].z = ddd_kpm_old[i].z;
			kpm_cloud.points[i*2+1+dd_sm*2].rgba = rgba_old;
			p.x = ddd_kpm_cur[i].x;
			p.y = ddd_kpm_cur[i].y;
			p.z = ddd_kpm_cur[i].z;
			ddd_line_matches.points.push_back(p);
			p.x = ddd_kpm_old[i].x;
			p.y = ddd_kpm_old[i].y;
			p.z = ddd_kpm_old[i].z;
			ddd_line_matches.points.push_back(p);
		}

		/// PUBLISH
		/// - Keypoint matches
		visualization_kps_pub_.publish(rm_line_matches); //Delete ALL previous lines
		sensor_msgs::PointCloud2 msg_pcd;
		pcl::toROSMsg(kpm_cloud, msg_pcd);
		msg_pcd.header.frame_id = cloud_keypoints_frame_id;
		msg_pcd.header.stamp = cur_stamp;
		cloud_keypoints_pub_.publish(msg_pcd);
		visualization_kps_pub_.publish(dd_line_matches);
		visualization_kps_pub_.publish(ddd_line_matches);

		/// - Trasnforms
		static tf2_ros::TransformBroadcaster odom_broadcaster;
		geometry_msgs::TransformStamped odom_trans;
		geometry_msgs::Quaternion R_acum_msg_q = tf2::toMsg(R_acum_tf2_q);
		odom_trans.header.stamp = cur_stamp;
		odom_trans.header.frame_id = odom_frame_id;
		odom_trans.child_frame_id = sub_cam_frame_id;
		odom_trans.transform.translation.x = RTr_acum.at<float>(0,3);
		odom_trans.transform.translation.y = RTr_acum.at<float>(1,3);
		odom_trans.transform.translation.z = RTr_acum.at<float>(2,3);
		odom_trans.transform.rotation = R_acum_msg_q;
		odom_broadcaster.sendTransform(odom_trans);
		visualization_trajectory_pub_.publish(trajectory);
		
		/// - Odometry
		nav_msgs::Odometry odom;
		odom.header.stamp = cur_stamp;
		odom.header.frame_id = odom_frame_id;
		odom.child_frame_id = sub_cam_frame_id;
		odom.pose.pose.position.x = RTr_acum.at<float>(0,3);
		odom.pose.pose.position.y = RTr_acum.at<float>(1,3);
		odom.pose.pose.position.z = RTr_acum.at<float>(2,3);
		odom.pose.pose.orientation = R_acum_msg_q;
		odom_pub_.publish(odom);

		entry_count++;


		// util::tick_high_resolution(start_t, tick, elapsed);
		// util::printElapsed(elapsed, "Callback blob detector: ");
		// util::printElapsed(elapsed_load_msg, "Load msg: ");
		// util::printElapsed(elapsed_blob_coarse, "MA Blob Coarse: ");
		// util::printElapsed(elapsed_kps, "MA Keypoints: ");
		// util::printElapsed(elapsed_regularization, "Regularization: ");
		// util::printElapsed(elapsed_plannar_fine, "MA Plannar fine: ");
		// util::printElapsed(elapsed_blob_fine_separate, "MA Blob fine separate: ");
		// util::printElapsed(elapsed_blob_fine_joint, "MA Blob fine joint: ");

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
	ros::Publisher visualization_kps_pub_;
	ros::Publisher visualization_trajectory_pub_;
	ros::Publisher odom_pub_;
	ros::Publisher blob_odom_pub_;
	message_filters::Subscriber<terreslam::BlobMatches> blob_matches_sub_filter_;
	message_filters::Subscriber<terreslam::BlobPoints> blob_points_sub_filter_;
	message_filters::Subscriber<terreslam::KeyPointMatches> dd_keypoint_matches_sub_filter_;
	message_filters::Subscriber<terreslam::KeyPointMatches> ddd_keypoint_matches_sub_filter_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> old_cloud_filtered_low_sub_filter_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_filtered_low_sub_filter_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_filter_;
	
	typedef message_filters::sync_policies::ExactTime
		<terreslam::BlobMatches,
		terreslam::BlobPoints,
		terreslam::KeyPointMatches,
		terreslam::KeyPointMatches,
		sensor_msgs::PointCloud2,
		sensor_msgs::PointCloud2,
		sensor_msgs::PointCloud2> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy>* exactSync_;

	/// Chrono timmings
	std::vector<double> elapsed;
	std::vector<double> elapsed_load_msg;
	std::vector<double> elapsed_blob_coarse;
	std::vector<double> elapsed_kps;
	std::vector<double> elapsed_regularization;
	std::vector<double> elapsed_plannar_fine;
	std::vector<double> elapsed_blob_fine_joint;
	std::vector<double> elapsed_blob_fine_separate;

	/// MA
	cv::Mat RTr_identity = (cv::Mat_<float>(4, 4)<<1, 0, 0, 0, 
																								 0, 1, 0, 0, 
																								 0, 0, 1, 0,
																								 0, 0, 0, 1);
	cv::Mat RTr_acum = RTr_identity.clone();

	///MA Blob Regulatization
	visualization_msgs::Marker trajectory;
	double delta_time;
	cv::Point2f blob_d_cur;
	cv::Point2f blob_v_cur;
	cv::Point2f blob_a_cur;
	std::deque<float> blob_v_old_z;
	std::deque<float> blob_v_old_x;
	cv::Mat RTr_blob_old = RTr_identity.clone();
	std::deque<float> blob_vels_x;
	std::deque<float> blob_vels_z;
	std::deque<float> blob_vels2_x;
	std::deque<float> blob_vels2_z;

};

PLUGINLIB_EXPORT_CLASS(terreslam::MetricAlignmentNodelet, nodelet::Nodelet);

}