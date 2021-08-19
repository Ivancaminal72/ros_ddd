/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-08-16 16:38:16
 */

#include "terreslam/frontend.h"
#include "terreslam/camera_model.h"
#include "terreslam/features/keypoint_detector.h"
#include "terreslam/processings/keypoint_processor.h"
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

//msgs
#include <terreslam/KeyPointMatches.h>

namespace terreslam
{

class DDKeypointFrontend : public terreslam::Frontend
{
public:
	DDKeypointFrontend() : 
	Frontend(),
		queue_size_(10)
		{
			// std::cout << "Constructor dd_keypoint_frontend..." << std::endl;
		}

private:

	virtual void onFrontendInit()
	{
		std::cout << "Initalize dd_keypoint_frontend..." << std::endl;
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();
		
		std::string subscribedTopicsMsg;

		ros::NodeHandle rgb_nh(nh, "rgb_img");
		ros::NodeHandle depth_nh(nh, "depth_img");
		ros::NodeHandle rgb_pnh(pnh, "rgb_img");
		ros::NodeHandle depth_pnh(pnh, "depth_img");
		image_transport::ImageTransport rgb_it(rgb_nh);
		image_transport::ImageTransport depth_it(depth_nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
		image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

		/// Subscribers
		rgb_sub_filter_.subscribe(rgb_it, sub_cam_frame_id, 1, hintsRgb);
		depth_sub_filter_.subscribe(depth_it, sub_cam_depth_frame_id, 1, hintsDepth);
		info_sub_filter_.subscribe(rgb_nh, sub_cam_info_frame_id, 1);

		exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>
			(MyExactSyncPolicy(queue_size_), rgb_sub_filter_, depth_sub_filter_, info_sub_filter_);
		exactSync_->registerCallback(boost::bind(&DDKeypointFrontend::callback, this, _1, _2, _3));

		// Publishers
		keypoint_matches_pub_ = nh.advertise<terreslam::KeyPointMatches>(keypoint_matches_frame_id, 1);
	} 

	void callback(
		const sensor_msgs::Image::ConstPtr& rgb_msg,
		const sensor_msgs::Image::ConstPtr& depth_msg,
		const sensor_msgs::CameraInfo::ConstPtr& info_msg)
	{
		std::cout << "Entry DD KP: " << entry_count << std::endl;
		///Start chrono ticking
		// std::chrono::duration<double> tick;
		// std::chrono::high_resolution_clock::time_point end_t, start_t;
		// start_t = std::chrono::high_resolution_clock::now();
		// end_t = std::chrono::high_resolution_clock::now();
		// tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);

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


		/// 2D KEYPOINT DETECTION
		cv::Mat img_gray;
		cv::cvtColor(img_rgb, img_gray, CV_BGR2GRAY);
		// util::tick_high_resolution(start_t, tick, elapsed_initialization);
		cur_kpts = detectKeyPoints(img_gray);
		// util::tick_high_resolution(start_t, tick, elapsed_KP_detection);
		cur_desc = computeDescriptors(img_rgb, cur_kpts);
		// util::tick_high_resolution(start_t, tick, elapsed_KP_description);


		/// PRE-BACKPROJECTION
		CameraModel cam_model(info);
		// cam_model.printModel();

		cur_P_inv = cam_model.P().inverse().matrix();
		Eigen::Vector4d point_eigen;
		Eigen::Vector4d point_eigen_backproj;
		// util::tick_high_resolution(start_t, tick, elapsed_cam_model);
		

		///MATCHING
		if(entry_count == 0 || reset) //First time initialize map
		{
			old_P_inv=cur_P_inv;
			old_kpts=cur_kpts;
			old_desc=cur_desc;
			reset=false;
			entry_count++;
			return;
		}
		else //Attempt matching
		{
			matches = matchTwoImage(cur_desc, old_desc);
		}

		// util::tick_high_resolution(start_t, tick, elapsed_matching);


		/// BACKPROJECTION
		/// For every KP match backproject it using the depth value with lowest distance
		cv::KeyPoint kpt;
		double depth_yx;
		int old_u,old_v,cur_u,cur_v,ws;
		std::vector<float> x_cur, y_cur, z_cur;
		std::vector<float> x_old, y_old, z_old;
		for (const cv::DMatch& match : matches)
		{
			///Query KeyPoint
			kpt = cur_kpts[match.queryIdx];
			cur_u = kpt.pt.x;
			cur_v = kpt.pt.y;
			for(ws=1; ws<=DDKP_ws; ++ws)
			{
				if(img_depth.at<ushort>(cur_u,cur_v) != 0) break;
				nonZeroWindowContourLookUp(cur_u, cur_v, ws, img_depth);
			}
			if(img_depth.at<ushort>(cur_u,cur_v) == 0) break;
			
			///Train KeyPoint
			kpt = old_kpts[match.trainIdx];
			old_u = kpt.pt.x;
			old_v = kpt.pt.y;
			for(ws=1; ws<=DDKP_ws; ++ws)
			{
				if(img_depth.at<ushort>(old_u,old_v) != 0) break;
				nonZeroWindowContourLookUp(old_u, old_v, ws, img_depth);
			}
			if(img_depth.at<ushort>(old_u,old_v) == 0) break;

			//Save valid Match
			depth_yx = (double) img_depth.at<ushort>(cur_u,cur_v) / depthScale;
			point_eigen << (double) cur_u * depth_yx, (double) cur_v * depth_yx, (double) depth_yx, 1;
			point_eigen_backproj = cur_P_inv * point_eigen;
			x_cur.emplace_back((float) point_eigen_backproj(0));
			y_cur.emplace_back((float) point_eigen_backproj(1));
			z_cur.emplace_back((float) point_eigen_backproj(2));

			depth_yx = (double) img_depth.at<ushort>(old_u,old_v) / depthScale;
			point_eigen << (double) old_u * depth_yx, (double) old_v * depth_yx, (double) depth_yx, 1;
			point_eigen_backproj = old_P_inv * point_eigen;
			x_old.emplace_back((float) point_eigen_backproj(0));
			y_old.emplace_back((float) point_eigen_backproj(1));
			z_old.emplace_back((float) point_eigen_backproj(2));
		}

		//Log Matches size
		// std::cout<<"Original Matches size: "<<matches.size()<<std::endl;
		// std::cout<<"Final Matches size:"<<x_cur.size()<<std::endl;

		// util::tick_high_resolution(start_t, tick, elapsed_backprojection);

		/// PUBLISH
		/// - KeyPoints
		terreslam::KeyPointMatchesPtr kpm_msg_ptr(new terreslam::KeyPointMatches);
		kpm_msg_ptr->header.frame_id = keypoint_matches_frame_id;
		kpm_msg_ptr->header.stamp = info_msg->header.stamp;
		kpm_msg_ptr->x_cur = x_cur;
		kpm_msg_ptr->y_cur = y_cur;
		kpm_msg_ptr->z_cur = z_cur;
		kpm_msg_ptr->x_old = x_old;
		kpm_msg_ptr->y_old = y_old;
		kpm_msg_ptr->z_old = z_old;
		keypoint_matches_pub_.publish(kpm_msg_ptr);

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
		// util::printElapsed(elapsed_backprojection, "Backprojection: ");
		// util::printElapsed(elapsed_publish, "Publish: ");
		
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
	ros::Publisher keypoint_matches_pub_;
	image_transport::SubscriberFilter rgb_sub_filter_;
	image_transport::SubscriberFilter depth_sub_filter_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_filter_;
	
	typedef message_filters::sync_policies::ExactTime
		<sensor_msgs::Image,
		sensor_msgs::Image,
		sensor_msgs::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;
	
	///Chrono timmings
	// std::vector<double> elapsed;
	std::vector<double> elapsed_initialization;
	std::vector<double> elapsed_KP_detection;
	std::vector<double> elapsed_KP_description;
	std::vector<double> elapsed_cam_model;
	std::vector<double> elapsed_matching;
	std::vector<double> elapsed_backprojection;
	std::vector<double> elapsed_publish;

	///Kpts
	std::vector<cv::KeyPoint> cur_kpts;
	std::vector<cv::KeyPoint> old_kpts;
	cv::Mat cur_desc;
	cv::Mat old_desc;
	std::vector<cv::DMatch> matches;
	Eigen::Matrix4d cur_P_inv;
	Eigen::Matrix4d old_P_inv;
	bool reset=false;

	
};

PLUGINLIB_EXPORT_CLASS(terreslam::DDKeypointFrontend, nodelet::Nodelet);

}
