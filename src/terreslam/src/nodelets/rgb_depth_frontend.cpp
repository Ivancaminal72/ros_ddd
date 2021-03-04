/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-19 11:47:07
 *    Last Modified: 2021-03-04 12:56:28
 */

#include "terreslam/frontend.h"
#include "terreslam/camera_model.h"
#include "terreslam/utils/util_msg.h"
#include "terreslam/utils/util_map.h"
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

class RGBDepthFrontend : public terreslam::Frontend
{
public:
	RGBDepthFrontend() :
		Frontend(),
		queue_size_(10)
		{
			std::cout << "Constructor rgb_depth_frontend..." << std::endl;
		}

private:

	virtual void onFrontendInit()
	{
		std::cout << "Initalize rgb_depth_frontend..." << std::endl;
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

		//Subscribers
		rgb_sub_.subscribe(rgb_it, sub_cam_frame_id_, 1, hintsRgb);
		depth_sub_.subscribe(depth_it, sub_cam_depth_frame_id_, 1, hintsDepth);
		info_sub_.subscribe(rgb_nh, sub_cam_info_frame_id_, 1);

		exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>
			(MyExactSyncPolicy(queue_size_), rgb_sub_, depth_sub_, info_sub_);
		exactSync_->registerCallback(boost::bind(&RGBDepthFrontend::callback, this, _1, _2, _3));
	} 

	void callback(
		const sensor_msgs::ImageConstPtr& rgb_msg,
		const sensor_msgs::ImageConstPtr& depth_msg,
		const sensor_msgs::CameraInfoConstPtr& info_msg)
	{
		std::cout << "Entry: " << entry_count_ << std::endl;
		// std::cout << rgb_msg->header.stamp << std::endl;
		// std::cout << depth_msg->header.stamp << std::endl;
		// std::cout << info_msg->header.stamp << std::endl;

		cv_bridge::CvImageConstPtr ptr_msg_rgb = cv_bridge::toCvShare(rgb_msg);
		cv_bridge::CvImageConstPtr ptr_msg_depth = cv_bridge::toCvShare(depth_msg);
		sensor_msgs::CameraInfo info = *info_msg;

		/// INITALIZATION
		uint32_t height, width;
		if (rgb_msg->height == depth_msg->height) height = rgb_msg->height; else {skipFrame("Different img height"); return;}
		if (rgb_msg->width == depth_msg->width) width = rgb_msg->width; else {skipFrame("Different img width"); return;}
		cv::Mat img_rgb = cv::Mat(rgb_msg->height, rgb_msg->width, ptr_msg_rgb->image.type());
		cv::Mat img_depth = cv::Mat(depth_msg->height, depth_msg->width, ptr_msg_depth->image.type());
		ptr_msg_rgb->image.copyTo(cv::Mat(img_rgb, cv::Rect(0, 0, rgb_msg->width, rgb_msg->height)));
		ptr_msg_depth->image.copyTo(cv::Mat(img_depth, cv::Rect(0, 0, depth_msg->width, depth_msg->height)));

		/// BACKPROJECTION
		CameraModel cam_model(info);
		// cam_model.printModel();

		/// Pointer to the Mat data
		uint8_t *rgb_ptr;
		rgb_ptr=img_rgb.data;

		scan_ = new Scan();
		scan_->points() = ptrPointCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		scan_->normals() = ptrNormalCloud (new pcl::PointCloud<pcl::Normal>);
		scan_->pixels() = ptrPixelCloud (new pcl::PointCloud<pcl::PointXY>);
		Eigen::Vector4d point_eigen;
		Eigen::Vector4d point_eigen_backproj;
		Eigen::Matrix4d P_inv = cam_model.P().inverse().matrix();
		pcl::PointXYZRGBA point_pcl;
		pcl::PointXY tmp_pointxy;
		/// clear the pointcloud 
		/// the allocated memory does not release
		/// the newly pushed elements cover the old ones
		scan_->points()->clear();
		scan_->normals()->clear();
		scan_->pixels()->clear();
		
		for (int v = 0; v < img_depth.rows; ++v)
			for (int u = 0; u < img_depth.cols; ++u)
			{
				double depth_yx = (double) img_depth.at<ushort>(v, u) / depthScale;
				if(depth_yx != 0)
				{
					/// 3 channels for one pixel in rgb image;
					point_pcl.b=*rgb_ptr;
					rgb_ptr++;
					point_pcl.g=*rgb_ptr;
					rgb_ptr++;
					point_pcl.r=*rgb_ptr;
					rgb_ptr++;
					tmp_pointxy.x=u;
					tmp_pointxy.y=v;
					point_eigen << (double) u * depth_yx, (double) v * depth_yx, (double) depth_yx, 1;
					point_eigen_backproj = P_inv * point_eigen;
					point_pcl.x = (float) point_eigen_backproj(0);
					point_pcl.y = (float) point_eigen_backproj(1);
					point_pcl.z = (float) point_eigen_backproj(2);
					scan_->points()->push_back(point_pcl);
					scan_->pixels()->push_back(tmp_pointxy);
				}
				else if (use_normal_integral_)
				{
					/// 3 channels for one pixel in rgb image
					point_pcl.b=*rgb_ptr;
					rgb_ptr++;
					point_pcl.g=*rgb_ptr;
					rgb_ptr++;
					point_pcl.r=*rgb_ptr;
					rgb_ptr++;
					point_pcl.x = point_pcl.y = point_pcl.z = bad_point;
					scan_->points()->push_back(point_pcl);
				}
				else rgb_ptr+=3;
			}

		/// NORMALS
		if(use_normal_integral_)
		{
			/// Organize the point_cloud for the normal estimation
			scan_->points()->width=width;
			scan_->points()->height=height;
			/// Generate the normal_cloud
			/// More methods --> AVERAGE_3D_GRADIENT; AVERAGE_DEPTH_CHANGE; COVARIANCE_MATRIX
			ne_integral.setNormalEstimationMethod(pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA,pcl::Normal>::AVERAGE_DEPTH_CHANGE);
			ne_integral.setDepthDependentSmoothing(true);
			ne_integral.setNormalSmoothingSize(40.0);
			ne_integral.setInputCloud(scan_->points());
			ne_integral.compute(*scan_->normals());
		}
		else
		{
			ne.setInputCloud(scan_->points());
			tree=pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGBA>());
  		ne.setSearchMethod(tree);
			ne.setRadiusSearch(1);
			ne.compute(*scan_->normals());
		}

		/// Visualize normals
		// Vis_.NormalView1(point_cloud, normal_cloud);

		/// Write normals
		// if(entry_count_ == 0) Disk_.WriteNormals(point_cloud, normal_cloud);

		/// PLANE DETECTOR
		PD_.detectPlanes(scan_);
		

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp_plane (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scan_planes (new pcl::PointCloud<pcl::PointXYZRGBA>);
		for(iterFeature it=scan_->beginFeature();it!=scan_->endFeature();it++)
		{
			if(it->second->Type()!=PLANE) continue;
			pcl::copyPointCloud(*it->second->ptrPoints(),*it->second->ptrIndices(),*tmp_plane);
			// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(tmp_plane,0,255,0);
			// uint8_t r = 255;
			// uint8_t g = 0;
			// uint8_t b = 0;
			// int32_t rgb = (r << 16) | (g << 8) | b; 
			// for(auto &p: tmp_plane->points) p.rgb=rgb;
			*scan_planes += *tmp_plane;
		}

		/// PUBLISH
		/// - Planes
		sensor_msgs::PointCloud2 msg_pcd;
		pcl::toROSMsg(*scan_planes, msg_pcd);
		msg_pcd.header.frame_id = "/terreslam/cloud/plane";
		plane_pub_.publish(msg_pcd);

		/// - Cloud
		pcl::toROSMsg(*scan_->points(), msg_pcd);
		msg_pcd.header.frame_id = "/terreslam/cloud";
		cloud_pub_.publish(msg_pcd);

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
	Scan* scan_;
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree;
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne_integral;
	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;

	/// Comms
	image_transport::SubscriberFilter rgb_sub_;
	image_transport::SubscriberFilter depth_sub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
	
	typedef message_filters::sync_policies::ExactTime
		<sensor_msgs::Image,
		sensor_msgs::Image,
		sensor_msgs::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;
};

PLUGINLIB_EXPORT_CLASS(terreslam::RGBDepthFrontend, nodelet::Nodelet);

}