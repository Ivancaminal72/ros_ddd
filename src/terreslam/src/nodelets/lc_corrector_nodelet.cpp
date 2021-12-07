/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-09-10 11:00:48
 */

#include "terreslam/nodelet.h"
#include "terreslam/utils/util_types.h"
#include "terreslam/utils/util_chrono.h"
#include "terreslam/utils/util_general.h"

#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace terreslam
{

class LCCorrectorNodelet : public terreslam::Nodelet
{
public:
	LCCorrectorNodelet() :
		queue_size_(10)
		{
			// std::cout << "Constructor lc_corrector_nodelet..." << std::endl;
		}

private:

	void onNodeletInit()
	{
		std::cout << "Initalize lc_corrector_nodelet..." << std::endl;
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		/// Subscribers
		odom_sub_ = nh.subscribe(odom_topic, queue_size_, &LCCorrectorNodelet::callback, this);
	} 

	void callback(
		const nav_msgs::Odometry::ConstPtr& o_msg_ptr)
	{
		if(debug) std::cout << "Entry LCC: " << entry_count << std::endl;
		// ///Start chrono ticking
		// std::chrono::duration<double> tick;
		// std::chrono::high_resolution_clock::time_point end_t, start_t;
		// start_t = std::chrono::high_resolution_clock::now();
		// end_t = std::chrono::high_resolution_clock::now();
		// tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);

		RTr_acum.at<float>(0,3) = o_msg_ptr->pose.pose.position.x;
		RTr_acum.at<float>(1,3) = o_msg_ptr->pose.pose.position.y;
		RTr_acum.at<float>(2,3) = o_msg_ptr->pose.pose.position.z;
		geometry_msgs::Quaternion R_acum_msg_q = o_msg_ptr->pose.pose.orientation;
		tf2::Quaternion R_acum_tf2_q;
		tf2::fromMsg(R_acum_msg_q, R_acum_tf2_q);
		cv::Mat tmp = util::quat2Mat(R_acum_tf2_q);
		tmp.copyTo(RTr_acum(cv::Rect( 0, 0, 3, 3 )));
		

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
	ros::Subscriber odom_sub_;

	/// Chrono timmings
	std::vector<double> elapsed;

	/// MA Accum
	cv::Mat RTr_identity = (cv::Mat_<float>(4, 4)<<1, 0, 0, 0, 
																								 0, 1, 0, 0, 
																								 0, 0, 1, 0,
																								 0, 0, 0, 1);
	cv::Mat RTr_acum = RTr_identity.clone();

};

PLUGINLIB_EXPORT_CLASS(terreslam::LCCorrectorNodelet, nodelet::Nodelet);

}