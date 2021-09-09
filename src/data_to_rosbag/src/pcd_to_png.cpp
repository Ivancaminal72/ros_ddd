/*
Copyright (c) 2016, Helen Oleynikova, ETH Zurich, Switzerland
You can contact the author at <helen dot oleynikova at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/subscribe_options.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <boost/program_options.hpp>
#include <queue>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/StdVector>
#include <chrono>
#include <numeric>
#include <thread>
#include <mutex>

#include "data_to_rosbag/kitti_parser.h"
#include "data_to_rosbag/kittiraw_ros_conversions.h"

// mutex for critical section
std::mutex mtx;

//Chrono timmings
std::vector<double> elapsed_load_pcd_msg;
std::vector<double> elapsed_load_pcd;
std::vector<double> elapsed_process_pcd;
std::vector<double> elapsed_depth_store;
std::vector<double> elapsed_msg_enqueue;
std::vector<double> elapsed_callback;
std::vector<double> elapsed_callback_color;


void printElapsed(std::vector<double> elapsed_vec, std::string block_str)
{
    double average = std::accumulate(elapsed_vec.begin(), elapsed_vec.end(), 0.0) / elapsed_vec.size();
    std::cout << block_str << "\n";
    std::cout << "  avg: " << average << "s \n";
    std::cout << "  max: " << *max_element(elapsed_vec.begin(), elapsed_vec.end()) << "s \n";
    std::cout << "  min: " << *min_element(elapsed_vec.begin(), elapsed_vec.end()) << "s \n";
}


void tick_high_resolution(const std::chrono::high_resolution_clock::time_point& start_t,
                          std::chrono::duration<double>& tick,
                          std::vector<double>& elapsed_vec)
{
    std::chrono::high_resolution_clock::time_point end_t = std::chrono::high_resolution_clock::now();
    double tick_old = tick.count();
    tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);
    double elapsed = tick.count() - tick_old;
    elapsed_vec.push_back(elapsed);
}

uint64_t get_unsigned_difference(uint64_t first, uint64_t second) {
    uint64_t abs_diff = (first > second) ? (first - second) : (second - first);
    assert(abs_diff<=INT64_MAX);
    return abs_diff;
}

namespace po = boost::program_options;

//Constants
const uint kPow2_16= pow(2,16);
const uint kPow2_16_Minus1= pow(2,16)-1;
const double depthScale = pow(2,16)/120;

namespace adapt {

class PcdToPng {
 public:
  PcdToPng(const ros::NodeHandle& nh_sub, const ros::NodeHandle& nh_pub,
           const ros::NodeHandle& nh_private, const int cam_idx_proj,
           const double playback_multiplier);

  // Creates a timer to automatically publish entries in 'realtime' versus
  // the original data,
  void startRePublishing(double rate_hz);
  void timerCallback(const ros::WallTimerEvent& event);
  bool loadNextSyncTimestamp();
  void publishClock(uint64_t timestamp_ns);
  bool rePublishEntry();
  void processLidar(const pcl::PointCloud<pcl::PointXYZI> &msg);
  void processColor(const sensor_msgs::ImageConstPtr& image_msg,
                      const sensor_msgs::CameraInfoConstPtr& info_msg);

  static void processPcd(pcl::PointCloud<pcl::PointXYZI> pcd,
                                int width,
                                int height,
                                const Eigen::Affine3d T,
                                const Eigen::Matrix<double, 3, 4> P,
                                std::string pub_cam_frame_id,
                                std::string lidar_frame_id,
                                std::queue<pcl::PointCloud<pcl::PointXYZI>>& buffer_pcd_pub,
                                std::queue<sensor_msgs::Image>& buffer_depth_pub,
                                std::queue<sensor_msgs::CameraInfo>& buffer_depth_info_pub,
                                //compute_intensity// std::queue<sensor_msgs::Image>& buffer_infrared_pub,
                                //compute_intensity// std::queue<sensor_msgs::CameraInfo>& buffer_infrared_info_pub,
                                bool& is_first_pcd_processed);

 private:

  ros::NodeHandle nh_sub_;
  ros::NodeHandle nh_pub_;
  ros::NodeHandle nh_private_;

  // Subscribers for the topics.
  ros::Subscriber clock_sub_;
  ros::Subscriber lidar_sub_;

  image_transport::ImageTransport image_transport_sub_;
  image_transport::CameraSubscriber image_rgb_sub_;


  // Publishers for the topics.
  ros::Publisher clock_pub_;
  ros::Publisher lidar_pub_;
  ros::Publisher pose_pub_;

  image_transport::ImageTransport image_transport_pub_;
  image_transport::CameraPublisher image_rgb_pub_;
  //compute_intensity// image_transport::CameraPublisher image_infrared_pub_;
  image_transport::CameraPublisher image_depth_pub_;
  // std::vector<image_transport::CameraPublisher> image_pubs_;

  ros::WallTimer publish_timer_;

  std::string world_frame_id_;
  std::string imu_frame_id_;
  std::string cam_frame_id_prefix_;
  std::string lidar_frame_id_;
  std::string sub_cam_frame_id_;
  std::string pub_cam_frame_id_;
  std::string pub_lidar_frame_id_;

  uint64_t current_entry_;
  uint64_t publish_dt_ns_;
  uint64_t current_timestamp_ns_;
  uint64_t next_entry_timestamp_ns_;

  double playback_multiplier_;

  // std::queue<rosgraph_msgs::Clock> buffer_clock_;
  std::queue<pcl::PointCloud<pcl::PointXYZI>> buffer_pcd_sub_;
  std::queue<pcl::PointCloud<pcl::PointXYZI>> buffer_pcd_pub_;
  std::queue<sensor_msgs::Image> buffer_rgb_pub_;
  std::queue<sensor_msgs::CameraInfo> buffer_rgb_info_pub_;
  std::queue<sensor_msgs::Image> buffer_depth_pub_;
  std::queue<sensor_msgs::CameraInfo> buffer_depth_info_pub_;
  //compute_intensity// std::queue<sensor_msgs::Image> buffer_infrared_pub_;
  //compute_intensity// std::queue<sensor_msgs::CameraInfo> buffer_infrared_info_pub_;

  //Adaptation
  std::string prefix_ = "bag/";
  int cam_idx_proj_;
  std::map<std::string, int> width_;
  std::map<std::string, int> height_;
  std::map<std::string, Eigen::Matrix<double, 3, 4>, std::less<std::string>,
         Eigen::aligned_allocator<std::pair<const std::string, Eigen::Matrix<double, 3, 4>> > > Proj_;
  Eigen::Affine3d T_cam0_lidar_;

  //Management
  bool is_tf_ready_;
  bool is_first_pcd_processed_;
  int pcds_processed_=0;
  const uint64_t kMaxMessageDesync = 1*1e9;

};

PcdToPng::PcdToPng(const ros::NodeHandle& nh_sub,
                   const ros::NodeHandle& nh_pub,
                   const ros::NodeHandle& nh_private,
                   const int cam_idx_proj,
                   double playback_multiplier)
    : nh_sub_(nh_sub),
      nh_pub_(nh_pub),
      nh_private_(nh_private),
      image_transport_sub_(nh_sub_),
      image_transport_pub_(nh_pub_),
      world_frame_id_("world"),
      imu_frame_id_("imu"),
      cam_frame_id_prefix_("cam"),
      lidar_frame_id_("lidar"),
      current_entry_(0),
      current_timestamp_ns_(0),
      cam_idx_proj_(cam_idx_proj),
      is_tf_ready_(false),
      is_first_pcd_processed_(false),
      playback_multiplier_(playback_multiplier){

  sub_cam_frame_id_ = getSensorFrameId(prefix_+cam_frame_id_prefix_, cam_idx_proj_);
  // std::string pub_cam_frame_id_ = getSensorFrameId(cam_frame_id_prefix_, cam_idx_proj_);
  pub_cam_frame_id_ = "adapt/"+cam_frame_id_prefix_;
  pub_lidar_frame_id_ = "adapt/"+lidar_frame_id_;

  // Subcribe to the node_live / rosbag topics
  lidar_sub_ = nh_sub_.subscribe(prefix_+lidar_frame_id_, 20, &PcdToPng::processLidar, this); //queue x2 recomended size
  image_rgb_sub_ = image_transport_sub_.subscribeCamera(sub_cam_frame_id_, 16, &PcdToPng::processColor, this);  //queue x2 recomended size
  ROS_INFO("Subscribed for bag topics");

  // Advertise all the publishing topics for ROS live streaming.
  clock_pub_ = nh_pub_.advertise<rosgraph_msgs::Clock>("adapt/clock", 1, false);
  lidar_pub_ = nh_pub_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      pub_lidar_frame_id_, 10, false);
  image_rgb_pub_ = image_transport_pub_.advertiseCamera(pub_cam_frame_id_, 1);
  //compute_intensity// image_infrared_pub_ = image_transport_pub_.advertiseCamera(pub_cam_frame_id_ + "_infrared",1);
  image_depth_pub_ = image_transport_pub_.advertiseCamera(pub_cam_frame_id_ + "_depth",1);
  image_depth_pub_.getInfoTopic();
  ROS_INFO("Advertized publishing topics");

  //     nh_pub_.advertise<geometry_msgs::PoseStamped>("pose_imu", 10, false);
  // transform_pub_ = nh_pub_.advertise<geometry_msgs::TransformStamped>(
  //     "transform_imu", 10, false);

  //Get transformation
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener transform_listener(tf_buffer);

  while (nh_sub_.ok() && is_tf_ready_ == false)
  {
    geometry_msgs::TransformStamped Ts_cam_lidar;
    try
    {
      Ts_cam_lidar = tf_buffer.lookupTransform(prefix_+lidar_frame_id_,
              sub_cam_frame_id_, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    T_cam0_lidar_= tf2::transformToEigen(Ts_cam_lidar);
    static tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;

    //Publish identity T_cam_lidar
    Ts_cam_lidar.transform.rotation.x=0;
    Ts_cam_lidar.transform.rotation.y=0;
    Ts_cam_lidar.transform.rotation.z=0;
    Ts_cam_lidar.transform.rotation.w=1;
    Ts_cam_lidar.transform.translation.x=0;
    Ts_cam_lidar.transform.translation.y=0;
    Ts_cam_lidar.transform.translation.z=0;
    Ts_cam_lidar.header.frame_id = pub_lidar_frame_id_;
    Ts_cam_lidar.child_frame_id = pub_cam_frame_id_;
    static_tf_broadcaster.sendTransform(Ts_cam_lidar);

    Ts_cam_lidar.child_frame_id = pub_cam_frame_id_ + "_depth";
    static_tf_broadcaster.sendTransform(Ts_cam_lidar);

    //compute_intensity// Ts_cam_lidar.child_frame_id = pub_cam_frame_id_ + "_infrared";
    //compute_intensity// static_tf_broadcaster.sendTransform(Ts_cam_lidar);

    is_tf_ready_=true;
    ROS_INFO("OK - Tf is ready");
  }
}

void PcdToPng::startRePublishing(double rate_hz)
{
  while (is_first_pcd_processed_ == false)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if(!nh_sub_.ok()) exit(1);
  }

  while (loadNextSyncTimestamp() == false)
  {
    uint64_t difference = get_unsigned_difference(0, current_timestamp_ns_);
    if (difference >= kMaxMessageDesync)
    {
      ROS_ERROR("Max message desync achieved");
      exit(1);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    current_timestamp_ns_ += 100e6;
    if(!nh_sub_.ok()) exit(1);
  }



  std::this_thread::sleep_for(std::chrono::milliseconds(1000)); //Delay to account for processing time differences and allow subscribers to stablish connections




  //Start republishing
  double publish_dt_sec = 1.0 / rate_hz;
  publish_dt_ns_ = static_cast<uint64_t>(publish_dt_sec * 1e9);
  current_timestamp_ns_=0;
  // std::cout << "Publish dt ns: " << publish_dt_ns_ << std::endl;
  publish_timer_ = nh_pub_.createWallTimer(ros::WallDuration(publish_dt_sec),
                                       &PcdToPng::timerCallback, this);
}

void PcdToPng::timerCallback(const ros::WallTimerEvent& event)
{
  // std::cout << "Publish dt ns: " << publish_dt_ns_ << std::endl;
  current_timestamp_ns_ += publish_dt_ns_*playback_multiplier_; // Variation from realtime
  publishClock(current_timestamp_ns_);
  uint64_t difference = get_unsigned_difference(next_entry_timestamp_ns_, current_timestamp_ns_);

  if (difference >= kMaxMessageDesync)
  {
    std::cout<<"Diff: " <<difference<<std::endl;
    ROS_ERROR("Max message desync achieved");
    exit(1);
  }
  else if (next_entry_timestamp_ns_ <= current_timestamp_ns_)
  {
    std::cout << "Current entry: " << current_entry_ << std::endl;
    if (!rePublishEntry())
    {
      ROS_ERROR("Could not republish entry");
      exit(1);
    }
    current_entry_++;
    uint tries = 0;
    while (!loadNextSyncTimestamp())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds((int)(100/playback_multiplier_/10)));
      if (tries >= 10)
      {
        ROS_ERROR("Error next entry probably missed");
        current_entry_++;
        if (tries >= 20) exit(1);
      }
      tries++;
    }
    // std::cout<<"Next: "<< next_entry_timestamp_ns_<<std::endl;
    // std::cout<<"Curr: "<< current_timestamp_ns_<<std::endl
  }
}

bool PcdToPng::loadNextSyncTimestamp()
{
  mtx.lock();
  // // This value should be in MICROSECONDS, not nanoseconds.
  // next_entry_timestamp_ns_ = buffer_pcd_pub_.front().header.stamp * 1000;
  next_entry_timestamp_ns_ = buffer_pcd_pub_.front().header.stamp;
  uint64_t b_rgb = (uint64_t) buffer_rgb_pub_.front().header.stamp.toNSec();
  uint64_t b_rgb_info = (uint64_t) buffer_rgb_info_pub_.front().header.stamp.toNSec();
  uint64_t b_depth = (uint64_t) buffer_depth_pub_.front().header.stamp.toNSec();
  uint64_t b_depth_info = (uint64_t) buffer_depth_info_pub_.front().header.stamp.toNSec();
  //compute_intensity// uint64_t b_infrared = (uint64_t) buffer_infrared_pub_.front().header.stamp.toNSec();
  //compute_intensity// uint64_t b_infrared_info = (uint64_t) buffer_infrared_info_pub_.front().header.stamp.toNSec();
  mtx.unlock();

  // std::cout << std::setprecision (17)
  //           <<"1: " << next_entry_timestamp_ns_ <<std::endl
  //           <<"2: " << b_rgb <<std::endl
  //           <<"3: " << b_rgb_info <<std::endl
  //           <<"4: " << b_depth <<std::endl
  //           <<"5: " << b_depth_info <<std::endl;
  //           //compute_intensity// <<"6: " << b_infrared <<std::endl
  //           //compute_intensity// <<"7: " << b_infrared_info <<std::endl;
  
  if (next_entry_timestamp_ns_ != b_rgb ||
      next_entry_timestamp_ns_ != b_rgb_info ||
      next_entry_timestamp_ns_ != b_depth ||
      next_entry_timestamp_ns_ != b_depth_info
      //compute_intensity// next_entry_timestamp_ns_ != b_infrared ||
      //compute_intensity// next_entry_timestamp_ns_ != b_infrared_info
      )
      return false;

  else return true;
}

void PcdToPng::publishClock(uint64_t timestamp_ns)
{
  ros::Time timestamp_ros;
  timestampToRos(timestamp_ns, &timestamp_ros);
  rosgraph_msgs::Clock clock_time;
  clock_time.clock = timestamp_ros;
  clock_pub_.publish(clock_time);
}

bool PcdToPng::rePublishEntry()
{
  //Get messages
  sensor_msgs::PointCloud2 msg_pcd;
  pcl::toROSMsg(buffer_pcd_pub_.front(), msg_pcd);
  auto msg_rgb = buffer_rgb_pub_.front();
  auto msg_rgb_info = buffer_rgb_info_pub_.front();
  auto msg_depth = buffer_depth_pub_.front();
  auto msg_depth_info = buffer_depth_info_pub_.front();
  //compute_intensity// auto msg_infrared = buffer_infrared_pub_.front();
  //compute_intensity// auto msg_infrared_info = buffer_infrared_info_pub_.front();

  //Modifications
  msg_pcd.header.stamp = msg_depth.header.stamp;
  msg_rgb.header.frame_id = pub_cam_frame_id_;
  msg_rgb_info.header.frame_id = pub_cam_frame_id_;

  //Publish messages
  lidar_pub_.publish(msg_pcd);
  image_rgb_pub_.publish(msg_rgb, msg_rgb_info, msg_rgb.header.stamp);
  image_depth_pub_.publish(msg_depth, msg_depth_info, msg_depth.header.stamp);
  //compute_intensity// image_infrared_pub_.publish(msg_infrared, msg_infrared_info, msg_depth.header.stamp);

  //Delete published messages from queues
  mtx.lock();
  buffer_pcd_pub_.pop();
  buffer_rgb_pub_.pop();
  buffer_rgb_info_pub_.pop();
  buffer_depth_pub_.pop();
  buffer_depth_info_pub_.pop();
  //compute_intensity// buffer_infrared_pub_.pop();
  //compute_intensity// buffer_infrared_info_pub_.pop();
  mtx.unlock();

  return true;
}

void PcdToPng::processColor(const sensor_msgs::ImageConstPtr& image_msg_ptr,
                              const sensor_msgs::CameraInfoConstPtr& info_msg_ptr) {
  // std::cout<<"COOOLOR INSIDE"<<std::endl;

  // //Start chrono ticking
  // std::chrono::duration<double> tick;
  // std::chrono::high_resolution_clock::time_point end_t, start_t;
  // start_t = std::chrono::high_resolution_clock::now();
  // end_t = std::chrono::high_resolution_clock::now();
  // tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);

  if(info_msg_ptr->header.frame_id != sub_cam_frame_id_)
  {
    ROS_ERROR("Camera and info id not equal when processing callback");
    exit(1);
  }
  else
  {
    buffer_rgb_pub_.push(*image_msg_ptr);
    buffer_rgb_info_pub_.push(*info_msg_ptr);
  }

  if (width_.find(info_msg_ptr->header.frame_id) == width_.end())
  {
    width_[info_msg_ptr->header.frame_id] = (int) info_msg_ptr->width;
    height_[info_msg_ptr->header.frame_id] = (int) info_msg_ptr->height;

    for (size_t i = 0; i < 3; ++i) {
      for (size_t j = 0; j < 4; ++j) {
        Proj_[info_msg_ptr->header.frame_id](i, j) = info_msg_ptr->P[4 * i + j];
      }
    }
  }

  // std::cout<<"Color processed: "<<image_msg_ptr->header.stamp.toNSec()<<std::endl;

  // tick_high_resolution(start_t, tick, elapsed_callback_color);
  // printElapsed(elapsed_callback_color, "processColor total");
  // std::cout<<std::endl;

  // cv_bridge::CvImagePtr image_cv_ptr = rosToImagePtr(image_msg, sensor_msgs::image_encodings::BGR8);
}

void PcdToPng::processLidar(const pcl::PointCloud<pcl::PointXYZI> &msg) {
  // std::cout<<"STAMPCD INSIDE"<<std::endl;

  // //Start chrono ticking
  // std::chrono::duration<double> tick;
  // std::chrono::high_resolution_clock::time_point end_t, start_t;
  // start_t = std::chrono::high_resolution_clock::now();
  // end_t = std::chrono::high_resolution_clock::now();
  // tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);

  buffer_pcd_sub_.push(msg);

  if(!is_tf_ready_) return;
  else if (width_.find(sub_cam_frame_id_) == width_.end() &&
            height_.find(sub_cam_frame_id_) == height_.end() &&
              Proj_.find(sub_cam_frame_id_) == Proj_.end()) return;
  else
  {
    pcl::PointCloud<pcl::PointXYZI> pointcloud = buffer_pcd_sub_.front();
    buffer_pcd_sub_.pop();

    // tick_high_resolution(start_t, tick, elapsed_load_pcd_msg);

    // //Multi-core
    // std::thread th(processPcd,
    //               pointcloud,
    //               width_[sub_cam_frame_id_],
    //               height_[sub_cam_frame_id_],
    //               T_cam0_lidar_,
    //               Proj_[sub_cam_frame_id_],
    //               pub_cam_frame_id_,
    //               lidar_frame_id_,
    //               std::ref(buffer_pcd_pub_),
    //               std::ref(buffer_depth_pub_),
    //               std::ref(buffer_depth_info_pub_),
    //               //compute_intensity// std::ref(buffer_infrared_pub_),
    //               //compute_intensity// std::ref(buffer_infrared_info_pub_),
    //               std::ref(is_first_pcd_processed_));
    // th.detach();

    //Single-core
    processPcd(pointcloud,
              width_[sub_cam_frame_id_],
              height_[sub_cam_frame_id_],
              T_cam0_lidar_,
              Proj_[sub_cam_frame_id_],
              pub_cam_frame_id_,
              pub_lidar_frame_id_,
              std::ref(buffer_pcd_pub_),
              std::ref(buffer_depth_pub_),
              std::ref(buffer_depth_info_pub_),
              //compute_intensity// std::ref(buffer_infrared_pub_),
              //compute_intensity// std::ref(buffer_infrared_info_pub_),
              std::ref(is_first_pcd_processed_));

    width_.erase(sub_cam_frame_id_);
    height_.erase(sub_cam_frame_id_);
    Proj_.erase(sub_cam_frame_id_);
    // std::cout<<"STAMPCD: "<< std::setprecision (17) <<pointcloud.header.stamp<<std::endl;
    // std::cout<<"Pcd processed: "<<pcds_processed_<<std::endl;
    pcds_processed_++;

    // tick_high_resolution(start_t, tick, elapsed_callback);
    // printElapsed(elapsed_load_pcd_msg, "Load pcd msg");
    // printElapsed(elapsed_callback, "Callback total"); std::cout<<std::endl;
  }
}

void PcdToPng::processPcd(pcl::PointCloud<pcl::PointXYZI> pcd,
                                 int width,
                                 int height,
                                 Eigen::Affine3d T,
                                 Eigen::Matrix<double, 3, 4> P,
                                 std::string pub_cam_frame_id,
                                 std::string lidar_frame_id,
                                 std::queue<pcl::PointCloud<pcl::PointXYZI>>& buffer_pcd_pub,
                                 std::queue<sensor_msgs::Image>& buffer_depth_pub,
                                 std::queue<sensor_msgs::CameraInfo>& buffer_depth_info_pub,
                                 //compute_intensity// std::queue<sensor_msgs::Image>& buffer_infrared_pub,
                                 //compute_intensity// std::queue<sensor_msgs::CameraInfo>& buffer_infrared_info_pub,
                                 bool& is_first_pcd_processed)
{
  // //Start chrono ticking
  // std::chrono::duration<double> tick;
  // std::chrono::high_resolution_clock::time_point end_t, start_t;
  // start_t = std::chrono::high_resolution_clock::now();
  // end_t = std::chrono::high_resolution_clock::now();
  // tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);

  cv::Mat depth_img;
  //compute_intensity// cv::Mat infrared_img;
  Eigen::Array3Xd ddd_pts_p;
  Eigen::RowVectorXd depth_pts;
  //compute_intensity// Eigen::RowVectorXd intensity_pts;
  Eigen::Matrix4Xd ddd_pts_h;

  //Load matrices
  size_t numPts = pcd.size();
  ddd_pts_h = Eigen::Matrix4Xd::Zero(4, numPts);
  //compute_intensity// intensity_pts = Eigen::RowVectorXd::Zero(1, numPts);
  unsigned i = 0;
  for(auto& point : pcd.points)
  {
    ddd_pts_h.col(i) << (double) point.x, (double) point.y, (double) point.z, 1.0;
    //compute_intensity// intensity_pts.col(i) << (double) point.intensity;
    ++i;
  }
  // tick_high_resolution(start_t, tick, elapsed_load_pcd);

  // Start projection
  ddd_pts_h = T * ddd_pts_h;
  ddd_pts_p = (P * ddd_pts_h).array();
  ddd_pts_p.row(0) /= ddd_pts_p.row(2);
  ddd_pts_p.row(1) /= ddd_pts_p.row(2);
  depth_pts = ddd_pts_h.row(2);
  depth_pts = depth_pts*depthScale;
  depth_img = cv::Mat::zeros(height, width, CV_16UC1);
  //compute_intensity// infrared_img = cv::Mat::zeros(height, width, CV_16UC1);
  // tick_high_resolution(start_t, tick, elapsed_process_pcd);

  // uint inside=0, outside=0, valid=0;
  int x, y;
  ushort d;
  //Store valid depth values
  for(int i=0; i<numPts; i++)
  {
      //store registered pointcloud
      pcd.at(i).x = (float) ddd_pts_h(0,i);
      pcd.at(i).y = (float) ddd_pts_h(1,i);
      pcd.at(i).z = (float) ddd_pts_h(2,i);

      x = round(ddd_pts_p(0,i));
      y = round(ddd_pts_p(1,i));

      //consider only points projected within camera sensor
      if(x<width && x>=0 && y<height && y>=0)
      {
        // inside +=1;
        //only positive depth
        if(round(depth_pts(0,i))>0)
        {
          // valid+=1;
          d = (ushort) round(depth_pts(0,i));

          //pixel need to be initalized or updated with a smaller depth
          if(depth_img.at<ushort>(y,x) == 0 || depth_img.at<ushort>(y,x) > d)
          {
            //exceed established limit (save max)
            if(d>=kPow2_16) depth_img.at<ushort>(y,x) = kPow2_16_Minus1;

            //inside limit (save the smaller sensed depth)
            else depth_img.at<ushort>(y,x) = d;

            //Save 16bit intensity
            //compute_intensity// infrared_img.at<ushort>(y,x) = (ushort) trunc(intensity_pts(0,i) * kPow2_16);
          }
        }
      }
      // else outside+=1;
  }
  // tick_high_resolution(start_t, tick, elapsed_depth_store);

  //Enqueue translated pointcloud messages
  pcd.header.frame_id = lidar_frame_id;
  mtx.lock();
  buffer_pcd_pub.push(pcd);
  mtx.unlock();

  //Enqueue depth/infrared image messages
  ros::Time timestamp_ros;
  timestampToRos(pcd.header.stamp, &timestamp_ros);

  sensor_msgs::Image image_depth_msg;
  //compute_intensity// sensor_msgs::Image image_infrared_msg;
  imageToRos(depth_img, &image_depth_msg);
  //compute_intensity// imageToRos(infrared_img, &image_infrared_msg);

  CameraCalibration cam_calib;
  sensor_msgs::CameraInfo cam_info;
  cam_calib.image_size << width, height;
  cam_calib.projection_mat = P;
  calibrationToRos(pub_cam_frame_id, cam_calib, &cam_info);

  image_depth_msg.header.stamp = timestamp_ros;
  image_depth_msg.header.frame_id = pub_cam_frame_id + "_depth";
  cam_info.header = image_depth_msg.header;

  mtx.lock();
  buffer_depth_pub.push(image_depth_msg);
  buffer_depth_info_pub.push(cam_info);
  mtx.unlock();

  //compute_intensity// image_infrared_msg.header.stamp = timestamp_ros;
  //compute_intensity// image_infrared_msg.header.frame_id = pub_cam_frame_id + "_infrared";
  //compute_intensity// cam_info.header = image_infrared_msg.header;

  //compute_intensity// mtx.lock();
  //compute_intensity// buffer_infrared_pub.push(image_infrared_msg);
  //compute_intensity// buffer_infrared_info_pub.push(cam_info);
  //compute_intensity// mtx.unlock();

  if(!is_first_pcd_processed) is_first_pcd_processed=true;

  // tick_high_resolution(start_t, tick, elapsed_msg_enqueue);
  // printElapsed(elapsed_load_pcd, "Load pcd");
  // printElapsed(elapsed_process_pcd, "Projection");
  // printElapsed(elapsed_depth_store, "Store depth");
  // printElapsed(elapsed_msg_enqueue, "Enqueue msg");
}


}  // namespace adapt

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcd_to_png");
  ros::NodeHandle nh_sub, nh_pub;
  ros::NodeHandle nh_private("~");


  //Declare variables
  int cam_idx_proj = -1;
  double play_mult = -1;

  //Parse arguments
  po::options_description mandatory_opts("Mandatory args");
  mandatory_opts.add_options()
    ("project,p", po::value<int>(&cam_idx_proj), "index of the camera to do pcd projection")
    ("multiplayback,r", po::value<double>(&play_mult), "multiplier reproduction to realtime")
    ;

  po::positional_options_description positional_opts;
  positional_opts.add("project", -1);

  po::options_description optional_opts("Optional args");
  optional_opts.add_options()
    ("help,h", "produce a help message");

  po::variables_map vm;
  po::options_description all_opts;
  all_opts.add(mandatory_opts).add(optional_opts);
  po::store(po::command_line_parser(argc, argv).options(all_opts).positional(positional_opts).run(), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << "Usage: rosrun data_to_rosbag pcd_to_png {args}\n";
    std::cout << all_opts << std::endl << std::endl;
    std::cout << "Warn: the 1st mandatory arg can be directly specified (without option)\n";
    return 0;
  }

  for(const auto& opt : mandatory_opts.options())
  {
    if(!vm.count(opt->long_name()))
    {
      std::cerr << "Missing mandatory_opts option " + opt->long_name() + "\n";
      std::cout << "Try --help for more information\n";
      return 1;
    }
  }

  //Ready to start
  ros::CallbackQueue pub_queue, sub_queue;
  nh_sub.setCallbackQueue(&sub_queue);
  nh_pub.setCallbackQueue(&pub_queue);
  adapt::PcdToPng node(nh_sub, nh_pub, nh_private, cam_idx_proj, play_mult);

  ros::AsyncSpinner spinner_sub(4, &sub_queue);
  ros::AsyncSpinner spinner_pub(1, &pub_queue);
  spinner_sub.start();
  spinner_pub.start();
  node.startRePublishing(50.0);

  // bool enable_sub = true;
  // bool trigger_sub = true;

  // // Loop with 100 Hz rate
  // ros::Rate loop_rate(100);
  // while (ros::ok())
  // {
  //   // Enable state changed
  //   if (trigger_sub)
  //   {
  //     if (enable_sub)
  //     {
  //       // Clear old callback from the queue
  //       sub_queue.clear();
  //       // Start the spinner
  //       spinner_sub.start();
  //       ROS_INFO("Spinner enabled");
  //     }
  //     else
  //     {
  //       // Stop the spinner
  //       spinner_sub.stop();
  //       ROS_INFO("Spinner disabled");
  //     }
  //     // Reset trigger
  //     trigger_sub = false;
  //   }

  //   // Process messages on global callback queue
  //   sub_queue.callOne(ros::WallDuration());
  //   loop_rate.sleep();
  // }

  // Wait for ROS threads to terminate
  ros::waitForShutdown();
}
