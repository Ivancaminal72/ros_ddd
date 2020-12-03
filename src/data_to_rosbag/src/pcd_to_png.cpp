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
std::vector<double> elapsed_load_pcd;
std::vector<double> elapsed_process_pcd;
std::vector<double> elapsed_depth_store;
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

namespace po = boost::program_options;

//Constants
const uint kPow2_16= pow(2,16);
const uint kPow2_16_Minus1= pow(2,16)-1;

namespace adapt {

class PcdToPng {
 public:
  PcdToPng(const ros::NodeHandle& nh_sub, const ros::NodeHandle& nh_pub,
           const ros::NodeHandle& nh_private, const int cam_idx_proj);

  // Creates a timer to automatically publish entries in 'realtime' versus
  // the original data,
  void start();
  void rePublishClock(const rosgraph_msgs::Clock &msg);
  void rePublishLidar(const pcl::PointCloud<pcl::PointXYZI> &msg);
  void rePublishColor(const sensor_msgs::ImageConstPtr& image_msg,
                      const sensor_msgs::CameraInfoConstPtr& info_msg);

  static void projectAndPublish(pcl::PointCloud<pcl::PointXYZI> pcd,
                                int width,
                                int height,
                                const Eigen::Affine3d T,
                                const Eigen::Matrix<double, 3, 4> P,
                                std::string pub_cam_frame_id,
                                std::string lidar_frame_id,
                                ros::Publisher& lidar_pub,
                                image_transport::CameraPublisher& image_depth_pub,
                                image_transport::CameraPublisher& image_infrared_pub,
                                bool& is_first_pcd_republished);

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
  image_transport::CameraPublisher image_infrared_pub_;
  image_transport::CameraPublisher image_depth_pub_;
  // std::vector<image_transport::CameraPublisher> image_pubs_;

  // Decides the rate of publishing (TF is published at this rate).
  // Call startPublishing() to turn this on.

  std::string world_frame_id_;
  std::string imu_frame_id_;
  std::string cam_frame_id_prefix_;
  std::string lidar_frame_id_;

  uint64_t current_entry_;
  uint64_t publish_dt_ns_;
  uint64_t current_timestamp_ns_;

  std::queue<rosgraph_msgs::Clock> buffer_clock_;
  std::queue<pcl::PointCloud<pcl::PointXYZI>> buffer_pcd_;
  std::queue<sensor_msgs::Image> buffer_rgb_;
  std::queue<sensor_msgs::CameraInfo> buffer_rgb_info_;
  
  //Adaptation
  std::string prefix_ = "bag_";
  int cam_idx_proj_;
  std::map<std::string, int> width_;
  std::map<std::string, int> height_;
  std::map<std::string, Eigen::Matrix<double, 3, 4>, std::less<std::string>, 
         Eigen::aligned_allocator<std::pair<const std::string, Eigen::Matrix<double, 3, 4>> > > Proj_;
  Eigen::Affine3d T_cam0_lidar_;
  
  //Management
  bool is_tf_ready_;
  bool is_first_pcd_republished_;
  int pcds_republished_=0;

};

PcdToPng::PcdToPng(const ros::NodeHandle& nh_sub,
                   const ros::NodeHandle& nh_pub,
                   const ros::NodeHandle& nh_private,
                   const int cam_idx_proj)
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
      is_first_pcd_republished_(false){

  std::string sub_cam_frame_id = getSensorFrameId(prefix_+cam_frame_id_prefix_, cam_idx_proj_);
  std::string pub_cam_frame_id = getSensorFrameId(cam_frame_id_prefix_, cam_idx_proj_);
  
  // Subcribe to the node_live / rosbag topics
  clock_sub_ = nh_sub_.subscribe(prefix_+"clock", 1, &PcdToPng::rePublishClock, this);
  lidar_sub_ = nh_sub_.subscribe(prefix_+lidar_frame_id_, 1, &PcdToPng::rePublishLidar, this);
  image_rgb_sub_ = image_transport_sub_.subscribeCamera(sub_cam_frame_id, 1, &PcdToPng::rePublishColor, this);
  ROS_INFO("Subscribed for bag topics");
   
  // Advertise all the publishing topics for ROS live streaming.
  clock_pub_ = nh_pub_.advertise<rosgraph_msgs::Clock>("/clock", 1, false);
  lidar_pub_ = nh_pub_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      lidar_frame_id_, 10, false);
  image_rgb_pub_ = image_transport_pub_.advertiseCamera(pub_cam_frame_id, 1);
  image_infrared_pub_ = image_transport_pub_.advertiseCamera(pub_cam_frame_id + "_infrared",1);
  image_depth_pub_ = image_transport_pub_.advertiseCamera(pub_cam_frame_id + "_depth",1);
  ROS_INFO("Advertized publishing topics");
  
  //     nh_pub_.advertise<geometry_msgs::PoseStamped>("pose_imu", 10, false);
  // transform_pub_ = nh_pub_.advertise<geometry_msgs::TransformStamped>(
  //     "transform_imu", 10, false);
}

void PcdToPng::start() {
  
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener transform_listener(tf_buffer);
  
  while (nh_sub_.ok() && is_tf_ready_ == false)
  {
    geometry_msgs::TransformStamped Ts_cam_lidar;
    try
    {
      Ts_cam_lidar = tf_buffer.lookupTransform(prefix_+lidar_frame_id_, 
              getSensorFrameId(prefix_+cam_frame_id_prefix_, cam_idx_proj_), ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    
    T_cam0_lidar_= tf2::transformToEigen(Ts_cam_lidar);
    static tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;
    
    //Publish identity T_cam_lidar 
    Ts_cam_lidar.header.frame_id = lidar_frame_id_;
    Ts_cam_lidar.child_frame_id = getSensorFrameId(cam_frame_id_prefix_, cam_idx_proj_);
    Ts_cam_lidar.transform.rotation.x=0;
    Ts_cam_lidar.transform.rotation.y=0;
    Ts_cam_lidar.transform.rotation.z=0;
    Ts_cam_lidar.transform.rotation.w=1;
    Ts_cam_lidar.transform.translation.x=0;
    Ts_cam_lidar.transform.translation.y=0;
    Ts_cam_lidar.transform.translation.z=0;
    static_tf_broadcaster.sendTransform(Ts_cam_lidar);

    is_tf_ready_=true;
    ROS_INFO("OK - Tf is ready");
  }
}

void PcdToPng::rePublishClock(const rosgraph_msgs::Clock &msg) {
  
  buffer_clock_.push(msg);
  
  if(!is_first_pcd_republished_) return;
  else
  {
    clock_pub_.publish(buffer_clock_.front());
    buffer_clock_.pop();
  }
  
}

void PcdToPng::rePublishColor(const sensor_msgs::ImageConstPtr& image_msg_ptr,
                              const sensor_msgs::CameraInfoConstPtr& info_msg_ptr) {                              
  
  // //Start chrono ticking
  // std::chrono::duration<double> tick;
  // std::chrono::high_resolution_clock::time_point end_t, start_t;
  // start_t = std::chrono::high_resolution_clock::now();
  // end_t = std::chrono::high_resolution_clock::now();
  // tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);                               
  
  buffer_rgb_.push(*image_msg_ptr);
  buffer_rgb_info_.push(*info_msg_ptr);

  if (width_.find(info_msg_ptr->header.frame_id) == width_.end())
  {
    width_[info_msg_ptr->header.frame_id] = (int) info_msg_ptr->width;
    height_[info_msg_ptr->header.frame_id] = (int) info_msg_ptr->height;
    
    for (size_t i = 0; i < 3; ++i) {
      for (size_t j = 0; j < 4; ++j) {
        Proj_[info_msg_ptr->header.frame_id](i, j) = info_msg_ptr->P[4 * i + j];
      }
    }

    info_msg_ptr->P;
  }
  else if(!is_first_pcd_republished_) return;
  else
  {
    sensor_msgs::Image image_msg = buffer_rgb_.front();
    image_msg.header.frame_id = getSensorFrameId(cam_frame_id_prefix_, cam_idx_proj_);
    image_rgb_pub_.publish(image_msg, buffer_rgb_info_.front(), image_msg.header.stamp);
    buffer_rgb_info_.pop();
    buffer_rgb_.pop();
  }

  // tick_high_resolution(start_t, tick, elapsed_callback_color);
  // printElapsed(elapsed_callback_color, "rePublishColor total");
  // std::cout<<std::endl;

  // cv_bridge::CvImagePtr image_cv_ptr = rosToImagePtr(image_msg, sensor_msgs::image_encodings::BGR8);
}

void PcdToPng::rePublishLidar(const pcl::PointCloud<pcl::PointXYZI> &msg) {

  // //Start chrono ticking
  // std::chrono::duration<double> tick;
  // std::chrono::high_resolution_clock::time_point end_t, start_t;
  // start_t = std::chrono::high_resolution_clock::now();
  // end_t = std::chrono::high_resolution_clock::now();
  // tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);

  buffer_pcd_.push(msg);
  std::string sub_cam_frame_id = getSensorFrameId(prefix_+cam_frame_id_prefix_, cam_idx_proj_);
  
  if(!is_tf_ready_) return;
  else if (width_.find(sub_cam_frame_id) != width_.end() && 
            height_.find(sub_cam_frame_id) != height_.end() &&
              Proj_.find(sub_cam_frame_id) != Proj_.end())
  {    
    pcl::PointCloud<pcl::PointXYZI> pointcloud = buffer_pcd_.front();
    buffer_pcd_.pop();
    int width = width_[sub_cam_frame_id];
    int height = height_[sub_cam_frame_id];
    std::string pub_cam_frame_id = getSensorFrameId(cam_frame_id_prefix_, cam_idx_proj_);
    std::thread th(projectAndPublish, 
                  pointcloud, 
                  width_[sub_cam_frame_id],
                  height_[sub_cam_frame_id],
                  T_cam0_lidar_,
                  Proj_[sub_cam_frame_id],
                  pub_cam_frame_id,
                  lidar_frame_id_,
                  std::ref(lidar_pub_),
                  std::ref(image_depth_pub_),
                  std::ref(image_infrared_pub_),
                  std::ref(is_first_pcd_republished_));
    th.detach();

    // projectAndPublish(pointcloud, 
    //                   width_[sub_cam_frame_id],
    //                   height_[sub_cam_frame_id],
    //                   T_cam0_lidar_,
    //                   Proj_[sub_cam_frame_id],
    //                   pub_cam_frame_id,
    //                   lidar_frame_id_,
    //                   lidar_pub_,
    //                   image_depth_pub_,
    //                   image_infrared_pub_,
    //                   is_first_pcd_republished_);

    pcds_republished_++;
    std::cout<<"Pcd republished: "<<pcds_republished_<<std::endl;
    
    // tick_high_resolution(start_t, tick, elapsed_callback);
    // printElapsed(elapsed_callback, "Callback total");
    // std::cout<<std::endl;
  }
}

void PcdToPng::projectAndPublish(pcl::PointCloud<pcl::PointXYZI> pcd,
                                 int width,
                                 int height,
                                 Eigen::Affine3d T,
                                 Eigen::Matrix<double, 3, 4> P,
                                 std::string pub_cam_frame_id,
                                 std::string lidar_frame_id,
                                 ros::Publisher& lidar_pub,
                                 image_transport::CameraPublisher& image_depth_pub,
                                 image_transport::CameraPublisher& image_infrared_pub,
                                 bool& is_first_pcd_republished)
{
  // //Start chrono ticking
  // std::chrono::duration<double> tick;
  // std::chrono::high_resolution_clock::time_point end_t, start_t;
  // start_t = std::chrono::high_resolution_clock::now();
  // end_t = std::chrono::high_resolution_clock::now();
  // tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);

  cv::Mat depth_img, infrared_img;
  Eigen::Array3Xd ddd_pts_p;
  Eigen::RowVectorXd depth_pts;
  Eigen::RowVectorXd intensity_pts;
  Eigen::Matrix4Xd ddd_pts_h;

  //Load matrices
  size_t numPts = pcd.size();
  ddd_pts_h = Eigen::Matrix4Xd::Zero(4, numPts);
  intensity_pts = Eigen::RowVectorXd::Zero(1, numPts);
  unsigned i = 0;
  for(auto& point : pcd.points)
  {
    ddd_pts_h.col(i) << (double) point.x, (double) point.y, (double) point.z, 1.0;
    intensity_pts.col(i) << (double) point.intensity;
    ++i;
  }
  // tick_high_resolution(start_t, tick, elapsed_load_pcd);

  // Start projection
  ddd_pts_h = T * ddd_pts_h;
  ddd_pts_p = (P * ddd_pts_h).array();
  ddd_pts_p.row(0) /= ddd_pts_p.row(2);
  ddd_pts_p.row(1) /= ddd_pts_p.row(2);
  depth_pts = ddd_pts_h.row(2);
  depth_img = cv::Mat::zeros(height, width, CV_16UC1);
  infrared_img = cv::Mat::zeros(height, width, CV_16UC1);
  // tick_high_resolution(start_t, tick, elapsed_process_pcd);

  uint inside=0, outside=0, valid=0;
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
            infrared_img.at<ushort>(y,x) = (ushort) trunc(intensity_pts(0,i) * kPow2_16); 
          }
        }
      }
      // else outside+=1;
  }
  // tick_high_resolution(start_t, tick, elapsed_depth_store);

  //Publish projected pointcloud
  pcd.header.frame_id = lidar_frame_id;
  mtx.lock();
  lidar_pub.publish(pcd);
  mtx.unlock();      

  //Publish depth/infrared images
  ros::Time timestamp_ros;
  timestampToRos(pcd.header.stamp * 1000, &timestamp_ros);

  sensor_msgs::Image image_depth_msg;
  sensor_msgs::Image image_infrared_msg;
  imageToRos(depth_img, &image_depth_msg);
  imageToRos(infrared_img, &image_infrared_msg);

  CameraCalibration cam_calib;
  sensor_msgs::CameraInfo cam_info;
  cam_calib.image_size << width, height;
  cam_calib.projection_mat = P;
  calibrationToRos(pub_cam_frame_id, cam_calib, &cam_info);

  image_depth_msg.header.stamp = timestamp_ros;
  image_depth_msg.header.frame_id = pub_cam_frame_id;
  cam_info.header = image_depth_msg.header;

  image_infrared_msg.header.stamp = timestamp_ros;
  image_infrared_msg.header.frame_id = pub_cam_frame_id;
  cam_info.header = image_infrared_msg.header;
  
  mtx.lock();
  image_depth_pub.publish(image_depth_msg, cam_info, timestamp_ros);
  image_infrared_pub.publish(image_infrared_msg, cam_info, timestamp_ros);
  mtx.unlock();

  if(!is_first_pcd_republished) is_first_pcd_republished=true;

  // printElapsed(elapsed_load_pcd, "Load pcd");
  // printElapsed(elapsed_process_pcd, "Projection");
  // printElapsed(elapsed_depth_store, "Store depth");

}


}  // namespace adapt

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcd_to_png");
  ros::NodeHandle nh_sub, nh_pub;
  ros::NodeHandle nh_private("~");


  //Declare variables
  int cam_idx_proj = -1;

  //Parse arguments
  po::options_description mandatory_opts("Mandatory args");
  mandatory_opts.add_options()
    ("project,p", po::value<int>(&cam_idx_proj), "Do pcd projection to camera idx");
  
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
  adapt::PcdToPng node(nh_sub, nh_pub, nh_private, cam_idx_proj);
  node.start();

  ros::AsyncSpinner spinner_sub(2, &sub_queue);
  ros::AsyncSpinner spinner_pub(1, &pub_queue);
  spinner_sub.start();
  spinner_pub.start();
  ros::waitForShutdown();

}
