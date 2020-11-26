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

#include "data_to_rosbag/kitti_parser.h"
#include "data_to_rosbag/kittiraw_ros_conversions.h"

namespace po = boost::program_options;

namespace adapt {

class PcdToPng {
 public:
  PcdToPng(const ros::NodeHandle& nh,
           const ros::NodeHandle& nh_private, const int cam_idx_proj);

  // Creates a timer to automatically publish entries in 'realtime' versus
  // the original data,
  void start();
  void rePublishClock(const rosgraph_msgs::Clock &msg);
  void rePublishLidar(const pcl::PointCloud<pcl::PointXYZI> &msg);
  void rePublishColor(const sensor_msgs::ImageConstPtr& image_msg,
                      const sensor_msgs::CameraInfoConstPtr& info_msg);

  bool projectPointcloud(pcl::PointCloud<pcl::PointXYZI>& pcd,
                          cv::Mat& depth_img,
                          cv::Mat& intensity_img);

 private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers for the topics.
  ros::Subscriber clock_sub_;
  ros::Subscriber pointcloud_sub_;

  image_transport::ImageTransport image_transport_private_;
  image_transport::CameraSubscriber image_rgb_sub_;
  
  
  // Publishers for the topics.
  ros::Publisher clock_pub_;
  ros::Publisher pointcloud_pub_;
  ros::Publisher pose_pub_;

  image_transport::ImageTransport image_transport_;
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
  int cam_idx_proj_;
  std::map<std::string, int> width_;
  std::map<std::string, int> height_;
  std::map<std::string, Eigen::Matrix<double, 3, 4>, std::less<int>, 
         Eigen::aligned_allocator<std::pair<const std::string, Eigen::Matrix<double, 3, 4>> > > Proj_;
  Eigen::Affine3d T_cam0_lidar_;
  
  //Magement
  bool is_tf_ready_;
  bool is_cam_ready_;
  bool is_first_pcd_republished_;
};

PcdToPng::PcdToPng(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private,
                   const int cam_idx_proj)
    : nh_(nh),
      nh_private_(nh_private),
      image_transport_(nh_),
      image_transport_private_(nh_private),
      world_frame_id_("world"),
      imu_frame_id_("imu"),
      cam_frame_id_prefix_("cam"),
      lidar_frame_id_("lidar"),
      current_entry_(0),
      current_timestamp_ns_(0),
      cam_idx_proj_(cam_idx_proj),
      is_tf_ready_(false),
      is_first_pcd_republished_(false){
 
  // Subcribe to the node_live / rosbag topics
  clock_sub_ = nh_private_.subscribe("clock_bag",1,&PcdToPng::rePublishClock,this);
  pointcloud_sub_ = nh_private_.subscribe("lidar_bag",1,&PcdToPng::rePublishLidar,this);
  image_rgb_sub_ = image_transport_private_.subscribeCamera(getCameraFrameId(cam_idx_proj_), 1, &PcdToPng::rePublishColor, this);
   
  // Advertise all the publishing topics for ROS live streaming.
  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1, false);
  pointcloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "lidar", 10, false);
  image_rgb_pub_ = image_transport_.advertiseCamera(getCameraFrameId(cam_idx_proj_),1);
  image_infrared_pub_ = image_transport_.advertiseCamera(getCameraFrameId(cam_idx_proj_)+"_infrared",1);
  image_depth_pub_ = image_transport_.advertiseCamera(getCameraFrameId(cam_idx_proj_)+"_depth",1);
  
  //     nh_.advertise<geometry_msgs::PoseStamped>("pose_imu", 10, false);
  // transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(
  //     "transform_imu", 10, false);
}

void PcdToPng::start() {
  
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener transform_listener(tf_buffer);
  
  while (nh_private_.ok())
  {
    geometry_msgs::TransformStamped Ts_cam0_lidar;
    try
    {
      Ts_cam0_lidar = tf_buffer.lookupTransform(lidar_frame_id_, 
              getCameraFrameId(0)+"_bag", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    
    T_cam0_lidar_= tf2::transformToEigen(Ts_cam0_lidar);
    is_tf_ready_=true;
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
  
  buffer_rgb_.push(*image_msg_ptr);
  buffer_rgb_info_.push(*info_msg_ptr);
  
  if(!is_first_pcd_republished_) return;
  else if (!is_cam_ready_)
  {
    width_[info_msg_ptr->header.frame_id] = (int) info_msg_ptr->width;
    height_[info_msg_ptr->header.frame_id] = (int) info_msg_ptr->height;
    
    for (size_t i = 0; i < 3; ++i) {
      for (size_t j = 0; j < 4; ++j) {
        Proj_[info_msg_ptr->header.frame_id](i, j) = info_msg_ptr->P[4 * i + j];
      }
    }

    info_msg_ptr->P;
    is_cam_ready_=true;
  }
  else
  {
    sensor_msgs::Image image_msg = buffer_rgb_.front();
    image_rgb_pub_.publish(image_msg, buffer_rgb_info_.front(), image_msg.header.stamp);
    buffer_rgb_info_.pop();
    buffer_rgb_.pop();
  }

  // cv_bridge::CvImagePtr image_cv_ptr = rosToImagePtr(image_msg, sensor_msgs::image_encodings::BGR8);
}

void PcdToPng::rePublishLidar(const pcl::PointCloud<pcl::PointXYZI> &msg) {

  buffer_pcd_.push(msg);
  
  if(!is_tf_ready_) return;
  else if (width_.find(getCameraFrameId(cam_idx_proj_)+"_bag") != width_.end() && 
            height_.find(getCameraFrameId(cam_idx_proj_)+"_bag") != height_.end() )
  {
    pcl::PointCloud<pcl::PointXYZI> pointcloud = buffer_pcd_.front();
    cv::Mat depth_img, intensity_img;
    if (cam_idx_proj_ >= 0)
      if (projectPointcloud(pointcloud, depth_img, intensity_img))
        buffer_pcd_.pop();
      else 
        ROS_ERROR("Error projecting points");
    
    //Publish
    if(!is_first_pcd_republished_) is_first_pcd_republished_=true;
  }

}

bool PcdToPng::projectPointcloud(pcl::PointCloud<pcl::PointXYZI>& pcd,
                                 cv::Mat& depth_img,
                                 cv::Mat& intensity_img)
{
  Eigen::Array3Xd ddd_pts_p;
  Eigen::RowVectorXd depth_pts;
  Eigen::RowVectorXd intensity_pts;
  Eigen::Matrix3Xd ddd_pts;

  Proj_[info_msg_ptr->header.frame_id]

  // Start projection
  *ddd_pts = T_cam0_lidar_ * ddd_pts->colwise().homogeneous();
  ddd_pts_p = (camera_calibrations_[cam_idx_proj_]
    .projection_mat * ddd_pts->colwise().homogeneous()).array();
  ddd_pts_p.rowwise() /= ddd_pts_p.row(2);
  depth_pts = ddd_pts->row(2);
  Eigen::Array<double,1,Eigen::Dynamic> depth_pts_arr = depth_pts.array();
  depth_pts = depth_pts_arr.round().matrix();
  *depth_img = cv::Mat::zeros(width, height, CV_16UC1);
  *intensity_img = cv::Mat::zeros(width, height, CV_16UC1);
  

  uint inside=0, outside=0, valid=0;
  int x, y;
  //iterate depth values
  for(int i=0; i<depth_pts.cols(); i++)
  {
      x = round(ddd_pts_p(0,i));
      y = round(ddd_pts_p(1,i));

      //consider only points projected within camera sensor
      if(x<width && x>=0 && y<height && y>=0)
      {
          inside +=1;
          //only positive depth
          if(depth_pts(0,i)>0)
          {
              valid+=1;
              ushort d = (ushort) depth_pts(0,i);
              
              //pixel need update
              if(depth_img->at<ushort>(y,x) == 0 
                || depth_img->at<ushort>(y,x) > d)
              {
                
                //exceed established limit (save max)
                if(d>=pow(2,16)) depth_img->at<ushort>(y,x) 
                  = pow(2,16)-1;
                
                //inside limit (save sensed depth)
                else if(depth_img->at<ushort>(y,x) == 0) 
                  depth_img->at<ushort>(y,x) = d;
                
                //pixel with value (save the smallest depth)
                else depth_img->at<ushort>(y,x) = d;

                //Save 16bit intensity
                intensity_img->at<ushort>(y,x) 
                  = trunc((*intensity_pts)(0,i) * pow(2,16));
              }
          }
      }
      else outside+=1;
  }
  return true;
}

}  // namespace adapt

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcd_to_png");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ros::CallbackQueue pub_queue;
  ros::CallbackQueue sub_queue;
  nh.setCallbackQueue(&pub_queue);
  nh_private.setCallbackQueue(&sub_queue);


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
  adapt::PcdToPng node(nh, nh_private, cam_idx_proj);
  node.startPublishing(50.0);

  ros::AsyncSpinner spinner_sub(3, &sub_queue);
  ros::AsyncSpinner spinner_pub(1, &pub_queue);
  spinner_sub.start();
  spinner_pub.start();
  ros::waitForShutdown();

}
