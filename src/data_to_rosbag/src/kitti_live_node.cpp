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
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <boost/program_options.hpp>

#include "data_to_rosbag/kitti_parser.h"
#include "data_to_rosbag/kittiraw_ros_conversions.h"

namespace po = boost::program_options;

namespace adapt {

class KittiToPng {
 public:
  KittiToPng(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                const std::string& sequence_dir);

  // Creates a timer to automatically publish entries in 'realtime' versus
  // the original data,
  void startPublishing(double rate_hz);

  bool publishEntry(uint64_t entry, uint64_t current_timestamp_ns_);

  void publishTfGroundTruth(uint64_t timestamp_ns, const Transformation& imu_pose);

  void publishTf(uint64_t timestamp_ns);

  void publishClock(uint64_t timestamp_ns);

 private:
  void timerCallback(const ros::WallTimerEvent& event);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers for the topics.
  ros::Publisher clock_pub_;
  ros::Publisher pointcloud_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher transform_pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  image_transport::ImageTransport image_transport_;
  std::vector<image_transport::CameraPublisher> image_pubs_;

  // Decides the rate of publishing (TF is published at this rate).
  // Call startPublishing() to turn this on.
  ros::WallTimer publish_timer_;

  adapt::KittiParser parser_;

  std::string world_frame_id_;
  std::string imu_frame_id_;
  std::string cam_frame_id_prefix_;
  std::string lidar_frame_id_;

  uint64_t current_entry_;
  uint64_t publish_dt_ns_;
  uint64_t current_timestamp_ns_;
  uint64_t next_entry_timestamp_ns_;

  int cam_idx_proj_;
  std::ifstream fin_time_;
};

KittiToPng::KittiToPng(const ros::NodeHandle& nh,
                             const ros::NodeHandle& nh_private,
                             const std::string& sequence_dir)
    : nh_(nh),
      nh_private_(nh_private),
      image_transport_(nh_),
      parser_(sequence_dir, true),
      world_frame_id_("bag/world"),
      imu_frame_id_("bag/imu"),
      cam_frame_id_prefix_("bag/cam"),
      lidar_frame_id_("bag/lidar"),
      current_entry_(0),
      publish_dt_ns_(0),
      current_timestamp_ns_(0) {
  
  ROS_INFO("KittiToPng class initialization: OK");

  // Load all the timestamp maps and calibration parameters.
  parser_.loadCalibration();
  ROS_INFO("KittiToPng loadCalibration: OK");
  
  fin_time_.open(sequence_dir + "/" + KittiParser::kTimestampFile, std::ios::in);
  if (!fin_time_) {
    ROS_ERROR("KittiToPng failed opening timestamps file");
    exit(1);
  }

  // Advertise all the publishing topics for ROS live streaming.
  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("bag/clock", 1, false);
  pointcloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      lidar_frame_id_, 10, false);
  // pose_pub_ =
  //     nh_.advertise<geometry_msgs::PoseStamped>("pose_imu", 10, false);
  // transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(
  //     "transform_imu", 10, false);

  for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) {
    image_pubs_.push_back(
        image_transport_.advertiseCamera(getSensorFrameId(cam_frame_id_prefix_, cam_id), 1, false));
  }
  ROS_INFO("KittiToPng advertise topics: OK");

  sleep(1); //Delay to allow subscribers to stablish connections
}


void KittiToPng::startPublishing(double rate_hz) {
  double publish_dt_sec = 1.0 / rate_hz;
  publish_dt_ns_ = static_cast<uint64_t>(publish_dt_sec * 1e9);
  // std::cout << "Publish dt ns: " << publish_dt_ns_ << std::endl;
  publish_timer_ = nh_.createWallTimer(ros::WallDuration(publish_dt_sec),
                                       &KittiToPng::timerCallback, this);
}

void KittiToPng::timerCallback(const ros::WallTimerEvent& event) {
  Transformation tf_interpolated;

  if (current_entry_ == 0) 
  {
    std::cout << "Current entry: " << current_entry_ << std::endl;
    if(!parser_.loadNextTimestamp(fin_time_, current_timestamp_ns_))
            publish_timer_.stop();
    publishTf(current_timestamp_ns_);
    // This is the first time this is running! Initialize the current timestamp
    // and publish this entry.
    if (!publishEntry(current_entry_, current_timestamp_ns_))
      publish_timer_.stop();
    
    publishClock(current_timestamp_ns_);
    // if (parser_.interpolatePoseAtTimestamp(current_timestamp_ns_,
    //                                        &tf_interpolated)) {
    //   publishTfGroundTruth(current_timestamp_ns_, tf_interpolated);
    // }
    current_entry_++;
    if(!parser_.loadNextTimestamp(fin_time_, next_entry_timestamp_ns_))
            publish_timer_.stop();
    return;
  }

  // std::cout << "Publish dt ns: " << publish_dt_ns_ << std::endl;
  current_timestamp_ns_ += publish_dt_ns_;
  // std::cout << "Updated timestmap: " << current_timestamp_ns_ << std::endl;
  publishClock(current_timestamp_ns_);
  // if (parser_.interpolatePoseAtTimestamp(current_timestamp_ns_,
  //                                        &tf_interpolated)) {
  //   publishTfGroundTruth(current_timestamp_ns_, tf_interpolated);
  //   // std::cout << "Transform: " << tf_interpolated << std::endl;
  // } else {
  //   std::cout << "Failed to interpolate!\n";
  // }

  // std::cout << "Current entry's timestamp: "
  //           << next_entry_timestamp_ns_ << std::endl;
  if (next_entry_timestamp_ns_ <= current_timestamp_ns_) 
  {
    std::cout << "Current entry: " << current_entry_ << std::endl;
    if (!publishEntry(current_entry_, next_entry_timestamp_ns_)) 
    {
      publish_timer_.stop();
      return;
    }
    current_entry_++;
    if(!parser_.loadNextTimestamp(fin_time_, next_entry_timestamp_ns_))
    {
      publish_timer_.stop();
      // sleep(1); //Send last queued messages...
      // exit(0);
    }
            
  }
}

void KittiToPng::publishClock(uint64_t timestamp_ns) {
  ros::Time timestamp_ros;
  timestampToRos(timestamp_ns, &timestamp_ros);
  rosgraph_msgs::Clock clock_time;
  clock_time.clock = timestamp_ros;
  clock_pub_.publish(clock_time);
}

bool KittiToPng::publishEntry(uint64_t entry, uint64_t timestamp_ns) {
  ros::Time timestamp_ros;
  rosgraph_msgs::Clock clock_time;
  timestampToRos(timestamp_ns, &timestamp_ros);

  // Publish poses + TF transforms + clock.
  // Transformation pose;
  // if (parser_.getPoseAtEntry(entry, &timestamp_ns, &pose)) {
  //   geometry_msgs::PoseStamped pose_msg;
  //   geometry_msgs::TransformStamped transform_msg;

  //   timestampToRos(timestamp_ns, &timestamp_ros);
  //   pose_msg.header.frame_id = world_frame_id_;
  //   pose_msg.header.stamp = timestamp_ros;
  //   transform_msg.header.frame_id = world_frame_id_;
  //   transform_msg.header.stamp = timestamp_ros;

  //   poseToRos(pose, &pose_msg);
  //   transformToRos(pose, &transform_msg);

  //   pose_pub_.publish(pose_msg);
  //   transform_pub_.publish(transform_msg);

  //   // publishClock(timestamp_ns);
  //   // publishTfGroundTruth(timestamp_ns, pose);
  // } else {
  //   return false;
  // }


  // Read images.
  cv::Mat rgb_img;
  // for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) { // WARN subscriber may mix camera info mesages...
  for (size_t cam_id = 2; cam_id < 3; ++cam_id) {
    if (parser_.getImageAtEntry(entry, cam_id, &rgb_img)) {
      sensor_msgs::Image image_msg;
      imageToRos(rgb_img, &image_msg);

      // TODO(helenol): cache this.
      // Get the calibration info for this camera.
      CameraCalibration cam_calib;
      parser_.getCameraCalibration(cam_id, &cam_calib);
      sensor_msgs::CameraInfo cam_info;
      cam_calib.image_size << rgb_img.size().width, rgb_img.size().height;
      std::string cam_frame_id = getSensorFrameId(cam_frame_id_prefix_, cam_id);
      calibrationToRos(cam_frame_id, cam_calib, &cam_info);

      image_msg.header.stamp = timestamp_ros;
      image_msg.header.frame_id = cam_frame_id;
      cam_info.header = image_msg.header;

      image_pubs_[cam_id].publish(image_msg, cam_info, timestamp_ros);
    }
  }
  
  // Publish pointclouds.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  // cv::Mat depth_img, intensity_img;
  // Eigen::RowVectorXd intensity_pts;
  // Eigen::Matrix3Xd ddd_pts;

  if (parser_.getPointcloudAtEntry(entry, &pointcloud)) {
  // if (parser_.getPointcloudAtEntry(entry, &pointcloud, &ddd_pts, &intensity_pts)) {
    // //Project points and obtain depth/intensity images
    // if (cam_idx_proj_ >= 0)
    //   if (parser_.projectPointcloud(cam_idx_proj_, &ddd_pts, &intensity_pts, &depth_img, &intensity_img))
    
    
    // // This value should be in MICROSECONDS, not nanoseconds.
    // pointcloud.header.stamp = timestamp_ns / 1000;
    pointcloud.header.stamp = timestamp_ns;
    pointcloud.header.frame_id = lidar_frame_id_;
    pointcloud_pub_.publish(pointcloud);
  }

  return true;
}

void KittiToPng::publishTf(uint64_t timestamp_ns) {
  ros::Time timestamp_ros;
  timestampToRos(timestamp_ns, &timestamp_ros);
  Transformation T_cam_lidar;

  tf::Transform tf_cam_lidar;
  static tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;
  // geometry_msgs::TransformStamped Ts_cam0_lidar = parser_.Ts_cam0_lidar();
  geometry_msgs::TransformStamped Ts_cam_lidar = parser_.Ts_cam0_lidar();
  
  // Ts_cam0_lidar.header.stamp = timestamp_ros;
  // Ts_cam0_lidar.header.frame_id = lidar_frame_id_;
  // Ts_cam0_lidar.child_frame_id = "bag/"+getCameraFrameId(0);
  // Ts_cam0_lidar.transform = Ts_cam0_lidar.transform;
  // static_tf_broadcaster.sendTransform(Ts_cam0_lidar);

  for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) 
  {
  Ts_cam_lidar.header.stamp = timestamp_ros;
  Ts_cam_lidar.header.frame_id = lidar_frame_id_;
  Ts_cam_lidar.child_frame_id = getSensorFrameId(cam_frame_id_prefix_, cam_id);
  static_tf_broadcaster.sendTransform(Ts_cam_lidar);
  }

}

// void KittiToPng::publishTfGroundTruth(uint64_t timestamp_ns,
//                               const Transformation& imu_pose) {
//   ros::Time timestamp_ros;
//   timestampToRos(timestamp_ns, &timestamp_ros);
//   Transformation T_imu_world = imu_pose;
//   Transformation T_lidar_imu = parser_.T_lidar_imu();
//   Transformation T_cam_imu;

//   tf::Transform tf_imu_world, tf_cam_imu, tf_lidar_imu;

//   transformToTf(T_imu_world, &tf_imu_world);
//   transformToTf(T_lidar_imu.inverse(), &tf_lidar_imu);

//   tf_broadcaster_.sendTransform(tf::StampedTransform(
//       tf_imu_world, timestamp_ros, world_frame_id_, imu_frame_id_));
//   tf_broadcaster_.sendTransform(tf::StampedTransform(
//       tf_lidar_imu, timestamp_ros, imu_frame_id_, lidar_frame_id_));

//   for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) {
//     T_cam_imu = parser_.T_camN_imu(cam_id);
//     transformToTf(T_cam_imu.inverse(), &tf_cam_imu);
//     tf_broadcaster_.sendTransform(tf::StampedTransform(
//         tf_cam_imu, timestamp_ros, imu_frame_id_, getCameraFrameId(cam_id)));
//   }
// }

}  // namespace adapt

int main(int argc, char** argv) {
  ros::init(argc, argv, "kitti_live_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  //Declare variables
  std::string sequence_dir;
  std::string calibration_file;
  std::string timestamp_file;

  //Parse arguments
  po::options_description mandatory_opts("Mandatory args");
  mandatory_opts.add_options()
    ("seq,s", po::value<std::string>(&sequence_dir), "Sequence directory (without trailing slash)");
  
  po::positional_options_description positional_opts;
  positional_opts.add("seq", -1);

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
    std::cout << "Usage: rosrun data_to_rosbag kitti_live_node {args}\n";
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
  ROS_INFO("Parsing of arguments: OK");

  //Ready to start
  adapt::KittiToPng node(nh, nh_private, sequence_dir);
  ROS_INFO("KittiToPng class constructor: OK");
  node.startPublishing(50.0);

  ros::spin();
}
