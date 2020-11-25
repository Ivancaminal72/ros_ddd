/*
Copyright (c) 2016, Helen Oleynikova, ETH Zurich, Switzerland
You can contact the author at <helen dot oleynikova at mavt dot ethz dot ch>

mandatory rights reserved.

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

#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <rosbag/bag.h>
#include <tf/tfMessage.h>
#include <boost/program_options.hpp>

#include "data_to_rosbag/kitti_parser.h"
#include "data_to_rosbag/kittiraw_ros_conversions.h"

namespace po = boost::program_options;

namespace adapt {

class KittiConverter {
 public:
  KittiConverter(const std::string& calibration_path,
                    const std::string& sequence_dir,
                    const std::string& output_path);

  void convertAll();
  bool convertEntry(uint64_t entry);
  void convertTf(uint64_t timestamp_ns, const Transformation& imu_pose);

 private:
  adapt::KittiParser parser_;

  rosbag::Bag bag_;

  std::string world_frame_id_;
  std::string imu_frame_id_;
  std::string cam_frame_id_prefix_;
  std::string velodyne_frame_id_;

  std::string pose_topic_;
  std::string transform_topic_;
  std::string pointcloud_topic_;
};

KittiConverter::KittiConverter(const std::string& calibration_path,
                                     const std::string& sequence_dir,
                                     const std::string& output_path)
    : parser_(calibration_path, sequence_dir, true),
      world_frame_id_("world"),
      cam_frame_id_prefix_("cam"),
      velodyne_frame_id_("velodyne"),
      pointcloud_topic_("velodyne_points") {
  // Load mandatory the timestamp maps and calibration parameters.
  parser_.loadCalibration();
  parser_.loadTimestampMaps();

  bag_.open(output_path, rosbag::bagmode::Write);
}

void KittiConverter::convertAll() {
  uint64_t entry = 0;
  while (convertEntry(entry)) {
    entry++;
  }
  std::cout << "Converted " << entry << " entries into a rosbag.\n";
}

bool KittiConverter::convertEntry(uint64_t entry) {
  ros::Time timestamp_ros;
  uint64_t timestamp_ns;

  // Convert poses + TF transforms.
  Transformation pose;
  if (parser_.getPoseAtEntry(entry, &timestamp_ns, &pose)) {
    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::TransformStamped transform_msg;

    timestampToRos(timestamp_ns, &timestamp_ros);
    pose_msg.header.frame_id = world_frame_id_;
    pose_msg.header.stamp = timestamp_ros;
    transform_msg.header.frame_id = world_frame_id_;
    transform_msg.header.stamp = timestamp_ros;

    poseToRos(pose, &pose_msg);
    transformToRos(pose, &transform_msg);

    bag_.write(pose_topic_, timestamp_ros, pose_msg);
    bag_.write(transform_topic_, timestamp_ros, transform_msg);

    convertTf(timestamp_ns, pose);
  } else {
    return false;
  }

  // Convert images.
  cv::Mat image;
  for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) {
    if (parser_.getImageAtEntry(entry, cam_id, &timestamp_ns, &image)) {
      timestampToRos(timestamp_ns, &timestamp_ros);

      sensor_msgs::Image image_msg;
      imageToRos(image, &image_msg);
      image_msg.header.stamp = timestamp_ros;
      image_msg.header.frame_id = getCameraFrameId(cam_id);

      // TODO(helenol): cache this.
      // Get the calibration info for this camera.
      CameraCalibration cam_calib;
      parser_.getCameraCalibration(cam_id, &cam_calib);
      sensor_msgs::CameraInfo cam_info;
      cam_calib.image_size << rgb_img.size().width, rgb_img.size().height;
      calibrationToRos(cam_id, cam_calib, &cam_info);
      cam_info.header = image_msg.header;

      bag_.write(getCameraFrameId(cam_id) + "/image_raw", timestamp_ros,
                 image_msg);
      bag_.write(getCameraFrameId(cam_id) + "/camera_info", timestamp_ros,
                 cam_info);
    }
  }

  // Convert pointclouds.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  if (parser_.getPointcloudAtEntry(entry, &timestamp_ns, &pointcloud)) {
    timestampToRos(timestamp_ns, &timestamp_ros);

    // This value is in MICROSECONDS, not nanoseconds.
    pointcloud.header.stamp = timestamp_ns / 1000;
    pointcloud.header.frame_id = velodyne_frame_id_;

    bag_.write(pointcloud_topic_, timestamp_ros, pointcloud);
  }

  return true;
}

void KittiConverter::convertTf(uint64_t timestamp_ns,
                                  const Transformation& imu_pose) {
  tf::tfMessage tf_msg;
  ros::Time timestamp_ros;
  timestampToRos(timestamp_ns, &timestamp_ros);

  // Create the full transform chain.
  Transformation T_imu_world = imu_pose;
  Transformation T_vel_imu = parser_.T_vel_imu();
  Transformation T_cam_imu;

  geometry_msgs::TransformStamped tf_imu_world, tf_vel_imu, tf_cam_imu;
  transformToRos(T_imu_world, &tf_imu_world);
  tf_imu_world.header.frame_id = world_frame_id_;
  tf_imu_world.child_frame_id = imu_frame_id_;
  tf_imu_world.header.stamp = timestamp_ros;
  transformToRos(T_vel_imu.inverse(), &tf_vel_imu);
  tf_vel_imu.header.frame_id = imu_frame_id_;
  tf_vel_imu.child_frame_id = velodyne_frame_id_;
  tf_vel_imu.header.stamp = timestamp_ros;

  // Put them into one tf_msg.
  tf_msg.transforms.push_back(tf_imu_world);
  tf_msg.transforms.push_back(tf_vel_imu);

  // Get mandatory of the camera transformations as well.
  for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) {
    T_cam_imu = parser_.T_camN_imu(cam_id);
    transformToRos(T_cam_imu.inverse(), &tf_cam_imu);
    tf_cam_imu.header.frame_id = imu_frame_id_;
    tf_cam_imu.child_frame_id = getCameraFrameId(cam_id);
    tf_cam_imu.header.stamp = timestamp_ros;
    tf_msg.transforms.push_back(tf_cam_imu);
  }

  bag_.write("/tf", timestamp_ros, tf_msg);
}

}  // namespace adapt

int main(int argc, char** argv) {

  //Declare variables
  std::string sequence_dir;
  std::string calibration_path;
  std::string output_path;

  //Parse arguments
  po::options_description mandatory("Mandatory args");
  mandatory.add_options()
    ("seq,s", po::value<std::string>(&sequence_dir), "sequence directory (without trailing slash)");
  
  po::positional_options_description p;
  p.add("seq", -1);

  po::options_description optional("Optional args");
  optional.add_options()
    ("help,h", "produce a help message")
    ("calib,c", po::value<std::string>(&calibration_path), "default: seq/calib.txt")
    ("out,o", po::value<std::string>(&output_path), "output rosbag path");

  po::variables_map vm;
  po::options_description options;
  options.add(mandatory).add(optional);
  po::store(po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);
  po::notify(vm);

  if (!vm.count("calib")) calibration_path = sequence_dir+"/calib.txt";

  if (vm.count("help")) 
  {
    std::cout << "Usage: rosrun data_to_rosbag kitti_rosbag_converter [$seq] {args}\n";
    std::cout << options << std::endl;
    return 1;
  }
  // else
  // {
  //   std::cout<<"sequence_dir"<<" -> "<<sequence_dir<<std::endl;
  //   std::cout<<"calibration_path"<<" -> "<<calibration_path<<std::endl;
  //   std::cout<<"output_path"<<" -> "<<output_path<<std::endl;
  // }
  

  adapt::KittiConverter converter(calibration_path, sequence_dir, output_path);
  converter.convertAll();
}
