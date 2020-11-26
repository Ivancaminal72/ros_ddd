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

#include <iostream>
#include <fstream>
#include <iomanip>

#include <opencv2/highgui/highgui.hpp>

#include "data_to_rosbag/kitti_parser.h"

namespace adapt {

const std::string KittiParser::kVelToCamCalibrationFilename =
    "calib_velo_to_cam.txt";
const std::string KittiParser::kCamToCamCalibrationFilename =
    "calib_cam_to_cam.txt";
const std::string KittiParser::kImuToVelCalibrationFilename =
    "calib_imu_to_velo.txt";

const std::string KittiParser::kVelodyneFolder = "velodyne";
const std::string KittiParser::kCameraFolder = "image_";
const std::string KittiParser::kCalibrationFile = "calib.txt";
const std::string KittiParser::kTimestampFile = "times.txt";
const size_t KittiParser::kMaxNumberOfScanPoints = 1e6;  // From Readme for raw files.

KittiParser::KittiParser(const std::string& sequence_dir, bool rectified)
    : sequence_dir_(sequence_dir),
      rectified_(rectified),
      initial_pose_set_(false) 
      {
        camera_calibrations_.resize(4);
      }

KittiParser::KittiParser()
    : rectified_(true),
      initial_pose_set_(false) 
      {
        camera_calibrations_.resize(4);
      }

bool KittiParser::loadCalibration() {
  std::string line_str, item_str, filename = sequence_dir_ + "/" + kCalibrationFile;
  std::istringstream line_istr;
  std::ifstream fin_calib(filename);
  
  while(true)
    {
      std::getline(fin_calib, line_str);
      if(fin_calib.eof()) break;
      line_istr.str(line_str);
      line_istr.clear();
      line_istr>>item_str;
      if(item_str[0]=='P')
      {
        int cam_idx = (int) item_str[1] - '0';
          if(cam_idx >= 0 && cam_idx <4)
          {
              for(int i=0; line_istr.good(); i++)
              {
                  line_istr>>item_str;
                  if(i==0 || i==2 || i==5 || i==6) 
                    camera_calibrations_[cam_idx]
                      .projection_mat(i/4, i%4) = stod(item_str);
                  else 
                    camera_calibrations_[cam_idx]
                      .projection_mat(i/4, i%4) = stod(item_str);
              }
          }
          else continue;

      }
      else
      {
          for(int i=0; line_istr.good(); i++)
          {
              line_istr>>item_str;
              T_cam0_lidar_(i/4, i%4) = stod(item_str);
          }
      }
    }
    fin_calib.close();

  return true;
}

bool KittiParser::parseVectorOfDoubles(const std::string& input,
                                       std::vector<double>* output) const {
  output->clear();
  // Parse the line as a stringstream for space-delimeted doubles.
  std::stringstream line_stream(input);
  if (line_stream.eof()) {
    return false;
  }

  while (!line_stream.eof()) {
    std::string element;
    std::getline(line_stream, element, ' ');
    if (element.empty()) {
      continue;
    }
    try {
      output->emplace_back(std::stod(element));
    } catch (const std::exception& exception) {
      std::cout << "Could not parse number in import file.\n";
      return false;
    }
  }
  return true;
}

bool KittiParser::loadNextTimestamp(std::ifstream& fin_time, uint64_t& timestamp) {
  std::string stamp_str;
  if (std::getline(fin_time, stamp_str)) 
  {
    // Seconds to nanoseconds
    timestamp = static_cast<uint64_t>(std::stod(stamp_str) * 1e9);
    return true;
  }
  else 
    return false;
}

bool KittiParser::loadTimestampMaps() {
  // Load timestamps for poses.
  std::ifstream fin_time(sequence_dir_ + "/" + kTimestampFile, std::ios::in);
  if (!fin_time) {
    return false;
  }

  timestamps_.clear();
  std::string stamp_str;
  while (std::getline(fin_time, stamp_str)) {
    // Seconds to nanoseconds
    uint64_t timestamp = static_cast<uint64_t>(std::stod(stamp_str) * 1e9);
    timestamps_.push_back(timestamp);
  }

  std::cout << "Timestamps: " << std::endl
            << timestamps_.front() << " " << timestamps_.back()
            << std::endl;

  // std::cout << "Timestmap map for pose:\n";
  // for (size_t i = 0; i < timestamps_.size(); i++) 
  //   std::cout << i << " " << timestamps_[i] << std::endl;
}

bool KittiParser::getCameraCalibration(uint64_t cam_id,
                                       CameraCalibration* cam) const {
  if (cam_id >= camera_calibrations_.size()) {
    return false;
  }
  *cam = camera_calibrations_[cam_id];
  return true;
}

// bool KittiParser::getPoseAtEntry(uint64_t entry, uint64_t* timestamp,
//                                  Transformation* pose) {
//   std::string filename = sequence_dir_ + "/" + kDataFolder +
//                          "/" + getFilenameForEntry(entry) + ".txt";

//   std::ifstream fin_time(filename, std::ios::in);
//   if (!fin_time) {
//     return false;
//   }
//   if (timestamps_.size() <= entry) {
//     return false;
//   }
//   *timestamp = timestamps_[entry];

//   std::string line_str;
//   std::vector<double> parsed_doubles;
//   while (std::getline(fin_time, line_str)) {
//     if (parseVectorOfDoubles(line_str, &parsed_doubles)) {
//       if (convertGpsToPose(parsed_doubles, pose)) {
//         return true;
//       }
//     }
//   }
//   return false;
// }

uint64_t KittiParser::getTimestampAtEntry(uint64_t entry) {
  if (timestamps_.size() <= entry) {
    return 0;
  }
  return timestamps_[entry];
}

bool KittiParser::getPointcloudAtEntry( uint64_t entry,
    pcl::PointCloud<pcl::PointXYZI>* ptcloud) {

  // Load the actual pointcloud.
  ptcloud->clear();
  ptcloud->reserve(kMaxNumberOfScanPoints);

  std::string filename = sequence_dir_ + "/" + kVelodyneFolder 
                         + "/" + getFilenameForEntry(entry) + 
                         ".bin";

  std::ifstream input(filename, std::ios::in | std::ios::binary);
  if (!input) {
    std::cout << "Could not open pointcloud file.\n";
    return false;
  }

  std::streampos begin, end;
  int numPts;
  begin = input.tellg();
  input.seekg (0, input.end);
  end = input.tellg();
  input.seekg(0, input.beg);
  numPts = (end-begin)/(4*sizeof(float)); //calculate number of points
  for (int i = 0; input.good() && !input.eof(); i++) {
    pcl::PointXYZI point;
    input.read((char*)&point.x, 3 * sizeof(float));
    input.read((char*)&point.intensity, sizeof(float));
    ptcloud->push_back(point);
  }
  input.close();
  return true;
}

bool KittiParser::getPointcloudAtEntry( uint64_t entry,
    pcl::PointCloud<pcl::PointXYZI>* ptcloud,
    Eigen::Matrix3Xd* ddd_pts,
    Eigen::RowVectorXd* intensity_pts) {

  // Load the actual pointcloud.
  ptcloud->clear();
  ptcloud->reserve(kMaxNumberOfScanPoints);

  std::string filename = sequence_dir_ + "/" + kVelodyneFolder 
                         + "/" + getFilenameForEntry(entry) + 
                         ".bin";

  std::ifstream input(filename, std::ios::in | std::ios::binary);
  if (!input) {
    std::cout << "Could not open pointcloud file.\n";
    return false;
  }

  std::streampos begin, end;
  int numPts;
  begin = input.tellg();
  input.seekg (0, input.end);
  end = input.tellg();
  input.seekg(0, input.beg);
  numPts = (end-begin)/(4*sizeof(float)); //calculate number of points
  *ddd_pts = Eigen::Matrix3Xd::Zero(3, numPts);
  *intensity_pts = Eigen::RowVectorXd::Zero(1, numPts);
  for (int i = 0; input.good() && !input.eof(); i++) {
    pcl::PointXYZI point;
    input.read((char*)&point.x, 3 * sizeof(float));
    input.read((char*)&point.intensity, sizeof(float));
    ptcloud->push_back(point);
    ddd_pts->col(i) << (double) point.x, (double) point.y, (double) point.z;
    intensity_pts->col(i) << (double) point.intensity;
  }
  input.close();
  return true;
}

bool KittiParser::projectPointcloud(int cam_idx_proj,
                                    Eigen::Matrix3Xd* ddd_pts,
                                    Eigen::RowVectorXd* intensity_pts, 
                                    cv::Mat* depth_img, cv::Mat* intensity_img)
{
  Eigen::Array3Xd ddd_pts_p;
  Eigen::RowVectorXd depth_pts;
  CameraCalibration cam_calib;
  getCameraCalibration(cam_idx_proj, &cam_calib);
  int width = (int) cam_calib.image_size.x();
  int height = (int) cam_calib.image_size.y();
  Eigen::Matrix<double, 3, 4> T_cam0_lidar_; //todeclare


  // Start projection
  *ddd_pts = T_cam0_lidar_ * ddd_pts->colwise().homogeneous();
  ddd_pts_p = (camera_calibrations_[cam_idx_proj]
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
}

bool KittiParser::getImageAtEntry(uint64_t entry, uint64_t cam_id,
                                  cv::Mat* image) {

  std::string filename = sequence_dir_ + "/" + getFolderNameForCamera(cam_id) +
                         "/" + getFilenameForEntry(entry) +
                         ".png";

  *image = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);

  if (!image->data) {
    std::cout << "Could not load image data.\n";
    return false;
  }
  return true;
}

// From the MATLAB raw data dev kit.
bool KittiParser::convertGpsToPose(const std::vector<double>& oxts,
                                   Transformation* pose) {
  if (oxts.size() < 6) {
    return false;
  }

  double lat = oxts[0];
  double lon = oxts[1];
  double alt = oxts[2];

  double roll = oxts[3];
  double pitch = oxts[4];
  double yaw = oxts[5];

  // Position.
  if (!initial_pose_set_) {
    mercator_scale_ = latToScale(lat);
  }
  Eigen::Vector2d mercator;
  latlonToMercator(lat, lon, mercator_scale_, &mercator);
  Eigen::Vector3d position(mercator.x(), mercator.y(), alt);

  // Rotation.
  const Eigen::AngleAxisd axis_roll(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd axis_pitch(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd axis_yaw(yaw, Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond rotation = axis_yaw * axis_pitch * axis_roll;

  Transformation transform(position, rotation);

  // Undo the initial transformation, if one is set.
  // If not, set it.
  // The transformation only undoes translation and yaw, not roll and pitch
  // (as these are observable from the gravity vector).
  if (!initial_pose_set_) {
    T_initial_pose_.getPosition() = transform.getPosition();
    T_initial_pose_.getRotation() = Rotation(axis_yaw);
    initial_pose_set_ = true;
  }
  // Get back to local coordinates.
  *pose = T_initial_pose_.inverse() * transform;

  return true;
}

// From the MATLAB raw data dev kit.
double KittiParser::latToScale(double lat) const {
  return cos(lat * M_PI / 180.0);
}

// From the MATLAB raw data dev kit.
void KittiParser::latlonToMercator(double lat, double lon, double scale,
                                   Eigen::Vector2d* mercator) const {
  double er = 6378137;
  mercator->x() = scale * lon * M_PI * er / 180.0;
  mercator->y() = scale * er * log(tan((90.0 + lat) * M_PI / 360.0));
}

std::string KittiParser::getFolderNameForCamera(int cam_number) const {
  char buffer[20];
  sprintf(buffer, "%s%d", kCameraFolder.c_str(), cam_number);
  return std::string(buffer);
}

std::string KittiParser::getFilenameForEntry(uint64_t entry) const {
  char buffer[20];
  sprintf(buffer, "%06lu", entry);
  return std::string(buffer);
}

// Transformation KittiParser::T_camN_vel(int cam_number) const {
//   return camera_calibrations_[cam_number].T_cam0_cam * T_cam0_vel_;
// }

// Transformation KittiParser::T_camN_imu(int cam_number) const {
//   return T_camN_vel(cam_number) * T_lidar_imu_;
// }

Transformation KittiParser::T_cam0_vel() const { return T_cam0_vel_; }
geometry_msgs::TransformStamped KittiParser::Ts_cam0_lidar() const { return tf2::eigenToTransform(T_cam0_lidar_); }

Transformation KittiParser::T_lidar_imu() const { return T_lidar_imu_; }

// bool KittiParser::interpolatePoseAtTimestamp(uint64_t timestamp,
//                                              Transformation* pose) {
//   // Look up the closest 2 timestamps to this.
//   size_t left_index = timestamps_.size();
//   for (size_t i = 0; i < timestamps_.size(); i++) {
//     if (timestamps_[i] > timestamp) {
//       if (i == 0) {
//         // Then we can't interpolate the pose since we're outside the range.
//         return false;
//       }
//       left_index = i - 1;
//       break;
//     }
//   }
//   if (left_index >= timestamps_.size()) {
//     return false;
//   }
//   // Make sure we don't go over the size
//   // if (left_index == timestamps_.size() - 1) {
//   //  left_index--;
//   //}

//   // Figure out what 't' should be, where t = 0 means 100% left boundary,
//   // and t = 1 means 100% right boundary.
//   double t = (timestamp - timestamps_[left_index]) /
//              static_cast<double>(timestamps_[left_index + 1] -
//                                  timestamps_[left_index]);

//   std::cout << "Timestamp: " << timestamp
//             << " timestamp left: " << timestamps_[left_index]
//             << " timestamp right: " << timestamps_[left_index + 1]
//             << " t: " << t << std::endl;

//   // Load the two transformations.
//   uint64_t timestamp_left, timestamp_right;
//   Transformation transform_left, transform_right;
//   if (!getPoseAtEntry(left_index, &timestamp_left, &transform_left) ||
//       !getPoseAtEntry(left_index + 1, &timestamp_right, &transform_right)) {
//     // For some reason couldn't load the poses.
//     return false;
//   }

//   // Interpolate between them.
//   *pose = interpolateTransformations(transform_left, transform_right, t);
//   return true;
// }

size_t KittiParser::getNumCameras() const {
  return camera_calibrations_.size();
}

}  // namespace adapt
