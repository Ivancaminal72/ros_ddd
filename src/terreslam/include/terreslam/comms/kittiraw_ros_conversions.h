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

#ifndef TERRESLAM_KITTI_ROS_CONVERSIONS_H_
#define TERRESLAM_KITTI_ROS_CONVERSIONS_H_

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>

#include "terreslam/comms/kittiraw_common.h"

namespace adapt {

void calibrationToRos(std::string frame_id, const CameraCalibration& cam,
					  sensor_msgs::CameraInfo* cam_msg);
void stereoCalibrationToRos(const CameraCalibration& left_cam,
							const CameraCalibration& right_cam,
							sensor_msgs::CameraInfo* left_cam_msg,
							sensor_msgs::CameraInfo* right_cam_msg);
void imageToRos(const cv::Mat& image, sensor_msgs::Image* image_msg);
cv_bridge::CvImagePtr rosToImagePtr(const sensor_msgs::ImageConstPtr& image_msg, const std::string& encoding);
void poseToRos(const Transformation& transform,
			   geometry_msgs::PoseStamped* pose_msg);
void transformToTf(const Transformation& transform,
				   tf::Transform* tf_transform);
void transformToRos(const Transformation& transform,
					geometry_msgs::TransformStamped* transform_msg);
void timestampToRos(uint64_t timestamp_ns, ros::Time* time);

std::string getCameraFrameId(int cam_id);
std::string getSensorFrameId(std::string frame_id, int cam_id);

}  // namespace kitty

#endif  // TERRESLAM_KITTI_ROS_CONVERSIONS_H_
