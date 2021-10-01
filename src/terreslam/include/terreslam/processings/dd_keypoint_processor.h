/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-04-23 10:32:49
 */

#pragma once

#include "terreslam/utils/util_types.h"
#include "terreslam/utils/util_map.h"

namespace terreslam
{

	std::vector<cv::DMatch> matchTwoImage(const cv::Mat& query_desc, const cv::Mat& train_desc);
	void nonZeroWindowContourLookUp(int& v, int& u, const int& ws, const cv::Mat& img);
	std::vector<cv::Point3f> backprojectKeypoints(const std::vector<cv::KeyPoint>& kpts, const Eigen::Matrix4d& P_inv, const double& depth_scale, const cv::Mat& img_depth, const int& max_ws);

}
