/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-08-27 10:11:20
 */

#pragma once

#include "terreslam/utils/util_types.h"
#include "terreslam/utils/util_map.h"

#include <vector>

namespace terreslam
{
	float sqErr_6Dof(cv::Point3f p1, cv::Point3f p2, const cv::Mat& RTr);
	float RMSE_6Dof(const std::vector<cv::Point3f>& src, const std::vector<cv::Point3f>& dst, const cv::Mat& RTr, const bool* inliers, const cv::Point3f center);
	int setInliers6Dof(const std::vector<cv::Point3f>& src, const std::vector <cv::Point3f>& dst, bool* inliers, const cv::Mat& RTr, const float max_er, const cv::Point3f center);
	float fit6DofQUADRATIC(const std::vector<cv::Point3f>& src_, const std::vector<cv::Point3f>& dst_, float* param, cv::Mat& param_RTr, const bool* inliers, const cv::Point3f center);
	float fit6DofRANSAC(const std::vector<cv::Point3f>& src, const std::vector<cv::Point3f>& dst, float* best_param, cv::Mat& best_param_RTr, bool* inliers, const cv::Point3f center, const float inlierMaxEr, const int niter, const bool debug);
}

