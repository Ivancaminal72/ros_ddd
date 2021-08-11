/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-08-04 17:32:35
 */

#pragma once

#include "terreslam/utils/util_types.h"
#include "terreslam/utils/util_map.h"

#include <vector>

namespace terreslam
{
	float sqErr_3Dof(cv::Point2f p1, cv::Point2f p2, float cos_alpha, float sin_alpha, cv::Point2f T);
	float RMSE_3Dof(const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst, const float* param, const bool* inliers, const cv::Point2f center);
	int setInliers3Dof(const std::vector<cv::Point2f>& src, const std::vector <cv::Point2f>& dst, bool* inliers, const float* param, const float max_er, const cv::Point2f center);
	float fit3DofQUADRATICold(const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst, float* param, const bool* inliers, const cv::Point2f center);
	float fit3DofQUADRATIC(const std::vector<cv::Point2f>& src_, const std::vector<cv::Point2f>& dst_, float* param, const bool* inliers, const cv::Point2f center);
	float fit3DofRANSAC(const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst, float* best_param,  bool* inliers, const cv::Point2f center, const float inlierMaxEr, const int niter);
}

