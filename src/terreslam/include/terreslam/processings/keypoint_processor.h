/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-04-23 10:32:49
 */

#pragma once

#include "terreslam/utils/util_types.h"
#include "terreslam/utils/util_map.h"

namespace terreslam
{

	std::vector<cv::DMatch> matchTwoImage(const cv::Mat &descriptor1, const cv::Mat &descriptor2);
	void nonZeroWindowContourLookUp(int& u, int& v, const int& ws, const cv::Mat& img);

}
