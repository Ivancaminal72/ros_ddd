/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-07-05 12:41:26
 */

#pragma once

#include <vector>
#include <deque>
#include <iostream>
#include <algorithm>

#include <opencv2/core/mat.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

namespace terreslam
{
namespace util
{
	float calculateMedian(std::vector<float> scores);
	float calculateMedian(std::deque<float> scores);
	cv::Mat quat2Mat(tf2::Quaternion q);
}
}