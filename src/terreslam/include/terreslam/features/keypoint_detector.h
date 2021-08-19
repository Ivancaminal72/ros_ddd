/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-04-23 09:40:10
 */

#pragma once

#include "terreslam/utils/util_types.h"
#include "terreslam/utils/util_map.h"

namespace terreslam
{

std::vector<cv::KeyPoint> detectKeyPoints(const cv::Mat &image);
cv::Mat computeDescriptors(const cv::Mat &image, std::vector<cv::KeyPoint> &keyPoints);

}