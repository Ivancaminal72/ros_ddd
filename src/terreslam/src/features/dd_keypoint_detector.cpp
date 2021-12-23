/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-04-23 09:40:30
 */

#include "terreslam/features/dd_keypoint_detector.h"
#include <opencv2/features2d.hpp>
#include <opencv4/opencv2/xfeatures2d.hpp>

namespace terreslam
{

std::vector<cv::KeyPoint> detectGFTTKeyPoints(const cv::Mat &image) 
{
    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(image, corners, 500, 0.05, 6);
    std::vector<cv::KeyPoint> kpts;
    for( size_t i = 0; i < corners.size(); ++i )
        kpts.push_back(cv::KeyPoint(corners[i], 1.f));
    return kpts;
}

cv::Mat computeBriefDescriptors(const cv::Mat &image, std::vector<cv::KeyPoint> &kpts)
{
    auto featureExtractor = cv::xfeatures2d::BriefDescriptorExtractor::create(32);
    cv::Mat descriptors;
    featureExtractor->compute(image, kpts, descriptors);
    return descriptors;
}

}