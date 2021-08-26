/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-08-23 16:52:09
 */

#pragma once

#include "terreslam/utils/util_types.h"
#include "terreslam/utils/util_map.h"

namespace terreslam
{

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr detectSIFTKeypoints (
	const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & points, 
	const pcl::PointCloud<pcl::Normal>::Ptr & normals,
	float min_scale, 
	int nr_octaves, 
	int nr_scales_per_octave, 
	float min_contrast);

pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFHDescriptors (
	const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & points, 
	const pcl::PointCloud<pcl::Normal>::Ptr & normals,
	const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & keypoints, 
	float feature_radius);

}