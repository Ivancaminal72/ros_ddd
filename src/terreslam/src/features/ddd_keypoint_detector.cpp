/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-08-23 17:06:03
 */

#include "terreslam/features/ddd_keypoint_detector.h"
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>

namespace terreslam
{

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr detectSIFTKeypoints (
	const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & points, 
	const pcl::PointCloud<pcl::Normal>::Ptr & normals,
	float min_scale, 
	int nr_octaves, 
	int nr_scales_per_octave, 
	float min_contrast)
	{
		pcl::SIFTKeypoint<pcl::PointXYZRGBA, pcl::PointWithScale> sift_detect;
		sift_detect.setSearchMethod (pcl::search::Search<pcl::PointXYZRGBA>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGBA>));
		sift_detect.setScales (min_scale, nr_octaves, nr_scales_per_octave);
		sift_detect.setMinimumContrast (min_contrast);
		sift_detect.setInputCloud (points);
		pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
		sift_detect.compute (keypoints_temp);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::copyPointCloud (keypoints_temp, *keypoints);
		return keypoints;
	}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFHDescriptors (
	const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & points, 
	const pcl::PointCloud<pcl::Normal>::Ptr & normals,
	const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & keypoints, 
	float feature_radius)
	{
		pcl::FPFHEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
		fpfh_estimation.setNumberOfThreads(6);
		fpfh_estimation.setSearchMethod (pcl::search::Search<pcl::PointXYZRGBA>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGBA>));
		fpfh_estimation.setRadiusSearch (feature_radius);
		fpfh_estimation.setSearchSurface (points);  
		fpfh_estimation.setInputNormals (normals);
		fpfh_estimation.setInputCloud (keypoints);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr local_descriptors (new pcl::PointCloud<pcl::FPFHSignature33>);
		fpfh_estimation.compute (*local_descriptors);

		return local_descriptors;
	}

}