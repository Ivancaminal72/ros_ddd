/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-02-25 13:26:36
 */

#pragma once

#include "terreslam/utils/util_map.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace terreslam
{

namespace util
{
	uint32_t rgba_palette(double ratio);
	void curvatureFilter(ptrPointCloud points, ptrNormalCloud normals, ptrPointCloud high_points, ptrNormalCloud high_normals, ptrPointCloud low_points, ptrNormalCloud low_normals, float thresh);
	void printEigenMatrix(Eigen::MatrixXd mat);
	void splitPointsXYZRGBA(ptrPointCloud cloud, ptrPointCloud high_cloud, ptrPointCloud low_cloud, const std::vector<int>& indices);
	void splitPointsNormal(ptrNormalCloud cloud, ptrNormalCloud high_cloud, ptrNormalCloud low_cloud, const std::vector<int>& indices);
	void subtractPointsXY(pcl::PointCloud<pcl::PointXY>::Ptr cloud, const std::vector<int>& indices);
}

}