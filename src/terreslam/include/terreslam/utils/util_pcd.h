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
	void curvatureFilter(ptrPointCloud points, ptrNormalCloud normals, float thresh, bool high_pass=true);
	void printEigenMatrix(Eigen::MatrixXd mat);
	void subtractPointsXYZRGBA(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, const std::vector<int>& indices);
	void subtractPointsNormal(pcl::PointCloud<pcl::Normal>::Ptr cloud, const std::vector<int>& indices);
	void subtractPointsXY(pcl::PointCloud<pcl::PointXY>::Ptr cloud, const std::vector<int>& indices);
}

}