/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-02-25 13:26:36
 */

#pragma once

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace terreslam
{

namespace util
{

	void printEigenMatrix(Eigen::MatrixXd mat);
	void subtractPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<int>& inliers);

}

}