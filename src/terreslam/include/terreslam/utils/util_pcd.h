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
	void curvatureFilter(Scan *scan, float thresh, bool high_pass=true);
	void printEigenMatrix(Eigen::MatrixXd mat);
	template <typename PointT> 
	struct pointType
	{
		void subtractPoints(typename pcl::PointCloud<PointT>::Ptr cloud, const std::vector<int>& indices);
	};
}

}