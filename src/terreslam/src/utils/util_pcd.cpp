/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-02-25 13:25:42
 *    Last Modified: 2021-02-25 13:59:10
 */

#include <pcl/filters/extract_indices.h>

#include "terreslam/utils/util_pcd.h"

namespace terreslam
{

namespace util
{

	void printEigenMatrix(Eigen::MatrixXd mat)
	{
		std::cout << std::endl << std::endl;
		std::string sep = "\n----------------------------------------\n";
		Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");
		std::cout << mat.matrix().format(CleanFmt) << std::endl;
		std::cout << std::endl << std::endl;
	}

	void subtractPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<int>& indices)
	{
		pcl::PointIndices::Ptr fIndices (new pcl::PointIndices);

		fIndices->indices = indices;

		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud);
		extract.setIndices (fIndices);
		extract.setNegative(false);
		extract.filter(*cloud);
	}

}

}