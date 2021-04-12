/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-02-25 13:25:42
 */

#include <pcl/filters/extract_indices.h>

#include "terreslam/utils/util_pcd.h"

namespace terreslam
{

namespace util
{

	void curvatureFilter(Scan *scan, float thresh, bool high_pass)
 {
	std::vector<int> inidices;
	// Reserve enough space for the indices
	inidices.resize(scan->points()->size());
  
	int j = 0;
	for (int i = 0; i < static_cast<int>(scan->points()->size()); ++i)
	{
		if(high_pass && scan->normals()->at(i).curvature < thresh) 
			continue;
		inidices[j] = i;
		j++;
	}
	if (j != static_cast<int> (scan->points()->size()))
	{
		// Resize to the correct size
		inidices.resize (j);
	}
	scan->filter(inidices);
 }

	void printEigenMatrix(Eigen::MatrixXd mat)
	{
		std::cout << std::endl << std::endl;
		std::string sep = "\n----------------------------------------\n";
		Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");
		std::cout << mat.matrix().format(CleanFmt) << std::endl;
		std::cout << std::endl << std::endl;
	}
	
	template <typename PointT> 
	void pointType<PointT>::subtractPoints(typename pcl::PointCloud<PointT>::Ptr cloud, const std::vector<int>& indices)
	{
		pcl::PointIndices::Ptr fIndices (new pcl::PointIndices);

		fIndices->indices = indices;

		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud (cloud);
		extract.setIndices (fIndices);
		extract.setNegative(false);
		extract.filter(*cloud);
	}
}

}