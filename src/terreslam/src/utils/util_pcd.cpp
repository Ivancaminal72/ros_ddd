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

	void curvatureFilter(ptrPointCloud points, ptrNormalCloud normals, float thresh, bool high_pass)
 {
	std::vector<int> inidices;
	// Reserve enough space for the indices
	inidices.resize(points->size());
  
	int j = 0;
	for (int i = 0; i < static_cast<int>(points->size()); ++i)
	{
		if(high_pass && normals->at(i).curvature < thresh) 
			continue;
		inidices[j] = i;
		j++;
	}
	if (j != static_cast<int> (points->size()))
	{
		// Resize to the correct size
		inidices.resize (j);
	}
	util::subtractPointsXYZRGBA(points, inidices);
	util::subtractPointsNormal(normals, inidices);
 }

	void printEigenMatrix(Eigen::MatrixXd mat)
	{
		std::cout << std::endl << std::endl;
		std::string sep = "\n----------------------------------------\n";
		Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");
		std::cout << mat.matrix().format(CleanFmt) << std::endl;
		std::cout << std::endl << std::endl;
	}
	
	void subtractPointsXYZRGBA(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, const std::vector<int>& indices)
	{
		pcl::PointIndices::Ptr fIndices (new pcl::PointIndices);

		fIndices->indices = indices;

		pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
		extract.setInputCloud (cloud);
		extract.setIndices (fIndices);
		extract.setNegative(false);
		extract.filter(*cloud);
	}

	void subtractPointsNormal(pcl::PointCloud<pcl::Normal>::Ptr cloud, const std::vector<int>& indices)
	{
		pcl::PointIndices::Ptr fIndices (new pcl::PointIndices);

		fIndices->indices = indices;

		pcl::ExtractIndices<pcl::Normal> extract;
		extract.setInputCloud (cloud);
		extract.setIndices (fIndices);
		extract.setNegative(false);
		extract.filter(*cloud);
	}
	void subtractPointsXY(pcl::PointCloud<pcl::PointXY>::Ptr cloud, const std::vector<int>& indices)
	{
		pcl::PointIndices::Ptr fIndices (new pcl::PointIndices);

		fIndices->indices = indices;

		pcl::ExtractIndices<pcl::PointXY> extract;
		extract.setInputCloud (cloud);
		extract.setIndices (fIndices);
		extract.setNegative(false);
		extract.filter(*cloud);
	}
}

}