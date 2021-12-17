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

	//input: ratio is between 0 to 1
	//output: rgb color
	uint32_t rgba_palette(double ratio)
	{
			//we want to normalize ratio so that it fits in to 6 regions
			//where each region is 256 units long
			int normalized = int(ratio * 256 * 6);

			//find the distance to the start of the closest region
			int x = normalized % 256;

			uint8_t alpha=255, red = 0, grn = 0, blu = 0;
			switch(normalized / 256)
			{
			case 0: red = 255;      grn = x;        blu = 0;       break;//red
			case 1: red = 255 - x;  grn = 255;      blu = 0;       break;//yellow
			case 2: red = 0;        grn = 255;      blu = x;       break;//green
			case 3: red = 0;        grn = 255 - x;  blu = 255;     break;//cyan
			case 4: red = x;        grn = 0;        blu = 255;     break;//blue
			case 5: red = 255;      grn = 0;        blu = 255 - x; break;//magenta
			}

			return (alpha << 24) + (red << 16) + (grn << 8) + blu;
	}

	void curvatureFilter(ptrPointCloud points, ptrNormalCloud normals, ptrPointCloud high_points, ptrNormalCloud high_normals, ptrPointCloud low_points, ptrNormalCloud low_normals, float thresh)
 {
	std::vector<int> inidices;
	// Reserve enough space for the indices
	inidices.resize(points->size());
  
	int j = 0;
	for (int i = 0; i < static_cast<int>(points->size()); ++i)
	{
		if(normals->at(i).curvature < thresh) 
			continue;
		inidices[j] = i;
		j++;
	}
	if (j != static_cast<int> (points->size()))
	{
		// Resize to the correct size
		inidices.resize (j);
	}
	util::splitPointsXYZRGBA(points, high_points, low_points, inidices);
	util::splitPointsNormal(normals, high_normals, low_normals, inidices);
 }

	void printEigenMatrix(Eigen::MatrixXd mat)
	{
		std::cout << std::endl << std::endl;
		std::string sep = "\n----------------------------------------\n";
		Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");
		std::cout << mat.matrix().format(CleanFmt) << std::endl;
		std::cout << std::endl << std::endl;
	}
	
	void splitPointsXYZRGBA(ptrPointCloud cloud, ptrPointCloud high_cloud, ptrPointCloud low_cloud, const std::vector<int>& indices)
	{
		pcl::PointIndices::Ptr fIndices (new pcl::PointIndices);

		fIndices->indices = indices;

		pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
		extract.setInputCloud (cloud);
		extract.setIndices (fIndices);
		if(high_cloud != NULL){
			extract.setNegative(false);
			extract.filter(*high_cloud);
		}
		if(low_cloud != NULL){
			extract.setNegative(true);
			extract.filter(*low_cloud);
		}
	}

	void splitPointsNormal(ptrNormalCloud cloud, ptrNormalCloud high_cloud, ptrNormalCloud low_cloud, const std::vector<int>& indices)
	{
		pcl::PointIndices::Ptr fIndices (new pcl::PointIndices);

		fIndices->indices = indices;

		pcl::ExtractIndices<pcl::Normal> extract;
		extract.setInputCloud (cloud);
		extract.setIndices (fIndices);
		if(high_cloud != NULL){
			extract.setNegative(false);
			extract.filter(*high_cloud);
		}
		if(low_cloud != NULL){
			extract.setNegative(true);
			extract.filter(*low_cloud);
		}
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