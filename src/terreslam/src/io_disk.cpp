/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-02-24 15:12:40
 */

#include "terreslam/io_disk.h"
#include "terreslam/utils/util_pcd.h"


namespace terreslam
{

IoDisk::IoDisk() 
	: 
	visualizing_(false)
{}

void IoDisk::WriteNormals
	(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_xyzrgba,
	 pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr clean_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::copyPointCloud(*cloud_xyzrgba, *cloud_xyz);
	
	// //Filter NAN normals
	// std::vector<int> indices;
	// pcl::removeNaNNormalsFromPointCloud(*normals, *clean_normals, indices);
	// util::subtractPoints(cloud_xyz, indices);
	
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud_xyz, *normals, *cloud_with_normals);
	pcl::io::savePCDFileASCII("/home/icaminal/outputs/unorganized/plane_detector/test_pcd.pcd", *cloud_with_normals);
}

}