/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-02-24 15:14:49
 *    Last Modified: 2021-02-24 15:25:32
 */

#pragma once

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace terreslam
{

class IoDisk
{
public:
	IoDisk();

	void WriteNormals
		(pcl::PointCloud<pcl::Normal>::ConstPtr normals);

private:
	bool visualizing_;
	
};

}
