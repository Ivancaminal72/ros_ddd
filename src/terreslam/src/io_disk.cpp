/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-02-24 15:12:40
 *    Last Modified: 2021-02-24 15:26:41
 */

#include <iostream>

#include "terreslam/io_disk.h"

namespace terreslam
{

IoDisk::IoDisk() 
	: 
	visualizing_(false)
{}

void IoDisk::WriteNormals(pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  
}

}