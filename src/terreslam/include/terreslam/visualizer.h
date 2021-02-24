/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-02-23 11:46:25
 *    Last Modified: 2021-02-23 12:26:28
 */

#pragma once

#include <iostream>
#include <mutex>
#include <boost/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

namespace terreslam
{

class Visualizer
{
public:
	Visualizer();

	void NormalView1
		(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud,
		 pcl::PointCloud<pcl::Normal>::ConstPtr normals);

private:
  boost::shared_ptr<boost::thread> viewerThread_;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
  std::mutex mutex_viewer_;
  bool visualizing_;
	void viewerThreadFunction();
};

}