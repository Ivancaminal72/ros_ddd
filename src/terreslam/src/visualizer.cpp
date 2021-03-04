/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-02-23 11:45:55
 */

#include <iostream>

#include "terreslam/visualizer.h"

namespace terreslam
{

Visualizer::Visualizer() 
	: 
	visualizing_(false)
{

	std::cout<<"Constructor visualizer"<<std::endl;

	int v1(0);
	viewer_ = boost::shared_ptr<pcl::visualization::PCLVisualizer> 
		(new pcl::visualization::PCLVisualizer("3D Normal Viewer"));
	viewer_->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer_->setBackgroundColor (0, 0, 0);
  viewer_->addCoordinateSystem (1.0);
  viewer_->initCameraParameters ();

	// viewerThread_ = boost::shared_ptr<boost::thread>
  //   (new boost::thread(boost::bind(&Visualizer::viewerThreadFunction, this)));
}

void Visualizer::NormalView1 
	(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud,
	 pcl::PointCloud<pcl::Normal>::ConstPtr normals) 
{
		// mutex_viewer_.lock();
	viewer_->removeAllShapes();
	std::cout<<"hola1\n";
	// viewer_->addPointCloudNormals<pcl::PointXYZRGBA,pcl::Normal>(cloud, normals);
	viewer_->addPointCloud<pcl::PointXYZRGBA>(cloud, "3D Normal Viewer");
	std::cout<<"hola2\n";
	viewer_->spinOnce(100);
	// mutex_viewer_.unlock();
	visualizing_=true;
}

void Visualizer::viewerThreadFunction() 
{
	// while (true)
	// {
	// 	if(visualizing_)
	// 	{
	// 		mutex_viewer_.lock();
	// 		if(!viewer_->wasStopped()) viewer_->spinOnce(100);
	// 		mutex_viewer_.unlock();
	// 	}
	// 	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	// }
}

}