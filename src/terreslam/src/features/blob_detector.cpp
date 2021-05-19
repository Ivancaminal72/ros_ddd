/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-04-23 09:40:30
 */

#include "terreslam/features/blob_detector.h"

namespace terreslam
{

void BlobDetector::detectBlobs(Scan *scan)
{
	fp_.open(logs_path_, std::ios::app);
	if(debug_)
	{
		fp_<<"*****************detectBlobs**************************************"<<std::endl;
	}
	
	 // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
  tree->setInputCloud (scan->points());

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
  ec.setClusterTolerance (0.15); // 2cm
  ec.setMinClusterSize (100/1.8);
  ec.setMaxClusterSize (25000/1.8);
  ec.setSearchMethod (tree);
  ec.setInputCloud (scan->points());
  ec.extract (cluster_indices);

  // int j = 0;
  // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  // {
  //   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
  //   for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
  //     cloud_cluster->push_back ((*cloud_filtered)[*pit]); //*
  //   cloud_cluster->width = cloud_cluster->size();
  //   cloud_cluster->height = 1;
  //   cloud_cluster->is_dense = true;

  //   std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
  //   std::stringstream ss;
  //   ss << "cloud_cluster_" << j << ".pcd";
  //   writer.write<pcl::PointXYZRGBA>(ss.str (), *cloud_cluster, false); //*
  //   j++;
  // }

	fp_.close();

}

}