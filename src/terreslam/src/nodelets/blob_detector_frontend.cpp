/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-06-15 10:37:02
 */

#include "terreslam/frontend.h"
#include "terreslam/utils/util_pcd.h"
#include "terreslam/utils/util_chrono.h"
#include "terreslam/features/blob_detector.h"

#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <math.h>

namespace terreslam
{

class BlobDetectorFrontend : public terreslam::Frontend
{
public:
	BlobDetectorFrontend() :
		queue_size_(10)
		{
			std::cout << "Constructor blob_detector_frontend..." << std::endl;
		}

private:

	void onFrontendInit()
	{
		std::cout << "Initalize blob_detector_frontend..." << std::endl;
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();
		ros::NodeHandle cf_nh(nh, "cloud_filtered");

		/// Subscribers
		cloud_filtered_sub_ = cf_nh.subscribe(cloud_filtered_frame_id, queue_size_, &BlobDetectorFrontend::callback, this);

		// Publishers
		blob_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cloud_filtered_blobs_frame_id, 10);
	} 

	void callback(
		const sensor_msgs::PointCloud2ConstPtr& cf_msg_ptr)
	{
		std::cout << "Entry blob: " << entry_count << std::endl;
		// ///Start chrono ticking
		// std::chrono::duration<double> tick;
		// std::chrono::high_resolution_clock::time_point end_t, start_t;
		// start_t = std::chrono::high_resolution_clock::now();
		// end_t = std::chrono::high_resolution_clock::now();
		// tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr points (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::fromROSMsg(*cf_msg_ptr, *points);
    
		///PARALLEL PROJECTION
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr points_pp (new pcl::PointCloud<pcl::PointXYZRGBA>);
		*points_pp = *points;
		for(auto& p_pp : *points_pp)
		{
			p_pp.y = 0;
		}


		///BLOB DETECTION
		// Creating the KdTree object for the search method of the extraction
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
		tree->setInputCloud (points_pp);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
		ec.setClusterTolerance (BD_tolerance);
		ec.setMinClusterSize (BD_min_size);
		ec.setMaxClusterSize (BD_max_size);
		ec.setSearchMethod (tree);
		ec.setInputCloud (points_pp);
		ec.extract (cluster_indices);

		// std::cout<<"Cluster size: "<<cluster_indices.size()<<std::endl;

		/// Blobs creation
		current_blobs.clear();
		int j = 0;
		float max_height=0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j+=1)
		{
			Blob blob;
			// uint32_t rgb = util::rgb_palette((double)j/cluster_indices.size());
			float height, height_acc=0, height2_acc=0, height_avg, height2_avg, height_dev;
			float centroid_dev;
			float x, z, x_acc=0, z_acc=0, x2_acc=0, z2_acc=0, x_avg, z_avg, x2_avg, z2_avg;
			for (const auto& idx1 : it->indices)
			{
				x=(*points)[idx1].x;
				z=(*points)[idx1].z;
				height = (*points)[idx1].y;
				if(height > 3) continue;
				else height = -1*height+3;
				// if(height > max_height) max_height=height;
				x_acc += x;
				z_acc += z;
				x2_acc += x*x;
				z2_acc += z*z;
				height_acc += height;
				height2_acc += pow(height, 2);
				// (*points_ori)[idx1].rgb=rgb;
			}
			x_avg = x_acc / it->indices.size(); //Centroide (x_avg, z_avg)
			z_avg = z_acc / it->indices.size();
			x2_avg = x2_acc / it->indices.size();
			z2_avg = z2_acc / it->indices.size();
			centroid_dev = 2*sqrt(x2_avg - pow(x_avg,2) + z2_avg - pow(z_avg,2));//Centroide dev (estimació radi)
			height_avg = height_acc / it->indices.size();
			height2_avg = height2_acc / it->indices.size();
			height_dev = 2*sqrt(height2_avg - pow(height_avg,2)); //Height_avg+height_dev (estimació alçada)
			
			blob.height = height_avg+height_dev;
			blob.radius = centroid_dev;
			blob.x=x_avg;
			blob.z=z_avg;

			current_blobs.emplace_back(blob);
		}

		// for(Blob blob : current_blobs)
		// {
		// 	std::cout<<"Blob height: "<<blob.height<<std::endl;
		// 	std::cout<<"Blob radius: "<<blob.radius<<std::endl;
		// 	std::cout<<"Blob x: "<<blob.x<<std::endl;
		// 	std::cout<<"Blob z: "<<blob.z<<std::endl;
		// }

		// ///MATCHING
		// //eigen matrix blob_dist[i,j]
		// if(map_blobs.size() == 0) 
		// {
		// 	map_blobs=current_blobs;
		// 	return;
		// }
		// else
		// {
		// 	float centroid_dist, radius_dist, height_dist;
		// 	for(Blob blob : current_blobs)
		// 	{
		// 		for(Blob blob_old : map_blobs)
		// 		{
		// 			centroid_dist = pow(blob.x - blob_old.x,2)+pow(blob.z - blob_old.z,2);
		// 			radius_dist = blob.radius - blob_old.radius;
		// 			height_dist = blob.height - blob_old.height;
		// 			blob_dist[i,j] = pow(centroid_dist,2) + pow(radius_dist,2) + pow(height_dist,2); 
		// 		}
		// 	}
		// }



		/// PUBLISH
		/// - Cloud Filtered Blobs
		sensor_msgs::PointCloud2 msg_pcd;
		pcl::toROSMsg(*points, msg_pcd);
		msg_pcd.header.frame_id = cloud_filtered_blobs_frame_id;
		msg_pcd.header.stamp = cf_msg_ptr->header.stamp;
		blob_pub_.publish(msg_pcd);

		/// - Blobs
		// pcl::toROSMsg(*scan_planes, msg_pcd);
		// msg_pcd.header.frame_id = cloud_plane_frame_id;
		// plane_pub.publish(msg_pcd);

		entry_count++;

		// tick_high_resolution(start_t, tick, elapsed);
		// printElapsed(elapsed, "Callback blob detector: ");
	}

	void skipFrame(std::string msg)
	{
		std::cerr<<msg<<std::endl;
		entry_count++;
	}

private:

	/// General variables
	int queue_size_;

	/// Comms
	ros::Publisher blob_pub_;
	ros::Subscriber cloud_filtered_sub_;

	/// Chrono timmings
	std::vector<double> elapsed;

	/// Blobs
	std::vector<Blob> current_blobs;
	std::vector<Blob> map_blobs; //Per cadascun hi ha un centroid, radi, height
	//frame discutir

};

PLUGINLIB_EXPORT_CLASS(terreslam::BlobDetectorFrontend, nodelet::Nodelet);

}