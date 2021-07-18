/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-06-15 10:37:02
 */

#include "terreslam/frontend.h"
#include "terreslam/utils/util_pcd.h"
#include "terreslam/utils/util_chrono.h"
#include "terreslam/utils/util_general.h"
#include "terreslam/features/blob_detector.h"

#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <math.h>
#include <fstream>

#include <Eigen/Dense>

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

		// Initialize palette
		for(int i=0; i<50; ++i)
				rgb[i] = util::rgb_palette((double)i/50);

		// Initialize logging
		// logs_path_ = logs_dir + "/log_dynamic_blob_detector.csv";
		// std::cout<<"Opening file to write at path: "<<logs_path_<<std::endl;
		// fp_.open(logs_path_);
		// if (fp_.is_open()) std::cout << "OK!\n";
		// else std::cerr << "Error opening file: "<< strerror(errno) <<std::endl;
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
		// float max_height=0;
		int j = 0;
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

		///MATCHING
		if(map_blobs.size() == 0) //First time initialize map
		{
			old_points = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
			map_blobs=current_blobs;
			old_cluster_indices=cluster_indices;
			old_points = points->makeShared();
			
			for(Blob blob : map_blobs)
			{
				int randNum = rand()%(50); //Rand number between 0 and 49
				blob.palette= rgb[randNum];
			}
			entry_count++;
			return;
		}
		else //Attempt matching
		{
			matches.clear();
			int cur_size = cluster_indices.size();
			int old_size = old_cluster_indices.size();
			Eigen::MatrixXf blob_dist(cur_size,old_size);
			Eigen::MatrixXf blob_dist_xz(cur_size,old_size);
			float centroid_dist, radius_dist, height_dist;
			int i = 0;
			for(Blob blob : current_blobs)
			{
				int j = 0;
				for(Blob blob_old : map_blobs)
				{
					centroid_dist = sqrt(pow(blob.x - blob_old.x,2)+pow(blob.z - blob_old.z,2));
					radius_dist = blob.radius - blob_old.radius;
					height_dist = blob.height - blob_old.height;
					blob_dist(i,j) = pow(centroid_dist,2) + pow(radius_dist,2) + pow(height_dist,2);
					blob_dist_xz(i,j) = centroid_dist;
					j+=1;
				}
				i+=1;
			}

			i=0;
			for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, i+=1)
			{
				Eigen::MatrixXf::Index minIndexCol, minIndexRow;
				blob_dist.row(i).minCoeff(&minIndexRow);
				blob_dist.col(minIndexRow).minCoeff(&minIndexCol);
				if(i == minIndexCol) //MATCHED
				{
					current_blobs.at(i).palette = map_blobs.at(minIndexRow).palette;
					matches.emplace_back(i,minIndexRow);
					matches_dist_xz.emplace_back(blob_dist_xz(i,minIndexRow));
				}
				else //NOT MATCHED
				{
					int randNum = rand()%(50); //Rand number between 0 and 49
					current_blobs.at(i).palette = rgb[randNum];
				}
			}

			j=0;
			for (std::vector<pcl::PointIndices>::const_iterator it = old_cluster_indices.begin (); it != old_cluster_indices.end (); ++it, j+=1)
			{
				for (const auto& idx : it->indices)
				{
					(*old_points)[idx].rgb=map_blobs.at(j).palette;
				}
			}

			//DYNAMIC Blobs
			std::vector<int> indices_to_delete;
			std::vector<float> matches_dist_xz_small = matches_dist_xz;
			
			for(i=0; i<matches.size(); ++i)
			{
				if(current_blobs.at(matches.at(i).first).radius > (2*BD_thres_radius))
				{
					indices_to_delete.emplace_back(i);		
					// std::vector<pcl::PointIndices>::const_iterator it = old_cluster_indices.begin()+matches.at(i).second;
					// for (const auto& idx : it->indices)
					// {
					// 	uint32_t rgb_color = ((uint8_t)255 << 16) + ((uint8_t)255 << 8) + (uint8_t)255; //white
					// 	(*old_points)[idx].rgb= rgb_color;
					// }
				}
			}
			for(i=0; i<indices_to_delete.size(); ++i)
			{
				matches_dist_xz_small.erase(matches_dist_xz_small.begin()+indices_to_delete.at(i)-i);
			}

			
			float median_xz_curr = util::calculateMedian(matches_dist_xz_small);
			median_xz = BD_alpha * median_xz_curr + (1-BD_alpha) * median_xz;

			if(median_xz > 0)
			{
				indices_to_delete.clear();
				for(i=0; i<matches.size(); ++i)
				{
					// fp_<<matches_dist_xz.at(i)<<std::endl;
					// if((abs(matches_dist_xz.at(i) - median_xz) > 0.035) && (abs(acos(x_avg*xi+z_avg*zi)) > M_PI/10)) //DYNAMIC
					if(abs(matches_dist_xz.at(i) - median_xz) > BD_thres_xz) //DYNAMIC
					{
						indices_to_delete.emplace_back(i);
						std::vector<pcl::PointIndices>::const_iterator it = old_cluster_indices.begin()+matches.at(i).second;
						for (const auto& idx : it->indices)
						{
							uint32_t rgb_color = ((uint8_t)255 << 16) + ((uint8_t)255 << 8) + (uint8_t)255; //white
							(*old_points)[idx].rgb= rgb_color;
						}
					} 
						
				}
				for(i=0; i<indices_to_delete.size(); ++i)
				{
					//old_cluster_indices.erase(old_cluster_indices.begin()+matches.at(indices_to_delete.at(i)).second-i);
					matches.erase(matches.begin()+indices_to_delete.at(i)-i);
					matches_dist_xz.erase(matches_dist_xz.begin()+indices_to_delete.at(i)-i);
				}
			}

		}

		/// PUBLISH
		/// - Cloud Filtered Blobs
		sensor_msgs::PointCloud2 msg_pcd;
		pcl::toROSMsg(*old_points, msg_pcd);
		msg_pcd.header.frame_id = cloud_filtered_blobs_frame_id;
		msg_pcd.header.stamp = cf_msg_ptr->header.stamp;
		blob_pub_.publish(msg_pcd);

		/// - Blobs
		// pcl::toROSMsg(*scan_planes, msg_pcd);
		// msg_pcd.header.frame_id = cloud_plane_frame_id;
		// plane_pub.publish(msg_pcd);

		//Update old data
		map_blobs=current_blobs;
		old_cluster_indices=cluster_indices;
		old_points->clear(); //Necessary??
		old_points = points->makeShared();
		
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
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr old_points;
	std::vector<pcl::PointIndices> old_cluster_indices;
	//frame discutir
	std::vector<std::pair<int,int>> matches;
	std::vector<float> matches_dist_xz;
	float median_xz;

	uint32_t rgb[50];

	/// Logging
	std::ofstream fp_;
	std::string logs_path_;

};

PLUGINLIB_EXPORT_CLASS(terreslam::BlobDetectorFrontend, nodelet::Nodelet);

}