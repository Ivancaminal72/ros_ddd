/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-06-15 10:37:02
 */

#include "terreslam/nodelet.h"
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

//msgs
#include <terreslam/BlobMatches.h>
#include <terreslam/BlobPoints.h>

namespace terreslam
{

class BlobDetectorNodelet : public terreslam::Nodelet
{
public:
	BlobDetectorNodelet() :
		queue_size_(10)
		{
			// std::cout << "Constructor blob_detector_nodelet..." << std::endl;
		}

private:

	void onNodeletInit()
	{
		std::cout << "Initalize blob_detector_nodelet..." << std::endl;
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		/// Subscribers
		cloud_filtered_sub_ = nh.subscribe(cloud_filtered_high_topic, queue_size_, &BlobDetectorNodelet::callback, this);

		// Publishers
		cloud_blobs_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cloud_filtered_blobs_topic, 10);
		blob_matches_pub_ = nh.advertise<terreslam::BlobMatches>(blob_matches_topic, 1);
		blob_points_pub_ = nh.advertise<terreslam::BlobPoints>(blob_points_topic, 1);

		// Initialize palette
		for(i=0; i<50; ++i)
				rgba[i] = util::rgba_palette((double)i/50);

		srand (time(NULL));

		// Initialize logging
		// logs_path_ = logs_dir + "/log_dynamic_blob_detector.csv";
		// std::cout<<"Opening file to write at path: "<<logs_path_<<std::endl;
		// fp_.open(logs_path_);
		// if (fp_.is_open()) std::cout << "OK!\n";
		// else std::cerr << "Error opening file: "<< strerror(errno) <<std::endl;
	} 

	void callback(
		const sensor_msgs::PointCloud2::ConstPtr& cf_msg_ptr)
	{
		if(debug) std::cout << "Entry blob: " << entry_count << std::endl;
		// ///Start chrono ticking
		// std::chrono::duration<double> tick;
		// std::chrono::high_resolution_clock::time_point end_t, start_t;
		// start_t = std::chrono::high_resolution_clock::now();
		// end_t = std::chrono::high_resolution_clock::now();
		// tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);

		ros::Time cur_stamp = cf_msg_ptr->header.stamp;

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
		std::vector<int> indices_to_delete;
		j=0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, j+=1)
		{
			Blob blob;
			// uint32_t rgba = util::rgba_palette((double)j/cluster_indices.size());
			float height, height_acc=0, height2_acc=0, height_avg, height2_avg, height_dev;
			float centroid_dev;
			float x, z, x_acc=0, z_acc=0, x2_acc=0, z2_acc=0, x_avg, z_avg, x2_avg, z2_avg;
			for (const auto& idx : it->indices)
			{
				x=(*points)[idx].x;
				z=(*points)[idx].z;
				height = (*points)[idx].y;
				if(height > 3) continue;
				else height = -1*height+3;
				// if(height > max_height) max_height=height;
				x_acc += x;
				z_acc += z;
				x2_acc += x*x;
				z2_acc += z*z;
				height_acc += height;
				height2_acc += pow(height, 2);
				// (*points_ori)[idx].rgba=rgba;
			}
			x_avg = x_acc / it->indices.size(); //Centroide (x_avg, z_avg)
			z_avg = z_acc / it->indices.size();
			x2_avg = x2_acc / it->indices.size();
			z2_avg = z2_acc / it->indices.size();
			centroid_dev = 2*sqrt(x2_avg - pow(x_avg,2) + z2_avg - pow(z_avg,2));//Centroide dev (estimació radi)
			height_avg = height_acc / it->indices.size();
			height2_avg = height2_acc / it->indices.size();
			height_dev = 2*sqrt(height2_avg - pow(height_avg,2)); //Height_avg+height_dev (estimació alçada)

			int ppa = (2*it->indices.size()) / (M_PI*pow(centroid_dev,2));
			if( ppa < BD_ppa)
			{
				indices_to_delete.emplace_back(j);
			}
			else
			{
				blob.height = height_avg+height_dev;
				blob.radius = centroid_dev;
				blob.x=x_avg;
				blob.z=z_avg;
				blob.ppa = ppa;
				current_blobs.emplace_back(blob);
			}
		}
		for(j=0; j<indices_to_delete.size(); ++j)
		{
			cluster_indices.erase(cluster_indices.begin()+indices_to_delete.at(j)-j);
		}

		if(cluster_indices.size() == 0)
		{
			reset=true;
			entry_count++;
			return;
		}

		// for(Blob blob : current_blobs)
		// {
		// 	std::cout<<"Blob height: "<<blob.height<<std::endl;
		// 	std::cout<<"Blob radius: "<<blob.radius<<std::endl;
		// 	std::cout<<"Blob x: "<<blob.x<<std::endl;
		// 	std::cout<<"Blob z: "<<blob.z<<std::endl;
		// }

		///MATCHING
		if(entry_count == 0 || reset) //First time initialize map
		{
			old_time = cur_stamp.toSec();
			old_points = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
			map_blobs=current_blobs;
			old_cluster_indices=cluster_indices;
			old_points = points->makeShared();
			
			for(Blob& blob_old : map_blobs)
			{
				int randNum = rand()%50; // [0, 50[
				blob_old.palette = rgba[randNum];
				blob_old.stability=0;
			}
			reset=false;
			entry_count++;
			return;
		}
		
		//Attempt matching
		delta_time = cur_stamp.toSec() - old_time;
		old_time = cur_stamp.toSec();
		matches.clear();
		int cur_size = cluster_indices.size();
		int old_size = old_cluster_indices.size();
		Eigen::MatrixXf blob_dist(cur_size,old_size);
		Eigen::MatrixXf blob_dist_xz(cur_size,old_size);
		float centroid_dist, radius_dist, height_dist;
		i=0;
		for(const Blob& blob : current_blobs)
		{
			j=0;
			for(const Blob& blob_old : map_blobs)
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
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, i+=1)
		{
			Eigen::MatrixXf::Index minIndexCol, minIndexRow;
			blob_dist.row(i).minCoeff(&minIndexRow);
			blob_dist.col(minIndexRow).minCoeff(&minIndexCol);
			if(i == minIndexCol) //MATCHED
			{
				// int randNum = rand()%50; // [0, 50[
				// current_blobs.at(i).palette = rgba[randNum];
				current_blobs.at(i).stability = map_blobs.at(minIndexRow).stability + 1;
				current_blobs.at(i).palette = map_blobs.at(minIndexRow).palette;
				matches.emplace_back(i,minIndexRow);
				matches_dist_xz.emplace_back(blob_dist_xz(i,minIndexRow));
			}
			else //NOT MATCHED
			{
				int randNum = rand()%50; // [0, 50[
				current_blobs.at(i).palette = rgba[randNum];
				current_blobs.at(i).stability = 0;
			}
		}

		j=0;
		for (std::vector<pcl::PointIndices>::const_iterator it = old_cluster_indices.begin(); it != old_cluster_indices.end(); ++it, j+=1)
		{
			for (const auto& idx : it->indices)
			{
				(*old_points)[idx].rgba=map_blobs.at(j).palette;
			}
		}

		// //DYNAMIC Blobs
		// indices_to_delete.clear();
		// std::vector<float> matches_dist_xz_small = matches_dist_xz;
		
		// for(i=0; i<matches.size(); ++i)
		// {
		// 	if(current_blobs.at(matches.at(i).first).radius > (2*BD_thres_radius))
		// 	{
		// 		indices_to_delete.emplace_back(i);		
		// 		// std::vector<pcl::PointIndices>::const_iterator it = old_cluster_indices.begin()+matches.at(i).second;
		// 		// for (const auto& idx : it->indices)
		// 		// {
		// 		// 	uint32_t rgba_color = ((uint8_t)255 << 16) + ((uint8_t)255 << 8) + (uint8_t)255; //white
		// 		// 	(*old_points)[idx].rgba= rgba_color;
		// 		// }
		// 	}
		// }
		// for(i=0; i<indices_to_delete.size(); ++i)
		// {
		// 	matches_dist_xz_small.erase(matches_dist_xz_small.begin()+indices_to_delete.at(i)-i);
		// }

		
		// float median_xbm_z_curr = util::calculateMedian(matches_dist_xz_small);
		// if(entry_count == 0) median_xz = median_xbm_z_curr;
		// median_xz = BD_alpha * median_xbm_z_curr + (1-BD_alpha) * median_xz;

		// if(median_xz > 0)
		// {
		// 	indices_to_delete.clear();
		// 	for(i=0; i<matches.size(); ++i)
		// 	{
		// 		// fp_<<matches_dist_xz.at(i)<<std::endl;
		// 		// if((abs(matches_dist_xz.at(i) - median_xz) > 0.035) && (abs(acos(x_avg*xi+z_avg*zi)) > M_PI/10)) //DYNAMIC
		// 		if(abs(matches_dist_xz.at(i) - median_xz) > BD_thres_xz) //DYNAMIC
		// 		{
		// 			indices_to_delete.emplace_back(i);
		// 			std::vector<pcl::PointIndices>::const_iterator it = old_cluster_indices.begin()+matches.at(i).second;
		// 			for (const auto& idx : it->indices)
		// 			{
		// 				uint32_t rgba_color = ((uint8_t)255 << 16) + ((uint8_t)255 << 8) + (uint8_t)255; //white
		// 				(*old_points)[idx].rgba= rgba_color;
		// 			}
		// 		} 
					
		// 	}
		// 	for(i=0; i<indices_to_delete.size(); ++i)
		// 	{
		// 		//old_cluster_indices.erase(old_cluster_indices.begin()+matches.at(indices_to_delete.at(i)).second-i);
		// 		matches.erase(matches.begin()+indices_to_delete.at(i)-i);
		// 		matches_dist_xz.erase(matches_dist_xz.begin()+indices_to_delete.at(i)-i);
		// 	}
		// }

		/// PRE-PUBLISH
		size_t sm = matches.size();
		size_t sp_old = (*old_points).size();
		size_t sp_cur = (*points).size();
		std::vector<uint8_t> stability(sm);
		std::vector<unsigned int> ppa(sm);
		std::vector<float> bm_x_old(sm), bm_z_old(sm), bm_radius_old(sm), bm_height_old(sm);
		std::vector<float> bm_x_cur(sm), bm_z_cur(sm), bm_radius_cur(sm), bm_height_cur(sm);
		std::vector<float> bp_x_old(sp_old), bp_y_old(sp_old), bp_z_old(sp_old);
		std::vector<float> bp_x_cur(sp_cur), bp_y_cur(sp_cur), bp_z_cur(sp_cur);
		std::vector<unsigned int> bp_s_old(sp_old), bp_s_cur(sp_cur);
		unsigned int j_old = 0, j_cur = 0;
		for(i=0; i<matches.size(); ++i)
		{
			ppa[i] = current_blobs.at(matches.at(i).first).ppa;
			stability[i] = current_blobs.at(matches.at(i).first).stability;
			bm_x_cur[i] = current_blobs.at(matches.at(i).first).x;
			bm_z_cur[i] = current_blobs.at(matches.at(i).first).z;
			bm_radius_cur[i] = current_blobs.at(matches.at(i).first).radius;
			bm_height_cur[i] = current_blobs.at(matches.at(i).first).height;
			bm_x_old[i] = map_blobs.at(matches.at(i).second).x;
			bm_z_old[i] = map_blobs.at(matches.at(i).second).z;
			bm_radius_old[i] = map_blobs.at(matches.at(i).second).radius;
			bm_height_old[i] = map_blobs.at(matches.at(i).second).height;

			for (const auto& idx : old_cluster_indices.at(matches.at(i).second).indices)
			{
				bp_x_old[j_old] = (*old_points)[idx].x;
				bp_y_old[j_old] = (*old_points)[idx].y;
				bp_z_old[j_old] = (*old_points)[idx].z;
				bp_s_old[j_old] = i;
				j_old+=1;
			}

			for (const auto& idx : cluster_indices.at(matches.at(i).first).indices)
			{
				bp_x_cur[j_cur] = (*points)[idx].x;
				bp_y_cur[j_cur] = (*points)[idx].y;
				bp_z_cur[j_cur] = (*points)[idx].z;
				bp_s_cur[j_cur] = i;
				j_cur+=1;
			}
		}

		/// PUBLISH
		/// - Cloud Filtered Blobs
		sensor_msgs::PointCloud2 msg_pcd;
		pcl::toROSMsg(*old_points, msg_pcd);
		msg_pcd.header.frame_id = cloud_filtered_blobs_frame_id;
		msg_pcd.header.stamp = cf_msg_ptr->header.stamp;
		cloud_blobs_pub_.publish(msg_pcd);

		/// - Blob Params
		terreslam::BlobMatchesPtr bm_msg_ptr(new terreslam::BlobMatches);
		bm_msg_ptr->header.frame_id = blob_matches_frame_id;
		bm_msg_ptr->header.stamp = cf_msg_ptr->header.stamp;
		bm_msg_ptr->delta_time = delta_time;
		bm_msg_ptr->stability = stability;
		bm_msg_ptr->ppa = ppa;
		bm_msg_ptr->x_old = bm_x_old;
		bm_msg_ptr->z_old = bm_z_old;
		bm_msg_ptr->radius_old = bm_radius_old;
		bm_msg_ptr->height_old = bm_height_old;
		bm_msg_ptr->x_cur = bm_x_cur;
		bm_msg_ptr->z_cur = bm_z_cur;
		bm_msg_ptr->radius_cur = bm_radius_cur;
		bm_msg_ptr->height_cur = bm_height_cur;
		blob_matches_pub_.publish(bm_msg_ptr);

		/// - Blob Points
		terreslam::BlobPointsPtr bp_msg_ptr(new terreslam::BlobPoints);
		bp_msg_ptr->header.frame_id = blob_points_frame_id;
		bp_msg_ptr->header.stamp = cf_msg_ptr->header.stamp;
		bp_msg_ptr->x_old = bp_x_old;
		bp_msg_ptr->y_old = bp_y_old;
		bp_msg_ptr->z_old = bp_z_old;
		bp_msg_ptr->s_old = bp_s_old;
		bp_msg_ptr->x_cur = bp_x_cur;
		bp_msg_ptr->y_cur = bp_y_cur;
		bp_msg_ptr->z_cur = bp_z_cur;
		bp_msg_ptr->s_cur = bp_s_cur;
		blob_points_pub_.publish(bp_msg_ptr);

		//Update old data
		map_blobs=current_blobs;
		old_cluster_indices=cluster_indices;
		old_points->clear(); //Necessary??
		old_points = points->makeShared();
		
		entry_count++;

		// util::tick_high_resolution(start_t, tick, elapsed);
		// util::printElapsed(elapsed, "Callback blob detector: ");
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
	ros::Publisher cloud_blobs_pub_;
	ros::Publisher blob_matches_pub_;
	ros::Publisher blob_points_pub_;
	ros::Subscriber cloud_filtered_sub_;

	/// Chrono timmings
	std::vector<double> elapsed;

	/// Blobs
	unsigned int i,j;
	std::vector<Blob> current_blobs;
	std::vector<Blob> map_blobs; //Per cadascun hi ha un centroid, radi, height
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr old_points;
	std::vector<pcl::PointIndices> old_cluster_indices;
	//frame discutir
	std::vector<std::pair<int,int>> matches;
	std::vector<float> matches_dist_xz;
	float median_xz=0;
	bool reset=false;

	uint32_t rgba[50];

	///Timming
	double delta_time = -1;
	double old_time = -1;

	/// Logging
	std::ofstream fp_;
	std::string logs_path_;

};

PLUGINLIB_EXPORT_CLASS(terreslam::BlobDetectorNodelet, nodelet::Nodelet);

}