/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-21 11:44:08
 */

#include <iostream>

#include "terreslam/features/plane_detector.h"

namespace terreslam
{

void Cell::computeAttribute()
{
	if(isEmpty)
		return;
	// compute the expectation of position and color
	for(std::vector<Point>::iterator it=points.begin();it!=points.end();++it)
	{
		avg_pps=avg_pps+it->pps;
		avg_rgb=avg_rgb+it->rgb;
	}
	avg_pps=avg_pps*(1.0/points.size());
	avg_rgb=avg_rgb*(1.0/points.size());
	
	Eigen::Vector3d tmp;
	// compute the covariance of position and color
	for(std::vector<Point>::iterator it=points.begin();it!=points.end();++it)
	{
		tmp=it->pps;
		tmp-=avg_pps;
		cov_pps+=tmp*tmp.transpose();
		tmp=it->rgb;
		tmp-=avg_rgb;
		cov_rgb+=tmp*tmp.transpose();
	}
	// biased estimation of the covariance;
	cov_pps=cov_pps*(1.0/points.size());
	cov_rgb=cov_rgb*(1.0/points.size());
}

void Cells_bottom::push_point(Point point_tmp, int idx)
{
	// std::cout<<"push point"<<std::endl;
	int theta=(point_tmp.pps[0])/delta_theta;
	int phy=(point_tmp.pps[1]+M_PI)/delta_phy;
	int d=(point_tmp.pps[2])/delta_d;
	// std::cout<<theta<<", "<<phy<<", "<<d<<std::endl;
	if(theta>=bins_theta)
		theta=bins_theta-1;
	if(phy>=bins_phy)
		phy=bins_phy-1;
	if(d>=bins_d)
		d=bins_d-1;
	int i=index(d,theta,phy);
	cells[i]->points.push_back(point_tmp);
	cells[i]->indices->indices.push_back(idx);
	cells[i]->isEmpty=false;
}

void Cells_bottom::computeCellAttributes()
{
	for(size_t i=0;i<cells.size();i++)
	{
		if(cells[i]->isEmpty)
			continue;
		cells[i]->computeAttribute();
	}
}

void Cells_bottom::SortCells()
{
	Sorted_Cell tmp_sort;
	sorted_cells.clear();
	for(int i=0;i<cells.size();i++)
	{
		tmp_sort.index=i;
		tmp_sort.num_point=cells[i]->points.size();
		sorted_cells.push_back(tmp_sort);
	}
	std::sort(sorted_cells.begin(), sorted_cells.end());
}

bool PlaneDetector::loadPoints(Scan *scan)
{
			if(scan->points()->empty() || scan->normals()->empty() || scan->pixels()->empty())
		{
			std::cerr<<"load points(), normals() and pixels() before this."<<std::endl;
			return false;
		}

		cells_bottom_->clear();

		computeRotationPCA(scan);
		if(debug_)
		{
			fp_<<"RotationPCA - "<<std::endl;
			fp_<<Rotation_PCA<<std::endl;
		}

		Point point_tmp;
		for(int i=0;i<scan->points()->size();i++)
		{
			if(std::isnan(scan->normals()->at(i).normal_x) && 
				 std::isnan(scan->normals()->at(i).normal_y) && 
				 std::isnan(scan->normals()->at(i).normal_z))
				continue;
			if(fabs(scan->points()->at(i).x)<1e-10 && 
				 fabs(scan->points()->at(i).y)<1e-10 && 
				 fabs(scan->points()->at(i).z)<1e-10)
				continue;
			// coordinate in RGB [0,1]
			point_tmp.rgb[0]=(double)scan->points()->at(i).r/255.0;
			point_tmp.rgb[1]=(double)scan->points()->at(i).g/255.0;
			point_tmp.rgb[2]=(double)scan->points()->at(i).b/255.0;
			// coordinate in the camera coordinate system
			point_tmp.xyz[0]=scan->points()->at(i).x;
			point_tmp.xyz[1]=scan->points()->at(i).y;
			point_tmp.xyz[2]=scan->points()->at(i).z;
			// local plane normal
			point_tmp.normal[0]=scan->normals()->at(i).normal_x;
			point_tmp.normal[1]=scan->normals()->at(i).normal_y;
			point_tmp.normal[2]=scan->normals()->at(i).normal_z;
			// coordinate in the PPS, after Rotation_PCA
			point_tmp.Cartesian2PPS(Rotation_PCA);
			// pixel coordinate
			point_tmp.u=scan->pixels()->at(i).x; // i/width
			point_tmp.v=scan->pixels()->at(i).y; // i%height
			// push the point into the corresponding cells
			cells_bottom_->push_point(point_tmp,i);
		}

		cells_bottom_->computeCellAttributes();

		cells_bottom_->SortCells();

//		fp_.close();
		return true;
}

bool PlaneDetector::computeNormals(Scan *scan, bool use_normal_integral)
{
	if(use_normal_integral)
	{
		/// Generate the normal_cloud
		/// More methods --> AVERAGE_3D_GRADIENT; AVERAGE_DEPTH_CHANGE; COVARIANCE_MATRIX
		pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne_integral;
		ne_integral.setNormalEstimationMethod(pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA,pcl::Normal>::AVERAGE_DEPTH_CHANGE);
		ne_integral.setDepthDependentSmoothing(true);
		ne_integral.setNormalSmoothingSize(40.0);
		ne_integral.setInputCloud(scan->points());
		ne_integral.compute(*scan->normals());
	}
	else
	{
		pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
		pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree;
		ne.setInputCloud(scan->points());
		tree=pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGBA>());
		ne.setSearchMethod(tree);
		ne.setRadiusSearch(1);
		ne.compute(*scan->normals());
	}
}

void PlaneDetector::detectPlanes(Scan *scan, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rest, bool use_normal_integral)
{
	fp_.open(logs_path_, std::ios::app);
	if(debug_)
	{
		fp_<<"*****************extractPlanes**************************************"<<std::endl;
	}

	computeNormals(scan, use_normal_integral);
	loadPoints(scan);

	// pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
	// vis->removeAllPointClouds();
	// if (!vis->updatePointCloud(scan->points(),"scan")) vis->addPointCloud(scan->points(),"scan");

	pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
	bool enough_plane = false;
	Eigen::Vector3d tmp_vec3d;
	pcl::PointXYZRGBA tmp_point_pcl;
	pcl::Normal tmp_normal_pcl;
	Plane *tmp_plane;
	Point tmp_point;
	bool have_same_plane=false;
	bool had_parallel_plane=false;

	// std::cout<<"cells_bottom "<<cells_bottom_->size()<<std::endl;
	// pcl::PointCloud<pcl::PointXYZRGBA>::Ptr allplane (new pcl::PointCloud<pcl::PointXYZRGBA>);
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_contain_plane (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_contain_plane_normal (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXY>::Ptr cloud_contain_plane_image (new pcl::PointCloud<pcl::PointXY>);
	std::vector<int> indices_cloud_contain_plane;
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::Normal>::Ptr plane_normal (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXY>::Ptr plane_image (new pcl::PointCloud<pcl::PointXY>);
	std::vector<int> indices_plane;

	// pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rest (new pcl::PointCloud<pcl::PointXYZRGBA>);
	// std::vector<int> indices_rest;
	
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normal;
	pcl::ExtractIndices<pcl::PointXY> extract_image;

	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(120);
	seg.setDistanceThreshold(0.02); //0.01

	// std::cout<<"cells_bottom "<<cells_bottom_->size()<<std::endl;

	/// Find the cell with the most points inside
	std::vector<Sorted_Cell>::iterator iter_sorted_cells=cells_bottom_->getHighestCell();
	int maxdir = 0;
	int maxnum = 0;
	maxdir=iter_sorted_cells->index;
	maxnum=iter_sorted_cells->num_point;
	if(debug_)
	{
		fp_<<"maxdir = "<<maxdir<<", maxnum = "<<maxnum<<std::endl;
	}

	cloud_rest->clear();
	// scan->observed_planes.clear(); //ivan: I think that should be deleted
	
	/// Extracting planes
	while ( maxnum > min_plane_size_ && enough_plane == false )
	{
		/// Save points in cells[maxdir] to cloud_contain_plane
		extract.setInputCloud (scan->points());
		extract.setIndices (cells_bottom_->getCell(maxdir)->indices);
		extract.setNegative (false);
		extract.filter (*cloud_contain_plane);

		indices_cloud_contain_plane=cells_bottom_->getCell(maxdir)->indices->indices;

		// cout<<"vis - cloud_contain_plane"<<endl;
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color1(cloud_contain_plane,255,0,0);
		// if (!vis->updatePointCloud(cloud_contain_plane,color1,"tmp")) vis->addPointCloud(cloud_contain_plane,color1,"tmp");
		// vis->spin();

		/// The corresponding normals() to cloud_contain_plane_normal
		extract_normal.setInputCloud (scan->normals());
		extract_normal.setIndices (cells_bottom_->getCell(maxdir)->indices);
		extract_normal.setNegative (false);
		extract_normal.filter (*cloud_contain_plane_normal);
		/// The corresponding pixels() pixel to cloud_contain_plane_image
		extract_image.setInputCloud (scan->pixels());
		extract_image.setIndices (cells_bottom_->getCell(maxdir)->indices);
		extract_image.setNegative (false);
		extract_image.filter (*cloud_contain_plane_image);
		/// Enough points in the cells[maxdir]
		if(debug_)
		{
			fp_<<"================================================="<<std::endl;
			fp_<<"maxdir="<<maxdir<<", maxnum="<<maxnum<<std::endl;
		}
		if (cloud_contain_plane->size() > min_plane_size_)
		{
			/// Estimate plane parameters using cells[maxdir]
			seg.setInputCloud(cloud_contain_plane);
			seg.segment(*inliers_plane, *coefficients_plane);
			/// Plane equation: ax+by+cz+d=0
			/// Make the plane normal point to the origin
			unifyPlaneDir(coefficients_plane);

			/// Plane
			/// - extracted plane
			/// - points on plane in the camera coordinate system
			extract.setInputCloud (cloud_contain_plane);
			extract.setIndices (inliers_plane);
			extract.setNegative (false);
			extract.filter (*plane); // plane extracted in the maxdir direction
			extract.setNegative (true);
			extract.filter (*cloud_contain_plane);

			extractIndices(inliers_plane->indices,indices_cloud_contain_plane,indices_plane);

			// cout<<"vis - inliers_plane"<<endl;
			// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color2(plane,255,0,0);
			// if (!vis->updatePointCloud(plane,color2,"tmp")) vis->addPointCloud(plane,color2,"tmp");
			// vis->spin();

			/// Plane_normal
			/// - extracted plane
			/// - local normal of each point on plane
			extract_normal.setInputCloud (cloud_contain_plane_normal);
			extract_normal.setIndices (inliers_plane);
			extract_normal.setNegative (false);
			extract_normal.filter (*plane_normal);
			extract_normal.setNegative (true);
			extract_normal.filter (*cloud_contain_plane_normal);

			/// Plane_image
			/// - extracted plane
			/// - pixel coordinate corresponding to each point on plane
			extract_image.setInputCloud (cloud_contain_plane_image);
			extract_image.setIndices (inliers_plane);
			extract_image.setNegative (false);
			extract_image.filter (*plane_image);
			extract_image.setNegative (true);
			extract_image.filter (*cloud_contain_plane_image);

			have_same_plane=false;

			if(debug_)
			{
				fp_<<"Plane size="<<plane->size()<<std::endl;
			}

			if(plane->size()<=min_plane_size_)
			{
				/// if the extracted plane is not large enough
				/// then the following "while" will not be activated
				*cloud_rest=*cloud_rest+*cloud_contain_plane;
				*cloud_rest=*cloud_rest+*plane;
			}

			while(plane->size() > min_plane_size_ && enough_plane == false)
			{
				if (scan->sizeFeature() < max_plane_)
				{
					/// tmp_plane
					/// - allocate a new plane feaure
					/// - normal, d: from the extraction method
					/// - points, num_points
					/// - avg_pps, cov_pps, avg_rgb and cov_rgb: computed from points
					tmp_plane=new Plane;
					unifyPlaneDir(coefficients_plane);
					// tmp_plane->points.clear();
					tmp_plane->n(0)=coefficients_plane->values[0];
					tmp_plane->n(1)=coefficients_plane->values[1];
					tmp_plane->n(2)=coefficients_plane->values[2];
					tmp_plane->d=coefficients_plane->values[3];
					tmp_plane->indices=indices_plane;
					tmp_plane->ptr_points=scan->points();
					tmp_plane->ptr_normals=scan->normals();
					tmp_plane->ptr_pixels=scan->pixels();

					// pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
					// pcl::copyPointCloud(*tmp_plane->ptr_points,tmp_plane->indices,*tmp);

					// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> color(tmp,0,255,0);
					// if (!vis->updatePointCloud (tmp, color, "plane")) vis->addPointCloud (tmp, color, "plane");

					// vis->spin();

					if(debug_)
					{
						fp_<<"allocate a new plane - "<<std::endl;
						fp_<<"\tnormal="<<tmp_plane->n.transpose()<<std::endl;
						fp_<<"\td="<<tmp_plane->d<<std::endl;
						fp_<<"\tplane size - "<<tmp_plane->indices.size()<<std::endl;
					}
				
					/// if "planes" are not empty, i.e., there are already extracted planes
					/// test if current plane is the same with any one in "planes"
					/// if so, fuse the same planes
					if(scan->sizeFeature()>0)
					{
						// for(int i_plane=0;i_plane<scan->sizeFeature();i_plane++)
						for(iterFeature it=scan->beginFeature();it!=scan->endFeature();it++)
						{
							if(it->second->Type()!=PLANE) continue;
							// angle(n1,n2)<thres_angle_, |d1-d2|<thres_dist_, delta_color<thres_color;
							double tmp_cos_angle=tmp_plane->n.transpose()*it->second->plane()->n;
							if(tmp_cos_angle>0.9999)
							{
								tmp_cos_angle=1.0;
							}
							// Eigen::Vector3d tmp_delta_rgb=tmp_plane->centroid_color-scan->observed_planes[i_plane]->centroid_color;
							if(debug_)
							{
								fp_<<"\tsimilarity with "<<it->second->ID()<<" - "<<tmp_cos_angle<<", "<<acos(tmp_cos_angle)<<", "
												<<fabs(tmp_plane->d-it->second->plane()->d)
											 	// <<", "<<tmp_delta_rgb.norm()
											<<std::endl;
							}
							if(acos(tmp_cos_angle)<thres_angle_ 
									&& fabs(tmp_plane->d-it->second->plane()->d)<thres_dist_) 
								 	// && tmp_delta_rgb.norm()<thres_color)
							{
								have_same_plane=true;
								fusePlanes(tmp_plane,it->second->plane());
								// tmp_plane->release();
								delete tmp_plane;
								// *allplane = *plane + *allplane;
								if(debug_)
								{
									fp_<<"\tsame with "<<it->second->ID()<<", after fusion:"<<std::endl;
									fp_<<"\t\tnormal="<<it->second->plane()->n.transpose()<<std::endl;
									fp_<<"\t\td="<<it->second->plane()->d<<std::endl;
									fp_<<"\t\tsize="<<it->second->plane()->indices.size()<<std::endl;
								}
								break;
							}
						}
					}

					/// if no same plane, add the tmp_plane to "planes"
					if(have_same_plane == false)
					{
						/// no similar planes
						/// save current plane tmp_plane to planes
						// tmp_plane->id="plane_"+std::to_string(scan->observed_planes.size());
						Feature *feature=new Feature(tmp_plane);
						scan->addFeature(feature);
						// *allplane = *plane + *allplane;
						if(debug_)
						{
							fp_<<"the "<<scan->sizeFeature()-1<<"th plane:"<<std::endl;
							fp_<<"\tnormal="<<tmp_plane->n.transpose()<<std::endl;
							fp_<<"\td="<<tmp_plane->d<<std::endl;
							fp_<<"\tplane size - "<<tmp_plane->indices.size()<<std::endl;
						}
					}

					/// if there are a lot of points left in the cloud_contain_plane
					/// then more planes may be extracted
					if (cloud_contain_plane->size() > min_plane_size_)
					{
						seg.setInputCloud(cloud_contain_plane);
						seg.segment(*inliers_plane, *coefficients_plane);

						extract.setInputCloud (cloud_contain_plane);
						extract.setIndices (inliers_plane);
						extract.setNegative (false);
						extract.filter (*plane);
						extract.setNegative (true);
						extract.filter (*cloud_contain_plane);

						extractIndices(inliers_plane->indices,indices_cloud_contain_plane,indices_plane);

						extract_normal.setInputCloud (cloud_contain_plane_normal);
						extract_normal.setIndices (inliers_plane);
						extract_normal.setNegative (false);
						extract_normal.filter (*plane_normal);
						extract_normal.setNegative (true);
						extract_normal.filter (*cloud_contain_plane_normal);

						extract_image.setInputCloud (cloud_contain_plane_image);
						extract_image.setIndices (inliers_plane);
						extract_image.setNegative (false);
						extract_image.filter (*plane_image);
						extract_image.setNegative (true);
						extract_image.filter (*cloud_contain_plane_image);

						have_same_plane=false;

						if (plane->size() > min_plane_size_)
						{
							// had_parallel_plane = true;
						}
						else
						{
							*cloud_rest = *cloud_rest + *plane;
							*cloud_rest = *cloud_rest + *cloud_contain_plane;
						}
					}
					else
					{
						*cloud_rest = *cloud_rest + *cloud_contain_plane;
						plane->clear();
					}
				}
				else
				{
					enough_plane = true;
				}		
			}
		}
		else
		{
			*cloud_rest=*cloud_rest+*cloud_contain_plane;
		}
		if( enough_plane == false )
		{
			iter_sorted_cells--;
			maxdir=iter_sorted_cells->index;
			maxnum=iter_sorted_cells->num_point;
		}
	}

	if(debug_)
	{
		fp_<<"extracted "<<scan->sizeFeature()<<" planes"<<std::endl;
		for(iterFeature it=scan->beginFeature();it!=scan->endFeature();it++)
		{
			fp_<<"\tnormal="<<it->second->plane()->n.transpose();
			fp_<<", d="<<it->second->plane()->d<<std::endl;
		}
	}

	fp_.close();

}

void PlaneDetector::computeRotationPCA(Scan *scan)
	{
		Eigen::Vector3d tmp_vec3d;
		Eigen::Matrix3d tmp_mat3d=Eigen::Matrix3d::Zero();
		int tmp_count=0;
		double tmp_max=0,tmp_min=99999;
		Eigen::Vector3d x,y,z;

		// find the direction with least normal vectors
		// set this direction as the z axis
		for(int i=0;i<scan->normals()->size();i++)
		{
			if(std::isnan(scan->normals()->at(i).normal_x) && std::isnan(scan->normals()->at(i).normal_y) && std::isnan(scan->normals()->at(i).normal_z))
				continue;
			else
			{
				tmp_vec3d(0)=scan->normals()->at(i).normal_x;
				tmp_vec3d(1)=scan->normals()->at(i).normal_y;
				tmp_vec3d(2)=scan->normals()->at(i).normal_z;
				tmp_mat3d+=tmp_vec3d*tmp_vec3d.transpose();
				tmp_count++;
			}
		}
		tmp_mat3d=tmp_mat3d*(1.0/tmp_count);
		Eigen::EigenSolver<Eigen::Matrix3d> es(tmp_mat3d);
		for(int i=0;i<3;i++)
		{
			if(es.eigenvalues().real()[i]>tmp_max)
			{
				tmp_max=es.eigenvalues().real()[i];
				y=es.eigenvectors().real().block<3,1>(0,i);
			}
			if(es.eigenvalues().real()[i]<tmp_min)
			{
				tmp_min=es.eigenvalues().real()[i];
				z=es.eigenvectors().real().block<3,1>(0,i);
			}
		}
		x=y.cross(z);
		x.normalize();
		// Rotation_PCA
		// - rotation from the original coordinate to the coordinate where 
		// - the z axis pointing to the direction with least normals()
		Rotation_PCA.block<1,3>(0,0)=x.transpose();
		Rotation_PCA.block<1,3>(1,0)=y.transpose();
		Rotation_PCA.block<1,3>(2,0)=z.transpose();
	}

void PlaneDetector::unifyPlaneDir(pcl::ModelCoefficients::Ptr plane)
{
	/// unifyPlaneDir
	/// - make the plane normal point to the origin
	
	double a=plane->values[0];
	double b=plane->values[1];
	double c=plane->values[2];
	double d=plane->values[3];
	bool flag_reverse;
	if ( abs(a)>=abs(b) && abs(a)>=abs(c) )
	{
		float x = (b+c-d)/a;
		if ( (a*(-x)+b+c)>0 )
			flag_reverse=false;
		else
			flag_reverse=true;
	}
	if ( abs(b)>=abs(a) && abs(b)>=abs(c) )
	{
		float y = (a+c-d)/b;
		if ( (b*(-y)+a+c)>0 )
			flag_reverse=false;
		else
			flag_reverse=true;
	}
	if ( abs(c)>=abs(b) && abs(c)>=abs(a) )
	{
		float z = (a+b-d)/c;
		if ( (c*(-z)+b+a)>0 )
			flag_reverse=false;
		else
			flag_reverse=true;
	}
	if(flag_reverse)
	{
		plane->values[0]=-plane->values[0];
		plane->values[1]=-plane->values[1];
		plane->values[2]=-plane->values[2];
		plane->values[3]=-plane->values[3];
	}
}

void PlaneDetector::fusePlanes(Plane *tmp_plane, Plane *fuse)
{
	for(std::vector<int>::iterator it=tmp_plane->indices.begin();it!=tmp_plane->indices.end();++it)
	{
		fuse->indices.push_back(*it);
	}
}

}