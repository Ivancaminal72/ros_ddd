/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-21 11:44:23
 */

#pragma once

#include "terreslam/utils/util_types.h"
#include "terreslam/utils/util_map.h"

namespace terreslam
{

struct Point
{
	// xyz in meters
	Eigen::Vector3d xyz, normal;

	// coordinate in the PPS for the local plane parameters (after Rotation_PCA)
	Eigen::Vector3d pps;
	void Cartesian2PPS(Eigen::Matrix3d Rotation_PCA=Eigen::Matrix3d::Identity())
	{
		Eigen::Vector3d tmp_vec3d;
		tmp_vec3d=Rotation_PCA*normal;
		if(tmp_vec3d(2)>1.0-1e-20)
		{
			tmp_vec3d(2)=1.0;
		}
		if(tmp_vec3d(2)<-1.0+1e-20)
		{
			tmp_vec3d(2)=-1.0;
		}
		pps(0)=acos(tmp_vec3d(2));
		pps(1)=atan2(tmp_vec3d(1),tmp_vec3d(0));
		pps(2)=-normal.dot(xyz);

		// if(pps(0)>M_PI)
		// {
		// 	pps(0)=M_PI*2-pps(0);
		// }
	}

	// rgb in [0,1]^3
	Eigen::Vector3d rgb;

	// u,v: pixel coordinate
	int u,v;

	// // cov of the point
	// // measurement uncertainty
	// Eigen::Matrix3d cov, cov_inv; 
	// void compute_cov(CameraIntrinsic& cam)
	// {
	// 	double sigma_d=cam.m_fp*cam.sigma_disparity*xyz(2)*xyz(2); //m
	// 	double d_fu=xyz(2)/cam.fx;
	// 	double d_fv=xyz(2)/cam.fy;
	// 	double x_d=xyz(0)/xyz(2);
	// 	double y_d=xyz(1)/xyz(2);
	// 	cov(0,0)=d_fu*d_fu*cam.sigma_u*cam.sigma_u+x_d*x_d*sigma_d*sigma_d;
	// 	cov(0,1)=x_d*y_d*sigma_d*sigma_d;
	// 	cov(0,2)=x_d*sigma_d*sigma_d;
	// 	cov(1,0)=cov(0,1);
	// 	cov(1,1)=d_fv*d_fv*cam.sigma_v*cam.sigma_v+y_d*y_d*sigma_d*sigma_d;
	// 	cov(1,2)=y_d*sigma_d*sigma_d;
	// 	cov(2,0)=cov(0,2);
	// 	cov(2,1)=cov(1,2);
	// 	cov(2,2)=sigma_d*sigma_d;
	// 	cov_inv=cov.inverse();
	// }

	// // weight in the plane fitting;
	// // weight = pln_n^T * cov_inv * pln_n;
	// // weight_angle = [p_pi/(pln_n^T*p_pi)]^T * cov_inv * [p_pi/(pln_n^T*p_pi)];
	// double weight;
};

struct Cell
{
	Cell()
	{
		isEmpty=true;
		isBottom=true;
		avg_pps.setZero(3);
		avg_rgb.setZero(3);
		cov_pps.setZero(3,3);
		cov_rgb.setZero(3,3);
		indices = boost::make_shared<pcl::PointIndices>();
	}

	~Cell()
	{
		std::vector<Point> tmp;
		tmp.swap(points);
	}

	Eigen::Vector3d avg_pps, avg_rgb;
	Eigen::Matrix3d cov_pps, cov_rgb;
	void computeAttribute();
	
	bool isEmpty;
	bool isBottom;

	std::vector<Point> points;

	// indices w.r.t. point_cloud and normal_cloud
	// to index the cell points in the point_cloud
	pcl::PointIndices::Ptr indices;

	void clear() 
	{
		points.clear();
		indices->indices.clear();
		isEmpty=true;
		isBottom=true;
		avg_pps.setZero(3);
		avg_rgb.setZero(3);
		cov_pps.setZero(3,3);
		cov_rgb.setZero(3,3);
	}

	// the followings are used in the sting
	int layer;
	bool relevant;
	Cell *father;
	Cell *child[8];
};

struct Sorted_Cell
{
	// Sorted_Cell
	// - store the index of the cells
	// - used to sort the cells in Cells_bottom
	Sorted_Cell(){}
	bool operator < (const Sorted_Cell &m)const
	{
		return num_point < m.num_point;
	}
	int index;
	int num_point;
};

class Cells_bottom
{
public:

	Cells_bottom(int theta, int phy, int d, double max_depth) 
	{
		bins_theta=theta;
		bins_phy=phy;
		bins_d=d;
		delta_theta=M_PI/bins_theta;
		delta_phy=M_PI*2/bins_phy;
		delta_d=max_depth/bins_d;
		// cells are allocated here;
		cells.resize(bins_theta*bins_phy*bins_d);
		for(size_t i=0;i<cells.size();i++)
			cells[i]=new Cell;
	}

	~Cells_bottom()
	{
		for(size_t i=0;i<cells.size();i++)
		{
			delete cells[i];
		}
		std::vector<Cell*> tmp;
		tmp.swap(cells);
		std::vector<Sorted_Cell> tmp2;
		tmp2.swap(sorted_cells);
	}

	void clear()
	{
		sorted_cells.clear();
		for(size_t i=0;i<cells.size();i++)
		{
			cells[i]->clear();
		}
		// cells.clear();
	}

	// push_point
	// - push the point point_tmp into the corresponding cell
	// - idx: the index w.r.t. the point_cloud
	void push_point(Point point_tmp, int idx);

	void computeCellAttributes();

	// SortCells
	// - sort the cells according to the number of inside points
	// - store the sorting result in the sorted_cells
	void SortCells();

	Cell* getCell(int i) {return cells[i];}
	Cell* getCell(int d, int theta, int phy) {return cells[index(d,theta,phy)];}

	int size() {return cells.size();}

	std::vector<Sorted_Cell>::iterator getHighestCell() {return sorted_cells.end()-1;}

private:

	int index(int d, int theta, int phy) 
	{ return d*bins_theta*bins_phy+theta*bins_phy+phy; }

	int bins_theta,bins_phy,bins_d;
	double delta_theta,delta_phy,delta_d;

	std::vector<Cell*> cells;
	std::vector<Sorted_Cell> sorted_cells;
};

class PlaneDetector
{
public:
	PlaneDetector(
		bool debug,
		int theta,
		int phi,
		int d,
		int max_plane,
		int min_plane_size,
		double thres_angle,
		double thres_dist,
		double max_depth,
		std::string logs_dir)
	:
		logs_path_(logs_dir + "/log_plane_detector.txt"),
		debug_(debug),
		max_plane_(max_plane),
		min_plane_size_(min_plane_size),
		thres_angle_(thres_angle),
		thres_dist_(thres_dist)
	{
		cells_bottom_ = new Cells_bottom(theta,phi,d, max_depth);
		thres_angle_ *= M_PI/180.0;
		remove(logs_path_.c_str());
	}

	~PlaneDetector()
	{
		delete cells_bottom_;
	}

	void setDebug(bool d) {debug_=d;}
	void detectPlanes(Scan *scan);

private:

	std::string logs_path_;
	bool debug_;
	std::ofstream fp_;

	double maxdist_point2plane;
	int max_plane_;
	int min_plane_size_;
	double thres_angle_, thres_dist_, thres_color_;

	Cells_bottom *cells_bottom_;

	Eigen::Matrix3d Rotation_PCA;

	bool loadPoints(Scan *scan);

	void computeRotationPCA(Scan *scan);

	void unifyPlaneDir(pcl::ModelCoefficients::Ptr plane);

	void fusePlanes(Plane *cur, Plane *fuse);

	void extractIndices(const std::vector<int> &indices_extr, std::vector<int> &indices, std::vector<int> &indices_plane)
	{
		indices_plane.clear();
		for(int i=0;i<indices_extr.size();i++)
		{
			int idx=indices_extr[i];
			indices_plane.push_back(indices[idx]);
			indices[idx]=-1;
		}
		std::vector<int> indices_rest;
		for(int i=0;i<indices.size();i++)
		{
			if(indices[i]==-1) continue;
			indices_rest.push_back(indices[i]);
		}
		indices.clear();
		indices=indices_rest;
	}

};
}