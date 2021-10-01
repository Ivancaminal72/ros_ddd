/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-04-23 10:50:34
 */

#include <iostream>

#include "terreslam/processings/dd_keypoint_processor.h"

namespace terreslam
{

std::vector<cv::DMatch> matchTwoImage(const cv::Mat& query_desc, const cv::Mat& train_desc)
{
	auto matcher = cv::DescriptorMatcher::create("BruteForce");
	std::vector<cv::DMatch> matches;
	matcher->match(query_desc, train_desc, matches);
	return matches;
}

void nonZeroWindowContourLookUp(int& v, int& u, const int& ws, const cv::Mat& img)
{
	assert(ws > 0);
	int j,i,vv,uu;
	uu = u+ws;
	for(j=-ws+1; j<ws; ++j)
	{
		vv = v+j;
		if(uu >= img.cols || vv >= img.rows || uu < 0 || vv < 0) break;
		if(img.at<ushort>(vv,uu) != 0) {v=vv; u=uu; return;}
	}
	uu = u-ws;
	for(j=-ws+1; j<ws; ++j)
	{
		vv = v+j;
		if(uu >= img.cols || vv >= img.rows || uu < 0 || vv < 0) break;
		if(img.at<ushort>(vv,uu) != 0) {v=vv; u=uu; return;}
	}
	vv = v-ws;
	for(i=-ws; i<=ws; ++i)
	{
		uu = u+i;
		if(uu >= img.cols || vv >= img.rows || uu < 0 || vv < 0) break;
		if(img.at<ushort>(vv,uu) != 0) {v=vv; u=uu; return;}
	}
	vv = v+ws;
	for(i=-ws; i<=ws; ++i)
	{
		uu = u+i;
		if(uu >= img.cols || vv >= img.rows || uu < 0 || vv < 0) break;
		if(img.at<ushort>(vv,uu) != 0) {v=vv; u=uu; return;}
	}
}

std::vector<cv::Point3f> backprojectKeypoints(const std::vector<cv::KeyPoint>& kpts, 
																							const Eigen::Matrix4d& P_inv,
																							const double& depth_scale, 
																							const cv::Mat& img_depth, 
																							const int& max_ws)
{
	std::vector<cv::Point3f> kpts_bkpj;
	Eigen::Vector4d point_eigen, point_eigen_bkpj;
	double depth_yx;
	int u,v,ws;
	for (auto& kpt : kpts)
	{
		u = kpt.pt.x;
		v = kpt.pt.y;
		for(ws=1; ws<=max_ws; ++ws)
		{
			if(img_depth.at<ushort>(v,u) != 0) break;
			nonZeroWindowContourLookUp(v, u, ws, img_depth);
		}
		if(img_depth.at<ushort>(v,u) == 0) {kpts_bkpj.push_back(cv::Point3f(0,0,0)); continue;}

		//Save valid Match
		depth_yx = (double) img_depth.at<ushort>(v, u) / depth_scale;
		point_eigen << (double) u * depth_yx, (double) v * depth_yx, (double) depth_yx, 1;
		point_eigen_bkpj = P_inv * point_eigen;
		kpts_bkpj.push_back(cv::Point3f((float) point_eigen_bkpj(0),(float) point_eigen_bkpj(1),(float) point_eigen_bkpj(2)));
	}
	return kpts_bkpj;
}

}
