/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-21 17:22:47
 *    Last Modified: 2021-01-26 10:23:44
 */

#include <iostream>

#include "terreslam/camera_model.h"
#include <rtabmap/utilite/UStl.h>
#include <assert.h>

namespace terreslam
{

CameraModel::CameraModel(
	const std::string & name,
	const cv::Size & size,
	const Eigen::Matrix3d & K,
	const Eigen::Projective3d & P)
 : 
	name_(name),
	size_(size),
	K_(K),
	P_(P)
{}

CameraModel::CameraModel(
	const sensor_msgs::CameraInfo & info)
{
		assert(info.K.empty() || info.K.size() == 9);
		if(!info.K.empty())
		{
			double data[9];
			memcpy(data, info.K.elems, 9*sizeof(double));
			K_ = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(data);
		}

		// cv::Mat D;
		// if(info.D.size())
		// {
		// 	if(info.D.size()>=4 &&
		// 		(uStrContains(info.distortion_model, "fisheye") ||
		// 			uStrContains(info.distortion_model, "equidistant")))
		// 	{
		// 		D = cv::Mat::zeros(1, 6, CV_64FC1);
		// 		D.at<double>(0,0) = info.D[0];
		// 		D.at<double>(0,1) = info.D[1];
		// 		D.at<double>(0,4) = info.D[2];
		// 		D.at<double>(0,5) = info.D[3];
		// 	}
		// 	else
		// 	{
		// 		D = cv::Mat(1, info.D.size(), CV_64FC1);
		// 		memcpy(D.data, info.D.data(), D.cols*sizeof(double));
		// 	}
		// }

		assert(info.R.empty() || info.R.size() == 9);
		if(!info.R.empty())
		{
			double data[9];
			memcpy(data, info.R.elems, 9*sizeof(double));
			R_ = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(data);
		}

		assert(info.P.empty() || info.P.size() == 12);
		if(!info.P.empty())
		{
			double data[12];
			memcpy(data, info.P.elems, 12*sizeof(double));
			P_ = Eigen::Projective3d::Identity();
			P_.matrix().block<3,4>(0,0) = Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor> >(data);
		}
}


void CameraModel::printModel()
{
	std::cout << std::endl << std::endl;
	std::string sep = "\n----------------------------------------\n";
	Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");
	std::cout << P_.matrix().format(CleanFmt) << std::endl;
	std::cout << std::endl << std::endl;
}


}