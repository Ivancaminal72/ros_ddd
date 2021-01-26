/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-21 17:23:39
 *    Last Modified: 2021-01-26 10:15:52
 */

#pragma once

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>

namespace terreslam
{

class CameraModel
{
public:
	CameraModel();
	// K is the camera intrinsic 3x3
	// D is the distortion coefficients 1x5
	// R is the rectification matrix 3x3 (computed from stereo or Identity)
	// P is the projection matrix 3x4 (computed from stereo or equal to [K [0 0 1]'])
	
	CameraModel(
			const std::string & name,
			const cv::Size & size,
			const Eigen::Matrix3d & K,
			const Eigen::Projective3d & P);

	//ideal
	CameraModel(
			const std::string & name,
			const cv::Size & size,
			const Eigen::Matrix3d & K,
			const std::array<double, 5> & D,
			const Eigen::Matrix3d & R,
			const Eigen::Projective3d & P,
			const Eigen::Affine3d & local_transform = Eigen::Affine3d::Identity());

	//ros
	CameraModel(
			const sensor_msgs::CameraInfo & info);
	//ideal
	// CameraModel(
	// 		const sensor_msgs::CameraInfo & info,
	// 		const Eigen::Affine3d & local_transform)
	
	void setName(const std::string & name) {name_=name;}
	const std::string & name() const {return name_;}

	void setImageSize(const cv::Size & size);
	const cv::Size & imageSize() const {return size_;}
	int imageWidth() const {return size_.width;}
	int imageHeight() const {return size_.height;}

	Eigen::Matrix3d K() const {return K_;}
	// std::array<double, 5> D_;
	Eigen::Matrix3d R() const {return R_;}
	Eigen::Projective3d P() const {return P_;}
	// cv::Mat mapX_;
	// cv::Mat mapY_;
	// Eigen::Affine3d local_transform_;

	void printModel();

private:
	std::string name_;
	cv::Size size_;
	Eigen::Matrix3d K_;
	// std::array<double, 5> D_;
	Eigen::Matrix3d R_;
	Eigen::Projective3d P_;
	// cv::Mat mapX_;
	// cv::Mat mapY_;
	Eigen::Affine3d local_transform_;
};

}