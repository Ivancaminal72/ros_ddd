/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-26 10:59:06
 *    Last Modified: 2021-01-26 11:49:02
 */

#pragma once

#include <sensor_msgs/CameraInfo.h>
#include <Eigen/Geometry>

namespace terreslam
{
  void printEigenMatrix(Eigen::MatrixXd mat);
}