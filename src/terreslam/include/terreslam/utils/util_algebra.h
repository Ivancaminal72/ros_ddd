/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-26 10:59:06
 */

#pragma once

#include <sensor_msgs/CameraInfo.h>
#include <Eigen/Geometry>

namespace terreslam
{
  void printEigenMatrix(Eigen::MatrixXd mat);
}