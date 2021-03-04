/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-26 10:59:24
 */

#include <assert.h>

#include "terreslam/utils/util_algebra.h"
#include <rtabmap/utilite/UStl.h>

namespace terreslam
{
  void printEigenMatrix(Eigen::MatrixXd mat)
  {
    std::cout << std::endl << std::endl;
    std::string sep = "\n----------------------------------------\n";
    Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");
    std::cout << mat.matrix().format(CleanFmt) << std::endl;
    std::cout << std::endl << std::endl;
  }
}