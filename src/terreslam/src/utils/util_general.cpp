/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-07-05 12:40:11
 */

#include "terreslam/utils/util_general.h"

namespace terreslam
{
namespace util
{

float calculateMedian(std::vector<float> scores)
{
  size_t size = scores.size();

  if (size == 0)
  {
    return -1;  
  }
  else
  {
    std::sort(scores.begin(), scores.end());
    if (size % 2 == 0)
    {
      return (scores[size / 2 - 1] + scores[size / 2]) / 2;
    }
    else 
    {
      return scores[size / 2];
    }
  }
}

cv::Mat quat2Mat(tf2::Quaternion q)
{
    tf2::Matrix3x3 m(q); 
    cv::Mat rotm_mat(3, 3, CV_32F); 
    for(int i = 0; i < 3; i++)
    {
         tf2::Vector3 test = m.getRow(i);
         rotm_mat.at<float>(i, 0) = test.getX();
         rotm_mat.at<float>(i, 1) = test.getY();
         rotm_mat.at<float>(i, 2) = test.getZ();
    }
return rotm_mat;
}

}
}