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

}
}