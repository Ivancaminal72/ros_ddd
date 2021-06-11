/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-26 10:59:06
 */

#pragma once

#include <vector>
#include <iostream>
#include <chrono>
#include <numeric>
#include <algorithm>
#include <cassert>

namespace terreslam
{
  void printElapsed(std::vector<double> elapsed_vec, std::string block_str);
  void tick_high_resolution(const std::chrono::high_resolution_clock::time_point& start_t,
														std::chrono::duration<double>& tick,
														std::vector<double>& elapsed_vec);
  uint64_t get_unsigned_difference(uint64_t first, uint64_t second);
}