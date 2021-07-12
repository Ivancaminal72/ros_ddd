/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-26 10:59:24
 */

#include "terreslam/utils/util_chrono.h"

namespace terreslam
{
namespace util
{

	void printElapsed(std::vector<double> elapsed_vec, std::string block_str)
	{
			double average = std::accumulate(elapsed_vec.begin(), elapsed_vec.end(), 0.0) / elapsed_vec.size();
			std::cout << block_str << "\n";
			std::cout << "  avg: " << average << "s \n";
			std::cout << "  max: " << *max_element(elapsed_vec.begin(), elapsed_vec.end()) << "s \n";
			std::cout << "  min: " << *min_element(elapsed_vec.begin(), elapsed_vec.end()) << "s \n";
	}


	void tick_high_resolution(const std::chrono::high_resolution_clock::time_point& start_t,
														std::chrono::duration<double>& tick,
														std::vector<double>& elapsed_vec)
	{
			std::chrono::high_resolution_clock::time_point end_t = std::chrono::high_resolution_clock::now();
			double tick_old = tick.count();
			tick = std::chrono::duration_cast<std::chrono::duration<double>>(end_t - start_t);
			double elapsed = tick.count() - tick_old;
			elapsed_vec.push_back(elapsed);
	}

	uint64_t get_unsigned_difference(uint64_t first, uint64_t second) {
			uint64_t abs_diff = (first > second) ? (first - second) : (second - first);
			assert(abs_diff<=INT64_MAX);
			return abs_diff;
	}
}
}