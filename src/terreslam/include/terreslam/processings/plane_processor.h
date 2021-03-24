/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-03-04 14:45:06
 */

#pragma once

#include "terreslam/utils/util_types.h"
#include "terreslam/utils/util_map.h"

namespace terreslam
{

class PlaneProcessor
{
public:
	PlaneProcessor(
		std::string logs_dir,
		bool debug
	:
		logs_path_(logs_dir + "/log_plane_detector.txt"),
		debug_(debug)
	{
		std::cout<<"Constructor plane_processor...\n";
		remove(logs_path_.c_str());
	}

	~PlaneProcessor()
	{
		delete cells_bottom_;
	}

	void setDebug(bool d) {debug_=d;}

	void processPlanes(Scan *scan);

private:

	std::string logs_path_;
	bool debug_;
	std::ofstream fp_;

};

}
