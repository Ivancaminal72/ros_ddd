/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-04-23 10:32:49
 */

#pragma once

#include "terreslam/utils/util_types.h"
#include "terreslam/utils/util_map.h"

namespace terreslam
{

class KeypointProcessor
{
public:
	KeypointProcessor(
		std::string logs_dir,
		bool debug)
	:
		logs_path_(logs_dir + "/log_keypoint_detector.txt"),
		debug_(debug)
	{
		std::cout<<"Constructor keypoint_processor...\n";
		remove(logs_path_.c_str());
	}

	~KeypointProcessor(){}

	void setDebug(bool d) {debug_=d;}
	void processKeypoints(Scan *scan);

private:
	std::string logs_path_;
	bool debug_;
	std::ofstream fp_;

};

}
