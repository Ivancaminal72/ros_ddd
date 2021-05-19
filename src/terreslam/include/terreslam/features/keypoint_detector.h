/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-04-23 09:40:10
 */

#pragma once

#include "terreslam/utils/util_types.h"
#include "terreslam/utils/util_map.h"

namespace terreslam
{

class KeypointDetector
{
public:
	KeypointDetector(
		bool debug,
		std::string logs_dir)
	:
		logs_path_(logs_dir + "/log_keypoint_detector.txt"),
		debug_(debug)
	{
		std::cout<<"Constructor keypoint_detector...\n";
		remove(logs_path_.c_str());
	}

	~KeypointDetector(){}

	void setDebug(bool d) {debug_=d;}
	void detectKeypoints(Scan *scan);

private:
	std::string logs_path_;
	bool debug_;
	std::ofstream fp_;
};

}