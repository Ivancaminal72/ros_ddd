/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-04-23 09:40:10
 */

#pragma once

#include "terreslam/utils/util_types.h"
#include "terreslam/utils/util_map.h"

#include<vector>

namespace terreslam
{

struct Blob
{
	float height; //-1*y+3
	float hue;
	float lightness;
	float x;
	float z;
	float radius;
	uint32_t palette;
	uint32_t ppa;
	uint8_t	stability;
	uint8_t	frame;
};

class BlobDetector
{
public:
	BlobDetector(
		bool debug,
		std::string logs_dir)
	:
		logs_path_(logs_dir + "/log_blob_detector.txt"),
		debug_(debug)
	{
		std::cout<<"Constructor blob_detector...\n";
		remove(logs_path_.c_str());
	}

	void setDebug(bool d) {debug_=d;}

	void detectBlobs(Scan *scan);

private:
	std::string logs_path_;
	bool debug_;
	std::ofstream fp_;
};

}