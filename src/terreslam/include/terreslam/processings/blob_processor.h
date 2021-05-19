/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-04-23 10:32:49
 */

#pragma once

#include "terreslam/utils/util_types.h"
#include "terreslam/utils/util_map.h"

namespace terreslam
{

class BlobProcessor
{
public:
	BlobProcessor(
		std::string logs_dir,
		bool debug)
	:
		logs_path_(logs_dir + "/log_blob_detector.txt"),
		debug_(debug)
	{
		std::cout<<"Constructor blob_processor...\n";
		remove(logs_path_.c_str());
	}

	~BlobProcessor(){}

	void setDebug(bool d) {debug_=d;}
	void processBlobs(Scan *scan);

private:
	std::string logs_path_;
	bool debug_;
	std::ofstream fp_;

};

}
