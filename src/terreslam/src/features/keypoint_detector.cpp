/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-04-23 09:40:30
 */

#include "terreslam/features/keypoint_detector.h"

namespace terreslam
{

void KeypointDetector::detectKeypoints(Scan *scan)
{
	fp_.open(logs_path_, std::ios::app);
	if(debug_)
	{
		fp_<<"*****************detectKeypoints**************************************"<<std::endl;
	}
	

	fp_.close();

}

}