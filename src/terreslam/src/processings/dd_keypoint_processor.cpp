/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-04-23 10:50:34
 */

#include <iostream>

#include "terreslam/processings/dd_keypoint_processor.h"

namespace terreslam
{

std::vector<cv::DMatch> matchTwoImage(const cv::Mat &descriptor1, const cv::Mat &descriptor2)
{
	auto matcher = cv::DescriptorMatcher::create("BruteForce");
	std::vector<cv::DMatch> matches;
	matcher->match(descriptor1, descriptor2, matches);
	return matches;
}

void nonZeroWindowContourLookUp(int& v, int& u, const int& ws, const cv::Mat& img)
{
	assert(ws > 0);
	int j,i,vv,uu;
	uu = u+ws;
	for(j=-ws+1; j<ws; ++j)
	{
		vv = v+j;
		if(uu >= img.cols || vv >= img.rows || uu < 0 || vv < 0) break;
		if(img.at<ushort>(vv,uu) != 0) {v=vv; u=uu; return;}
	}
	uu = u-ws;
	for(j=-ws+1; j<ws; ++j)
	{
		vv = v+j;
		if(uu >= img.cols || vv >= img.rows || uu < 0 || vv < 0) break;
		if(img.at<ushort>(vv,uu) != 0) {v=vv; u=uu; return;}
	}
	vv = v-ws;
	for(i=-ws; i<=ws; ++i)
	{
		uu = u+i;
		if(uu >= img.cols || vv >= img.rows || uu < 0 || vv < 0) break;
		if(img.at<ushort>(vv,uu) != 0) {v=vv; u=uu; return;}
	}
	vv = v+ws;
	for(i=-ws; i<=ws; ++i)
	{
		uu = u+i;
		if(uu >= img.cols || vv >= img.rows || uu < 0 || vv < 0) break;
		if(img.at<ushort>(vv,uu) != 0) {v=vv; u=uu; return;}
	}
}

}
