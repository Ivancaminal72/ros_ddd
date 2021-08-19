/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-04-23 10:50:34
 */

#include <iostream>

#include "terreslam/processings/keypoint_processor.h"

namespace terreslam
{

std::vector<cv::DMatch> matchTwoImage(const cv::Mat &descriptor1, const cv::Mat &descriptor2)
{
	auto matcher = cv::DescriptorMatcher::create("BruteForce");
	std::vector<cv::DMatch> matches;
	matcher->match(descriptor1, descriptor2, matches);
	return matches;
}

void nonZeroWindowContourLookUp(int& u, int& v, const int& ws, const cv::Mat& img)
{
	assert(ws > 0);
	int i,j;
	for(j=-ws+1; j<ws; ++j)
	{
		if(img.at<ushort>(u+ws,v+j) != 0) {u=u+ws; v=v+j; return;}
	}
	for(j=-ws+1; j<ws; ++j)
	{
		if(img.at<ushort>(u-ws,v+j) != 0) {u=u-ws; v=v+j; return;}
	}
	for(i=-ws; i<=ws; ++i)
	{
		if(img.at<ushort>(u+i,v-ws) != 0) {u=u+i; v=v-ws; return;}
	}
	for(i=-ws; i<=ws; ++i)
	{
		if(img.at<ushort>(u+i,v+ws) != 0) {u=u+i; v=v+ws; return;}
	}
}

}
