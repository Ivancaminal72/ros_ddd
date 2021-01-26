/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-26 19:35:30
 *    Last Modified: 2021-01-26 19:36:28
 */

#include "ros/ros.h"
#include "nodelet/loader.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rgb_cloud_frontend");
	nodelet::Loader nodelet;
	nodelet::M_string remap(ros::names::getRemappings());
	nodelet::V_string nargv;
	for(int i=1;i<argc;++i) nargv.push_back(argv[i]);
	std::string nodelet_name = ros::this_node::getName();
	nodelet.load(nodelet_name, "terreslam/rgb_cloud_frontend", remap, nargv);
	ros::spin();
	return 0;
}