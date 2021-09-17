/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-06-15 11:13:30
 */

#include "ros/ros.h"
#include "nodelet/loader.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "blob_detector_nodelet");
	nodelet::Loader nodelet;
	nodelet::M_string remap(ros::names::getRemappings());
	nodelet::V_string nargv;
	for(int i=1;i<argc;++i) nargv.push_back(argv[i]);
	std::string nodelet_name = ros::this_node::getName();
	nodelet.load(nodelet_name, "terreslam/blob_detector_nodelet", remap, nargv);
	ros::spin();
	return 0;
}