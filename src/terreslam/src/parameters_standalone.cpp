/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-12 13:59:57
 */

#include <iostream>

#include "terreslam/parameters.h"

namespace terreslam
{

void printHelp(po::options_description all_opts, uint status)
{
	std::cout << "Usage: rosrun data_to_rosbag pcd_to_png {args}\n";
	std::cout << all_opts << std::endl << std::endl;
	std::cout << "Warn: the 1st mandatory arg can be directly specified (without option)\n";
	exit(status);
}

void loadFrontendParameters(int argc, char** argv)
{
	//Parse arguments
	po::options_description mandatory_opts("Mandatory args");
	mandatory_opts.add_options()
		("project,p", po::value<int>(&params_frontend.cam_idx_proj), "index of the camera to do pcd projection");
	
	po::positional_options_description positional_opts;
	positional_opts.add("project", -1);

	po::options_description optional_opts("Optional args");
	optional_opts.add_options()
		("help,h", "produce a help message");

	po::variables_map vm;
	po::options_description all_opts;
	all_opts.add(mandatory_opts).add(optional_opts);
	po::store(po::command_line_parser(argc, argv).options(all_opts).positional(positional_opts).run(), vm);
	po::notify(vm);

	if (vm.count("help")) printHelp(all_opts, 0);

	for(const auto& opt : mandatory_opts.options())
	{
		if(!vm.count(opt->long_name()))
		{
		std::cerr << "ERROR: Missing mandatory_opts option " + opt->long_name() + "\n";
		printHelp(all_opts, 1);
		}
	}
}

void loadBackendParameters(int argc, char** argv)
{
	//Parse arguments
	po::options_description mandatory_opts("Mandatory args");
	mandatory_opts.add_options()
		("project,p", po::value<int>(&params_frontend.cam_idx_proj), "index of the camera to do pcd projection");
	
	po::positional_options_description positional_opts;
	positional_opts.add("project", -1);

	po::options_description optional_opts("Optional args");
	optional_opts.add_options()
		("help,h", "produce a help message");

	po::variables_map vm;
	po::options_description all_opts;
	all_opts.add(mandatory_opts).add(optional_opts);
	po::store(po::command_line_parser(argc, argv).options(all_opts).positional(positional_opts).run(), vm);
	po::notify(vm);

	if (vm.count("help")) printHelp(all_opts, 0);

	for(const auto& opt : mandatory_opts.options())
	{
		if(!vm.count(opt->long_name()))
		{
		std::cerr << "ERROR: Missing mandatory_opts option " + opt->long_name() + "\n";
		printHelp(all_opts, 1);
		}
	}
}

void loadFrontendParameters(std::vector<std::string> str_argv)
{
	std::vector<char*> argv;
	argv.reserve(str_argv.size());
	for(size_t i = 0; i < str_argv.size(); ++i)
		argv.push_back(const_cast<char*>(str_argv[i].c_str()));
	std::vector<char*> pointerVec(str_argv.size());
	loadFrontendParameters(str_argv.size(), &argv[0]);
}

void loadBackendParameters(std::vector<std::string> str_argv)
{
	std::vector<char*> argv;
	argv.reserve(str_argv.size());
	for(size_t i = 0; i < str_argv.size(); ++i)
		argv.push_back(const_cast<char*>(str_argv[i].c_str()));
	std::vector<char*> pointerVec(str_argv.size());
	loadBackendParameters(str_argv.size(), &argv[0]);
}

}