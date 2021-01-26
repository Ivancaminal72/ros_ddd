/*
 *    Author: Ivan Caminal
 *    Created Date: 2021-01-12 14:00:11
 *    Last Modified: 2021-01-21 12:43:38
 */

#pragma once

#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace terreslam
{

void printHelp(po::options_description all_opts, uint status);
void loadFrontendParameters(int argc, char** argv);
void loadFrontendParameters(std::vector<std::string> str_argv);
void loadBackendParameters(int argc, char** argv);
void loadBackendParameters(std::vector<std::string> str_argv);


//Frontend parameters
struct frontendParams
{
	int cam_idx_proj = -1;
} params_frontend;

//Backend parameters
struct backendParams
{
	
} params_backend;

}