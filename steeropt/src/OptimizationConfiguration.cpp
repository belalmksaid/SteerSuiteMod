/*
 * OptimizationConfiguration.cpp
 *
 *  Created on: Feb 2, 2016
 *      Author: Glen
 */

#include "OptimizationConfiguration.h"
#include "util/Misc.h"

namespace SteerOpt {

OptimizationConfiguration::OptimizationConfiguration(SteerLib::OptionDictionary options) 
{
	// TODO Auto-generated constructor stub
	// _logger = NULL;
	// Default values
	_renderResults = false;
	this->_saveIterationData = false;
	this->_xtolerance = 0.05;
	this->_ftolerance = 0.5;
	this->_max_fevals = 100;
	this->_diversity_weight = 0.1;
	this->_kdmin = 20.0;
	_degree_weight = 1.0;
	_depth_weight = 1.0;
	_entropy_weight = 1.0;
	_clearance_weight = 1.0;
	_alignment_weight = 1.0;
	_wall_length_weight = 0.0;
	_clearence_distance = 0.50;
	this->_metric_threshold = 0.7;
	this->_sigma = 0.3;
	this->_lambda = -1;
	this->_minimize_or_maximize = (1.0);
	this->setConfiguration(options);

}

OptimizationConfiguration::~OptimizationConfiguration() {
	// TODO Auto-generated destructor stub
	
}

void OptimizationConfiguration::setConfiguration(SteerLib::OptionDictionary options)
{
	// parse command line options
	this->options = options;
	SteerLib::OptionDictionary::const_iterator optionIter;
	try
	{
		for (optionIter = options.begin(); optionIter != options.end(); ++optionIter)
		{
			// std::cout << (*optionIter).first << ": " << (*optionIter).second << std::endl;
			if ((*optionIter).first == "maxIterations") {
				_max_fevals = atoi((*optionIter).second.c_str());
				// std::cout << "Max number of function evaluations: " << _max_fevals << std::endl;
			}
			else if ((*optionIter).first == "xtolerance") {
				// this->_xtolerance = atof((*optionIter).second.c_str());
				std::cout << "Parsing xtolerance: " << this->_xtolerance << std::endl;
			}
			else if ((*optionIter).first == "saveIterationData") {
				if (((*optionIter).second == "true"))
				{
					_saveIterationData = true;
				}
			}
			else if ((*optionIter).first == "minimize_or_maximize") {
				this->_minimize_or_maximize = atof((*optionIter).second.c_str());
			}
			else if ((*optionIter).first == "wall_length_weight") {
				this->_wall_length_weight = atof((*optionIter).second.c_str());
			}
			else if ((*optionIter).first == "clearence_distance") {
				this->_clearence_distance = atof((*optionIter).second.c_str());
			}
			else if ((*optionIter).first == "sigma") {
				this->_sigma = atof((*optionIter).second.c_str());
			}
			else if ((*optionIter).first == "lambda") {
				this->_lambda = atof((*optionIter).second.c_str());
			}
			else if ((*optionIter).first == "grid_cells_pre_meter") {
				this->_grid_cells_pre_meter = atof((*optionIter).second.c_str());
			}
			else if ((*optionIter).first == "metric_threshold") {
				this->_metric_threshold = atof((*optionIter).second.c_str());
			}
			else if ((*optionIter).first == "degree_weight") {
				this->_degree_weight = atof((*optionIter).second.c_str());
			}
			else if ((*optionIter).first == "depth_weight") {
				this->_depth_weight = atof((*optionIter).second.c_str());
			}
			else if ((*optionIter).first == "entropy_weight") {
				this->_entropy_weight = atof((*optionIter).second.c_str());
			}
			else if ((*optionIter).first == "clearance_weight") {
				this->_clearance_weight = atof((*optionIter).second.c_str());
			}
			else if ((*optionIter).first == "alignment_weight") {
				this->_alignment_weight = atof((*optionIter).second.c_str());
			}
			else if ((*optionIter).first == "kdmin") {
				this->_kdmin = atof((*optionIter).second.c_str());
			}
			else if ((*optionIter).first == "diversityWeight") {
				this->_diversity_weight = atof((*optionIter).second.c_str());
			}
			else if ((*optionIter).first == "ftolerance") {
				this->_ftolerance = atof((*optionIter).second.c_str());
			}
			else if ((*optionIter).first == "objectiveFunction") {
				_objectiveFunction = (*optionIter).second;
			}
			else if ((*optionIter).first == "graphFileName") {
				_graphFileName = (*optionIter).second;
			}
			else if ((*optionIter).first == "parameterFileName") {
				_parameterFileName = (*optionIter).second;
			}
			else if ((*optionIter).first == "logFileName") {
				_logFileName = (*optionIter).second;
				// initLogger(_logFileName);
			}
			else if ((*optionIter).first == "renderResults") {
				if ( ( (*optionIter).second == "true") )
				{
					_renderResults = true;
				}
			}
			//RECORD NEEDS TO BE PROCESSED
			else
			{
				// Glen - I'll leave this for now but it would be better if
				// options were passed to all others modules.
				throw Util::GenericException("unrecognized option \"" + Util::toString((*optionIter).first) + "\" given to steeropt module.");
				// std::cerr << "option: " << (*optionIter).first << " not recognized by " <<
				// 	"scenario module" <<  std::endl;
			}

		}
	}
	catch (Util::GenericException& e)
	{
		std::cerr << e.what() << std::endl;
		throw new Util::GenericException(" Something went wrong while parsing steeropt configuration parameters: ");
		exit(0);
	}
}


} /* namespace SteerOpt */
