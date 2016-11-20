/*
 * OptimizationConfiguration.h
 *
 *  Created on: Feb 2, 2016
 *      Author: Glen
 */

#ifndef STEEROPT_INCLUDE_OPTIMIZATIONCONFIGURATION_H_
#define STEEROPT_INCLUDE_OPTIMIZATIONCONFIGURATION_H_

#include "interfaces/ModuleInterface.h"
#include "SteerOptPlugin.h"

namespace SteerOpt {

class STEEROPTPLUG_API OptimizationConfiguration {
public:
	OptimizationConfiguration() {}
	OptimizationConfiguration(SteerLib::OptionDictionary options);
	virtual ~OptimizationConfiguration();

	virtual void setConfiguration(SteerLib::OptionDictionary options);

	size_t _max_fevals;
	double _ftolerance;
	double _xtolerance;
	double _kdmin; // diversity min distance weight
	double _diversity_weight; // weight for the diversity metric
	double _degree_weight;
	double _depth_weight;
	double _entropy_weight;
	double _clearance_weight;
	double _alignment_weight;
	double _wall_length_weight;
	double _metric_threshold;
	double _grid_cells_pre_meter;
	double _sigma;
	double _clearence_distance;
	double _minimize_or_maximize;
	int _lambda;

	std::string _objectiveFunction;
	std::string _graphFileName;
	std::string _parameterFileName;
	std::string _logFileName;
	bool _renderResults;
	bool _saveIterationData;

private:
	SteerLib::OptionDictionary options;

};

} /* namespace SteerOpt */

#endif /* STEEROPT_INCLUDE_OPTIMIZATIONCONFIGURATION_H_ */
