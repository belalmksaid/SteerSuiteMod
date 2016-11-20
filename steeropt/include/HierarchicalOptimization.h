/*
 * HierarchicalOptimization.h
 *
 *  Created on: Apr 24, 2016
 *      Author: Glen
 */

#ifndef STEEROPT_INCLUDE_HIERARCHICALOPTIMIZATION_H_
#define STEEROPT_INCLUDE_HIERARCHICALOPTIMIZATION_H_

#include "SteerOptPlugin.h"
#include "CMAOptimize.h"
#include <vector>
#include "RoundRobinOptimization.h"

namespace SteerOpt {

class  STEEROPTPLUG_API HierarchicalOptimization : public SteerOpt::RoundRobinOptimization {
public:
	HierarchicalOptimization(SteerSuite::SimWorld * world, SteerSuite::SteerSuite * steersuite);
	virtual ~HierarchicalOptimization();

	virtual void optimize();

	virtual std::vector<double> getBestMember(size_t m);
	virtual size_t getNumMembers();
	virtual void logOptimization(std::string logFileName);
private:
	std::vector<float> _metric_thresholds;
};

} /* namespace SteerOpt */

#endif /* STEEROPT_INCLUDE_HIERARCHICALOPTIMIZATION_H_ */
