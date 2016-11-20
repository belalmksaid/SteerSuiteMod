/*
 * RoundRobinOptimization.h
 *
 *	Wrapper for the RoundRobin Optimization algorithm.
 *
 *  Created on: Jan 16, 2016
 *      Author: Glen
 */

#ifndef STEEROPT_INCLUDE_ROUNDROBINOPTIMIZATION_H_
#define STEEROPT_INCLUDE_ROUNDROBINOPTIMIZATION_H_

#include "SteerOptPlugin.h"
#include "CMAOptimize.h"
#include <vector>
#include "CMAOptimizePrivate.h"

// template class libcmaes::RoundRobinCMA<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, libcmaes::linScalingStrategy>>;
// using namespace libcmaes;
namespace SteerOpt {

class STEEROPTPLUG_API RoundRobinOptimization : public SteerOpt::CMAOptimize {
public:
	RoundRobinOptimization(SteerSuite::SimWorld * world, SteerSuite::SteerSuite * steersuite);
	virtual ~RoundRobinOptimization();

	virtual void optimize();
	
	virtual std::vector<double> getBestMember(size_t m);
	virtual size_t getNumMembers();
	virtual void logOptimization(std::string logFileName);
	virtual double getMemberDiversity(size_t m);

protected:
	std::vector<std::vector<double> > _solutions;
	std::vector<double> _diversity;
	/// If there are items in thie list, a different optimization objective will be used.
	std::vector<double> _objectiveWeights;
};

} /* namespace SteerOpt */

#endif /* STEEROPT_INCLUDE_ROUNDROBINOPTIMIZATION_H_ */
