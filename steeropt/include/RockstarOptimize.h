//RockstarOptimize.h

#include "SteerSimOptimize.h"
#include <Eigen/Core>
#include <iostream>
#include "SteerOptPlugin.h"
// #include "Rockstar.hpp"
// #include "CMAES.hpp"

namespace SteerOpt {

class STEEROPTPLUG_API RockstarOptimize : public SteerSimOptimize {
public:
	RockstarOptimize(SteerSuite::SimWorld * world, SteerSuite::SteerSuite * steersuite);
	virtual ~RockstarOptimize();
	virtual void optimize();

	/*** rosenbrock - default objective function for testing ***/
	virtual double rosen(Eigen::VectorXd x);
};

} /* namespace SteerOpt */
