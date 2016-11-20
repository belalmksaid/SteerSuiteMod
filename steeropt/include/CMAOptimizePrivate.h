/*
 * CMAOptimizePrivate.h
 *
 *  Created on: Oct 29, 2015
 *      Author: Glen
 *
 *  Created only because SWIG did not like a lambda expression...
 */

#ifndef STEEROPT_INCLUDE_CMAOPTIMIZEPRIVATE_H_
#define STEEROPT_INCLUDE_CMAOPTIMIZEPRIVATE_H_

#include "src/cmaes.h"
#include "CMAOptimize.h"
#include "SteerOptPlugin.h"

namespace SteerOpt {

class CMAOptimizePrivate : public CMAOptimize {
public:
	CMAOptimizePrivate(SteerSuite::SimWorld * world, SteerSuite::SteerSuite * steersuite);
	virtual ~CMAOptimizePrivate();

	// libcmaes::FitFunc eval;
	libcmaes::FitFunc eval = [this](const double *x, const int N)
	{ // TODO, this might work...
		// m_evaluationCounter++;
		return this->simulate(x, N) * this->_optConfig._minimize_or_maximize;
	};

	// libcmaes::FitFunc eval;
	libcmaes::FitFunc evalPLE = [this](const double *x, const int N)
	{ // TODO, this might work...
		// m_evaluationCounter++;
		return this->simulateSteeringAlgorithm(x, N) * this->_optConfig._minimize_or_maximize;
	};

	libcmaes::FitFunc evalDegree = [this](const double *x, const int N)
	{ // TODO, this might work...
	  // m_evaluationCounter++;
		this->preprocessEnvironment(x, N);
		double out = this->calcDegree(x, N) * this->_optConfig._minimize_or_maximize;
		this->postprocessEnvironment(x, N);
		return out;
	};

	/*
	 * This is for combining many objectives
	 * Degree, Depth, Entropy
	 */
	libcmaes::FitFunc evalManyObjective = [this](const double *x, const int N)
	{ // TODO, this might work...
	  // m_evaluationCounter++;
		this->preprocessEnvironment(x, N);
		double out = (this->calcMultiObjectiveObjectives(x, N) * this->_optConfig._minimize_or_maximize) + this->calcMultiObjectivePenalties(x, N);
		this->postprocessEnvironment(x, N);
		return out;
	};

	libcmaes::FitFunc hierarchicalThreshold = [this](const double *x, const int N)
	{ // TODO, this might work...
	  // m_evaluationCounter++;
		this->preprocessEnvironment(x, N);
		double degree = (this->calcMultiObjectiveObjectives(x, N) * this->_optConfig._minimize_or_maximize) + this->calcMultiObjectivePenalties(x, N);
		this->postprocessEnvironment(x, N);
		double out;
		if (degree > (_degree_threshold))
		{
			out =  0.0;
		}
		else
		{
			out = -std::pow(((this->_degree_threshold - degree) + 10), 4);
		}
		// std::cout << "Degree penalty: " << out << std::endl;
		return out;
	};

	double _degree_threshold;
};

} /* namespace SteerOpt */

#endif /* STEEROPT_INCLUDE_CMAOPTIMIZEPRIVATE_H_ */
