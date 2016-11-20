/*
 * CMAOptimize.h
 *
 *  Created on: Oct 28, 2015
 *      Author: Glen
 */

#ifndef STEEROPT_INCLUDE_CMAOPTIMIZE_H_
#define STEEROPT_INCLUDE_CMAOPTIMIZE_H_

#include "SteerSimOptimize.h"
#include "SteerOptPlugin.h"
#include "Graph.h"
#include "src/cmaes.h"

namespace SteerOpt {

class STEEROPTPLUG_API CMAOptimize : public SteerSimOptimize {
public:
	CMAOptimize(SteerSuite::SimWorld * world, SteerSuite::SteerSuite * steersuite);
	virtual ~CMAOptimize();

	/// \brief From INameable: return the class name.
	virtual void optimize(std::string objective="PLE");
	virtual void preprocessEnvironment(const double * x, int N);
	virtual void postprocessEnvironment(const double * x, int N);
	virtual void preprocessEnvironment(std::vector<double> in)
	{
		this->preprocessEnvironment(&in.front(), in.size());
	}
	virtual void postprocessEnvironment(std::vector<double> in)
	{
		this->postprocessEnvironment(&in.front(), in.size());
	}
	/// preprocessEnvironment should be called before these.
	virtual double calcMultiObjective(const double * x, int N);
	virtual double calcMultiObjectiveObjectives(const double * x, int N);
	virtual double calcMultiObjectivePenalties(const double * x, int N);
	virtual std::vector<double> _calcMultiObjective(const double * x, int N);
	virtual std::vector<double> _calcMultiObjectiveObjectives(const double * x, int N);
	virtual std::vector<double> _calcMultiObjectivePenalties(const double * x, int N);
	virtual double calcDegree(const double * x, int N);
	virtual std::vector<float> calcDepthAndEntropy(const double * x, int N);

	virtual double calcMultiObjective(std::vector<double> in)
	{
		return this->calcMultiObjective(&in.front(), in.size());
	}
	virtual std::vector<double> _calcMultiObjective(std::vector<double> in)
	{
		return this->_calcMultiObjective(&in.front(), in.size());
	}
	virtual double calcDegree(std::vector<double> in)
	{
		return this->calcDegree(&in.front(), in.size());
	}
	virtual std::vector<float> calcDepthAndEntropy(std::vector<double> in)
	{
		return this->calcDepthAndEntropy(&in.front(), in.size());
	}
	///
	virtual void setGraph(Graphing::Graph graph) { this->_graph = graph; }

	Util::Point _gridBound1, _gridBound2;
	unsigned int _gridNumX , _gridNumZ;
	float _height, _meanDegree;



	virtual void logOptimization(std::string logFileName);

public:
	libcmaes::CMASolutions _cmasols;
	dMat _best_x;
	Graphing::Graph _graph;
	Graphing::Graph _tmp_graph;

};

} /* namespace SteerOpt */

#endif /* STEEROPT_INCLUDE_CMAOPTIMIZE_H_ */
