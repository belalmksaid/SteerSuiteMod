/*
 * GraphOptimize.h
 *
 *  Created on: 2015-12-08
 *      Author: gberseth
 */

#ifndef GRAPHOPTIMIZE_H_
#define GRAPHOPTIMIZE_H_

#include "Graph.h"
#include "src/cmaes.h"
#include "OptimizationParameters.h"
#include "SteerOptPlugin.h"

namespace Graphing {
// #include "src/cmaes.h"

class STEEROPTPLUG_API GraphOptimize {
public:
	GraphOptimize(Graph g, OptimizationParameters params);
	virtual ~GraphOptimize();

	virtual void optimize();

	// libcmaes::FitFunc eval;

	libcmaes::FitFunc eval = [this](const double *x, const int N)
	{ // TODO, this might work...
		// m_evaluationCounter++;
		return this->update(x,N);
	};

	virtual double update(const double *x, const int N);
public:
	Graph _graph;
	libcmaes::CMASolutions _cmasols;
	OptimizationParameters _params;
	double _best_f;
	Graph _best_graph;
	std::vector<double> _best_x;
};

} /* namespace Graphing */
#endif /* GRAPHOPTIMIZE_H_ */
