/*
 * GraphOptimizePrivate.h
 *
 *  Created on: 2015-12-08
 *      Author: gberseth
 */

#ifndef GRAPHOPTIMIZEPRIVATE_H_
#define GRAPHOPTIMIZEPRIVATE_H_

#include "src/cmaes.h"
#include "GraphOptimize.h"
#include "SteerOptPlugin.h"

namespace Graphing {

class STEEROPTPLUG_API GraphOptimizePrivate : public GraphOptimize {
public:
	GraphOptimizePrivate(Graph g, OptimizationParameters params);
	virtual ~GraphOptimizePrivate();

	libcmaes::FitFunc eval = [this](const double *x, const int N)
	{ // TODO, this might work...
		// m_evaluationCounter++;
		return this->update(x,N);
	};
};

} /* namespace Graphing */
#endif /* GRAPHOPTIMIZEPRIVATE_H_ */
