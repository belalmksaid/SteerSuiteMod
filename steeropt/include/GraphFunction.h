/*
 * GraphFunction.h
 *
 *  Created on: 2015-12-08
 *      Author: gberseth
 */

#ifndef GRAPHFUNCTION_H_
#define GRAPHFUNCTION_H_
#include "SteerOptPlugin.h"
#include "Graph.h"

namespace Graphing {

class STEEROPTPLUG_API GraphFunction {
public:
	GraphFunction();
	virtual ~GraphFunction();

	/// Compute the length of the graph, sum of edge lengths
	virtual double graphLength(Graph g);


};

} /* namespace Graphing */
#endif /* GRAPHFUNCTION_H_ */
