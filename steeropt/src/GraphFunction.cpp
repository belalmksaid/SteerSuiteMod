/*
 * GraphFunction.cpp
 *
 *  Created on: 2015-12-08
 *      Author: gberseth
 */

#include "GraphFunction.h"

namespace Graphing {

GraphFunction::GraphFunction() {
	// TODO Auto-generated constructor stub

}

GraphFunction::~GraphFunction() {
	// TODO Auto-generated destructor stub
}

double GraphFunction::graphLength(Graph graph)
{
	double sum = 0;
	for (size_t edge=0; edge < graph.edges.size(); edge++)
	{
		double length = (graph.nodes.at(graph.edges[edge]._origin) - graph.nodes.at(graph.edges[edge]._end)).norm();
		sum += length;
	}
	return sum;
}

} /* namespace Graphing */
