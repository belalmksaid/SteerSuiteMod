/*
 * Graph.h
 *
 *  Created on: 2015-12-01
 *      Author: gberseth
 */

#ifndef GRAPH_H_
#define GRAPH_H_

#include <Eigen/Core>
#include <vector>
#include "SteerOptPlugin.h"
#include "OptimizationParameters.h"
#include <clipper/clipper.hpp>

namespace Graphing {

struct STEEROPTPLUG_API Edge {

	Edge(size_t o, size_t e)
	{
		_origin = o;
		_end = e;
	}
	size_t _origin;
	size_t _end;
};

inline std::ostream &operator<<(std::ostream &out, const Edge &a)
{ // methods used here must be const
	out << "edge:<" << a._origin << "," << a._end << ">" << std::endl;
	return out;
}

class STEEROPTPLUG_API Graph {
public:
	Graph();
	virtual ~Graph();

	void importFromFile(std::string filename);
	void saveToFile(std::string filename);
	void saveSVGToFile(std::string filename);
	void saveSVGToFile(std::string filename, OptimizationParameters params);
	void computeMinkowskiSums(double radius);
	void clearMinkowskiSums();
	void updateMinkowskiSums(std::set<size_t> nodes_, double radius);
	double computeMinkowskiSumIntersectionArea();
	double alignmentPenalty();
	double sumWallLengths();
	double edgeTheta(size_t edge);
	void initFrom(Graph g);
	std::vector<Eigen::Vector3d> nodes;
	std::vector<Edge> edges;
	virtual size_t num_edges()
	{
		return edges.size();
	}
	virtual size_t num_nodes()
	{
		return nodes.size();
	}
	virtual std::vector<double> getNode(size_t e)
	{
		std::vector<double> out;
		out.push_back(this->nodes[e](0));
		out.push_back(this->nodes[e](1));
		out.push_back(this->nodes[e](2));
		return out;
	}

	virtual void setNode(std::vector<double> n, size_t node)
	{
		this->nodes[node] = Eigen::Vector3d(n[0], n[1], n[2]);

	}

	virtual Edge get_edge(size_t e)
	{
		return this->edges[e];
	}

private:
	void _updateEdge(size_t edge, double radius);
	ClipperLib::Path _getEdgeGeometry(size_t edge, double scale);
	std::vector<ClipperLib::Paths> _obstacles;
	std::vector<ClipperLib::Paths> _intersections;


};

} /* namespace Graphing */
#endif /* GRAPH_H_ */
