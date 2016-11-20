/*
 * Graph.cpp
 *
 *  Created on: 2015-12-01
 *      Author: gberseth
 */

#include "Graph.h"
#include <iostream>
#include <fstream>
// Clipper library
#include <clipper/clipper_extra.hpp>
// #include "obstacles/OrientedBoxObstacle.h"
#include "util/Geometry.h"
// #include "interfaces/ObstacleInterface.h"
#include "obstacles/OrientedBoxObstacle.h"

namespace Graphing {

Graph::Graph() {
	// TODO Auto-generated constructor stub

}

Graph::~Graph() {
	// TODO Auto-generated destructor stub
}



ClipperLib::Path vectorToPath(std::vector<Util::Point> pts, double scale)
{
	ClipperLib::Path pg;
	for (size_t i = 0; i < pts.size(); i++)
	{
		pg.push_back(ClipperLib::IntPoint((ClipperLib::cInt)(pts[i].x * scale), (ClipperLib::cInt)(pts[i].z * scale)));
	}
	return pg;
}

std::vector<Util::Point> clipperPointsToPoints(ClipperLib::Path pts, double scale)
{
	std::vector<Util::Point> out;
	for (size_t i = 0; i < pts.size(); i++)
	{
		out.push_back(Util::Point(pts[i].X/scale, 0, pts[i].Y/scale));
	}
	return out;
}

std::vector<Util::Point> createCircle(size_t points, double radius)
{
	std::vector<Util::Point> pts;
	float angleIncrement = (2 * M_PI) / (float)points;
	// angleIncrement = 90.0f;
	Util::Point turningVector = Util::Point(1 * radius, 0.0, 0 * radius);
	// point nextTurningVector = rotateInXZPlane(turningVector, angleIncrement);
	// std::cout << "Point is: (" << turningVector.X << ", " << turningVector.Y << ")" << std::endl;
	pts.push_back(turningVector);
	for (size_t i = 0; i < (points - 1); i++)
	{
		// DrawLib::drawLine(loc + turningVector, loc + nextTurningVector, color);
		turningVector = rotateInXZPlane(turningVector, -angleIncrement);
		// nextTurningVector = rotateInXZPlane(turningVector, angleIncrement);
		// std::cout << "Point is: (" << turningVector.X << ", " << turningVector.Y << ")" << std::endl;
		pts.push_back(turningVector);

	}

	// turningVector = Util::Point(1 * radius, 0.0, 0 * radius);
	// pts.push_back(turningVector);
	return pts;
}

void Graph::clearMinkowskiSums()
{
	_obstacles.clear();
	_intersections.clear();
}

void Graph::computeMinkowskiSums(double radius)
{
	this->clearMinkowskiSums();
	bool debugDraw = false;
	ClipperLib::Clipper c;
	// ClipperLib::Paths cp;
	ClipperLib::Paths subject, clip, solution, solution2;
	// typedef boost::geometry::model::box<point_2d> box_2d;
	double scale = 1000000.0;
	// std::vector<ClipperLib::Paths> obstaclesOriginal, interections;
	// double radius = 0.5;
	ClipperLib::Path cir = vectorToPath(createCircle(24, radius), scale);
	clip.clear();
	clip.push_back(cir);

	this->_obstacles.resize(edges.size());
	for (size_t edge = 0; edge < this->edges.size(); edge++)
	{
		this->_updateEdge(edge, 0.5);
	}
/*
	std::vector<SteerLib::OrientedBoxObstacle> obs;
	for (size_t edge = 0; edge < this->edges.size(); edge++)
	{
		Eigen::Vector3d cp_ = (this->nodes.at(this->edges[edge]._origin) + this->nodes.at(this->edges[edge]._end)) / 2.0;
		Util::Point cp(cp_(0), cp_(1), cp_(2));
		Eigen::Vector3d edge_ = (this->nodes.at(this->edges[edge]._end) - this->nodes.at(this->edges[edge]._origin)).normalized();
		double length = (this->nodes.at(this->edges[edge]._origin) - this->nodes.at(this->edges[edge]._end)).norm();
		double theta = std::atan2(edge_(2), edge_(0)) * 180.0 / M_PI;
		// world->addOrientedObstacle(cp, length, 0.1, 0.0, 1.0, -theta);
		SteerLib::OrientedBoxObstacle box(cp, length, 0.1,	0.0, 1.0, -theta);
		obs.push_back(box);
	}


	for (size_t o = 0; o < obs.size(); o++)
	{
		std::vector<Util::Point> obstaclePoints = obs[o].get2DStaticGeometry();
		ClipperLib::Path sol = vectorToPath(obstaclePoints, scale);
		subject.clear();
		subject.push_back(sol);
		solution2.clear();
		MinkowskiSum(subject[0], clip[0], solution2, true);
		_obstacles.push_back(solution2);


	}
*/
}

void Graph::updateMinkowskiSums(std::set<size_t> nodes_, double radius)
{

	for (size_t e=0; e < edges.size(); e++)
	{
		if ( (nodes_.find(edges[e]._origin) != nodes_.end()) ||
				(nodes_.find(edges[e]._end) != nodes_.end()) )
		{ // One of the edge nodes in the list of nodes should be updated
			this->_updateEdge(e, radius);
			// std::cout << "Updated edge" << std::endl;
		}
	}

}

/*
ClipperLib::Path Graph::_getEdgeGeometry(size_t edge, double scale)
{
	Eigen::Vector3d cp_ = (this->nodes.at(this->edges[edge]._origin) + this->nodes.at(this->edges[edge]._end)) / 2.0;
	Util::Point cp(cp_(0), cp_(1), cp_(2));
	Eigen::Vector3d edge_ = (this->nodes.at(this->edges[edge]._end) - this->nodes.at(this->edges[edge]._origin)).normalized();
	double length = (this->nodes.at(this->edges[edge]._origin) - this->nodes.at(this->edges[edge]._end)).norm();
	double theta = std::atan2(edge_(2), edge_(0)) * 180.0 / M_PI;
	// world->addOrientedObstacle(cp, length, 0.1, 0.0, 1.0, -theta);
	SteerLib::OrientedBoxObstacle box(cp, length, 0.1, 0.0, 1.0, -theta);

	std::vector<Util::Point> obstaclePoints = box.get2DStaticGeometry();
	ClipperLib::Path sol = vectorToPath(obstaclePoints, scale);
	return sol;
}
*/

ClipperLib::Path Graph::_getEdgeGeometry(size_t edge, double scale)
{

	std::vector<Util::Point> obstaclePoints;
	Eigen::Vector3d v0 = this->nodes[this->edges[edge]._origin];
	Eigen::Vector3d v1 = this->nodes[this->edges[edge]._end];
	obstaclePoints.push_back(Util::Point(v0(0), v0(1), v0(2)));
	obstaclePoints.push_back(Util::Point(v1(0), v1(1), v1(2)));
	ClipperLib::Path sol = vectorToPath(obstaclePoints, scale);
	return sol;
}

void Graph::_updateEdge(size_t edge, double radius)
{
	ClipperLib::Clipper c;
	// ClipperLib::Paths cp;
	ClipperLib::Paths subject, clip, solution, solution2;
	// typedef boost::geometry::model::box<point_2d> box_2d;
	double scale = 1000000.0;
	// std::vector<ClipperLib::Paths> obstaclesOriginal, interections;
	// double radius = 0.5;
	ClipperLib::Path cir = vectorToPath(createCircle(24, radius), scale);
	clip.clear();
	clip.push_back(cir);

	ClipperLib::Path sol = this->_getEdgeGeometry(edge, scale);
	subject.clear();
	subject.push_back(sol);
	solution2.clear();
	MinkowskiSum(subject[0], clip[0], solution2, true);
	this->_obstacles[edge] = solution2;
}

bool _edgesAttached(Edge e1, Edge e2)
{
	if ((e1._origin == e2._end) ||
		(e1._origin == e2._origin) ||
		(e1._end == e2._origin) ||
		(e1._end == e2._end))
	{
		return true;
	}
	return false;
}

double Graph::computeMinkowskiSumIntersectionArea()
{
	_intersections.clear();

	bool debugDraw = false;
	ClipperLib::Clipper c;
	// ClipperLib::Paths cp;
	ClipperLib::Paths subject, clip, solution, solution2;
	double scale = 1000000.0;
	double areaSum = 0;

	for (size_t i=0 ; i < _obstacles.size(); i++)
	{
		// ClipperLib::Paths obstaclePointsA = obstacles[i];
		// subject.clear();
		subject = _obstacles[i];
		
		for (size_t j = i + 1; j < _obstacles.size(); j++)
		{
			if (_edgesAttached(this->edges[i], this->edges[j]))
			{
				// std::cout << "Found edge pair " << this->edges[i] << ", " << this->edges[j] << std::endl;
				continue;
			}
			
			clip.clear();
			clip = _obstacles[j];

			c.AddPaths(subject, ClipperLib::ptSubject, true);
			c.AddPaths(clip, ClipperLib::ptClip, true);
			if (!c.Execute(ClipperLib::ctIntersection, solution))
			{
				std::cout << ("Intersection failed!\n\n");
				// return 1;
			}
			if (solution.size() > 0)
			{
				double area = Area(solution[0]) / (scale*scale);
				areaSum += area;
				// std::cout << "Area of random Polygon is " << area << std::endl;
				if (debugDraw)
				{
					/*
					interections.push_back(solution);
					SteerLib::PolygonObstacle * p1 = new SteerLib::PolygonObstacle(clipperPointsToPoints(solution[0], scale));
					this->_driver->getEngine()->addObstacle(p1);
					this->_driver->getEngine()->getSpatialDatabase()->addObject(p1, p1->getBounds());
					p1->setColour(Util::Color(1.0, 0.0, 0.0));
					*/
				}
				_intersections.push_back(solution);
			}
			else
			{
				// std::cout << "No intersection found" << std::endl;
			}
			c.Clear();
		}
		//std::cout << pMin << std::endl << pMax << std::endl;
		//obstacleList.push_back(Obstacle(pMin, pMax, 0, ObstacleType::BLOCK)); //this is for axis aligned box
	}
	return areaSum;
}

double Graph::edgeTheta(size_t edge)
{
	Eigen::Vector3d edge_ = (nodes.at(edges[edge]._end) - nodes.at(edges[edge]._origin)).normalized();
	// double length = (tmp_graph.nodes.at(tmp_graph.edges[edge]._origin) - tmp_graph.nodes.at(tmp_graph.edges[edge]._end)).norm();
	double theta = std::atan2(edge_(2), edge_(0)) * (180.0 / M_PI);
	return theta;
}

double Graph::alignmentPenalty()
{
	/*
	This should be weighted by the distances away from each other.
	Obstacle that are farther away from each other should have less
	affect on this metric.
	*/
	// std::vector<SteerLib::ObstacleInterface*> obstacleList(obs.begin(), obs.end());
	double thetaSum = 0;
	for (size_t i = 0; i < edges.size(); i++)
	{
		// SteerLib::ObstacleInterface* o = (*itr);
		double theta1 = edgeTheta(i);
		for (size_t j = i; j < edges.size(); j++)
		{
			double theta2 = edgeTheta(j);
			double diff = std::fabs(theta2 - theta1);
			thetaSum += std::fmod(diff, M_PI_OVER_2);	
		}
	}
	return thetaSum;
}

double Graph::sumWallLengths()
{
	double lengthSum = 0;
	for (size_t i = 0; i < edges.size(); i++)
	{
		Edge e = this->edges[i];
		lengthSum += (this->nodes[e._origin] - this->nodes[e._end]).squaredNorm();
	}
	return lengthSum;
}

void Graph::saveSVGToFile(std::string filename, OptimizationParameters params)
{
	double scale = 1000000.0;
	SVGBuilder svg;
	svg.style.penWidth = 0.750;
	svg.style.brushClr = 0x12000000;
	svg.style.penClr = 0xDF0A0A0A;
	svg.style.pft = ClipperLib::pftEvenOdd;
	for (size_t i = 0; i < this->edges.size(); i++)
	{
		ClipperLib::Path path = _getEdgeGeometry(i, scale);
		ClipperLib::Paths paths;
		paths.push_back(path);
		svg.AddPaths(paths);
	}
	svg.style.penWidth = 0.750;
	svg.style.brushClr = 0x129C0000;
	svg.style.penClr = 0xCCFFA07A;
	svg.style.pft = ClipperLib::pftEvenOdd;
	for (size_t i = 0; i < this->_obstacles.size(); i++)
	{
		svg.AddPaths(this->_obstacles[i]);
	}

	svg.style.brushClr = 0x12009C00;
	svg.style.penClr = 0xCC88FF7A;
	svg.style.pft = ClipperLib::pftEvenOdd;
	for (size_t i = 0; i < this->_intersections.size(); i++)
	{
		svg.AddPaths(this->_intersections[i]);
	}
	
	svg.style.penWidth = 0.00;
	svg.style.brushClr = 0x6080ff9C;
	svg.style.penClr = 0xFF003300;
	svg.style.pft = ClipperLib::pftNonZero;
	for (size_t i = 0; i < params._refRegions.size(); i++)
	{// Works for axis-aligned bounding points
		std::vector<Util::Point> path_;
		Util::Point minPoint = params._refRegions[i][0];
		Util::Point maxPoint = params._refRegions[i][1];
		path_.push_back(minPoint);
		path_.push_back(Util::Point(minPoint.x, 0.0, maxPoint.z));
		path_.push_back(maxPoint);
		path_.push_back(Util::Point(maxPoint.x, 0.0, minPoint.z));
		ClipperLib::Path path = vectorToPath(path_, scale);
		
		
		ClipperLib::Paths paths;
		paths.push_back(path);
		svg.AddPaths(paths);
	}

	svg.style.brushClr = 0xcc60809C;
	svg.style.penClr = 0xDD330000;
	svg.style.pft = ClipperLib::pftNonZero;
	for (size_t i = 0; i < params._queryRegions.size(); i++)
	{// Works for axis-aligned bounding points
		std::vector<Util::Point> path_;
		Util::Point minPoint = params._queryRegions[i][0];
		Util::Point maxPoint = params._queryRegions[i][1];
		path_.push_back(minPoint);
		path_.push_back(Util::Point(minPoint.x, 0.0, maxPoint.z));
		path_.push_back(maxPoint);
		path_.push_back(Util::Point(maxPoint.x, 0.0, minPoint.z));
		ClipperLib::Path path = vectorToPath(path_, scale);


		ClipperLib::Paths paths;
		paths.push_back(path);
		svg.AddPaths(paths);
	}
	
	svg.SaveToFile(filename, 1.0 / scale, 1.0);
}


void Graph::saveSVGToFile(std::string filename)
{
	double scale = 1000000.0;
	SVGBuilder svg;
	svg.style.penWidth = 0.75;
	svg.style.brushClr = 0x1200009C;
	svg.style.penClr = 0xCCD3D3DA;
	svg.style.pft = ClipperLib::pftEvenOdd;
	for (size_t i = 0; i < this->edges.size(); i++)
	{
		ClipperLib::Path path = _getEdgeGeometry(i, scale);
		ClipperLib::Paths paths;
		paths.push_back(path);
		svg.AddPaths(paths);
	}
	svg.style.penWidth = 0.75;
	svg.style.brushClr = 0x129C0000;
	svg.style.penClr = 0xCCFFA07A;
	svg.style.pft = ClipperLib::pftEvenOdd;
	for (size_t i = 0; i < this->_obstacles.size(); i++)
	{
		svg.AddPaths(this->_obstacles[i]);
	}
	/*
	svg.style.brushClr = 0x6080ff9C;
	svg.style.penClr = 0xFF003300;
	svg.style.pft = ClipperLib::pftNonZero;
	for (size_t i = 0; i < interections.size(); i++)
	{
		svg.AddPaths(interections[i]);
	}
	*/
	svg.SaveToFile(filename, 1.0 / scale, 1.0);
}



void Graph::saveToFile(std::string filename)
{
	std::ofstream outfile(filename, std::ifstream::out);
	if (!outfile.eof() && (outfile.fail() || outfile.bad()))
	{
		std::cout << "error opening file graph data file " << filename
			<< " error: " << strerror(errno) << std::endl;
		// return;
	}

	// this->nodes.push_back(Eigen::Vector3d(x, y, z));
	for (size_t n=0; n < this->nodes.size(); n++)
	{
		outfile << "v " << this->nodes[n](0) << " " << this->nodes[n](1) << " " << this->nodes[n](2) <<  std::endl;
	}
	outfile << std::endl;
	for (size_t e=0; e < this->edges.size(); e++)
	{
		outfile << "e " << this->edges[e]._origin << " " << this->edges[e]._end  << std::endl;
	}

}

void Graph::importFromFile(std::string filename)
{
	this->edges.clear();
	this->nodes.clear();
	std::ifstream infile(filename, std::ifstream::in);

	if (!infile.eof() && (infile.fail() || infile.bad()))
	{
		std::cout << "error reading file graph data file " << filename
			<< " error: " << strerror(errno) << std::endl;
		// return;
	}
	std::string line;
	float x, y, z;
	size_t e0, e1;
	char type;
	while (infile >> type)
	{

		if (type == 'v')
		{
			infile >> x >> y >> z;
			this->nodes.push_back(Eigen::Vector3d(x, y, z));
		}
		if (type == 'e')
		{
			infile >> e0 >> e1;
			if (e0 != e1)
			{
				this->edges.push_back(Edge(e0, e1));
			}
			else
			{
				std::cout << "Error: Found an edge that has origin == end" << std::endl;
			}
		}
	}
}

void Graph::initFrom(Graph g)
{
	this->nodes.clear();
	this->edges.clear();
	for (size_t v=0; v < g.nodes.size(); v++)
	{
		this->nodes.push_back(g.nodes[v]);
	}
	for (size_t e=0; e < g.edges.size(); e++)
	{
		Edge edge(g.edges[e]._origin, g.edges[e]._end);
		this->edges.push_back(edge);
	}
	for (size_t o=0; o < g._obstacles.size(); o++)
	{
		this->_obstacles.push_back(g._obstacles[o]);
	}
}

} /* namespace Graphing */
