//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


/// @file steersim/src/Main.cpp
/// @brief Entry point of SteerSim.

#include "SteerSuite.h"
#include "SimWorld.h"
#include "Graph.h"
#include "OptimizationParameters.h"
#include "RoundRobinOptimization.h"

/*
Run with
/SteerSuite/build/bin
$ ./steerpluginTester.exe ../../testcases/simpleRoomGraph.graph

*/
int graphInit(std::string graphFileName)
{

	Graphing::Graph graph;
	
	graph.importFromFile(graphFileName);

	std::stringstream filename;
	size_t m = 0;
	filename << "../results/testOut" << m << ".graph";
	graph.saveToFile(filename.str());
	filename.str(std::string());
	filename << "../results/testOut" << m << ".svg";
	// graph.computeMinkowskiSums(0.5);
	// graph.clearMinkowskiSums();
	graph.saveSVGToFile(filename.str());
	
	SteerSuite::SimWorld * world = new SteerSuite::SimWorld();

	for (size_t edge = 0; edge < graph.edges.size(); edge++)
	{
		Eigen::Vector3d cp_ = (graph.nodes.at(graph.edges[edge]._origin) + graph.nodes.at(graph.edges[edge]._end)) / 2.0;
		Util::Point cp(cp_(0), cp_(1), cp_(2));
		Eigen::Vector3d edge_ = (graph.nodes.at(graph.edges[edge]._end) - graph.nodes.at(graph.edges[edge]._origin)).normalized();
		double length = (graph.nodes.at(graph.edges[edge]._origin) - graph.nodes.at(graph.edges[edge]._end)).norm();
		double theta = std::atan2(edge_(2), edge_(0)) * 180.0 / M_PI;
		world->addOrientedObstacle(cp, length, 0.1, 0.0, 1.0, -theta);
	}

	SteerSuite::SteerSuite steerSuite;
	steerSuite.initRender(world);

	graph.computeMinkowskiSums(0.5);
	double area = graph.computeMinkowskiSumIntersectionArea();
	std::cout << "Area of Minkowski intersection is: " << area << std::endl;
	double alignment = graph.alignmentPenalty();
	std::cout << "Alignment Penalty: " << alignment << std::endl;

	steerSuite.simulate();
	

	steerSuite.finish();
	return EXIT_SUCCESS;
}

int oldInit()
{
	Util::Point gridBound1(-100, 0, -100), gridBound2(100, 0, 100);
	Util::Point regionBound1(-20, 0, -30), regionBound2(-10, 0, -20);
	float gridNumX = 100, gridNumZ = 100, height = 0, meanDegree = 0;
	std::vector<Util::Point> pointsOfInterest;

	size_t aabb_size = 6 * 6; // 6 values for 5 items.
	double * aabbs = new double[aabb_size];
	double stuff[] = { -16.2806451612904, -16.0806451612903,0, 2,-41.6887096774194, 3.5112903225807,
		-16.1806451612903, 25.4193548387096,0, 2,-41.6887096774194, -41.4887096774194,
		25.2193548387096, 25.4193548387097,0, 2,-41.5887096774194, 3.51129032258058,
		4.71935483870968, 25.3193548387097,0, 2,3.31129032258058, 3.51129032258064,
		-16.1806451612903, 2.91935483870968,0, 2,3.31129032258064, 3.5112903225807,
		-3, -1, 0, 2, 1, 2
	};

	SteerSuite::SimWorld * world = new SteerSuite::SimWorld();

	for (size_t i = 0; i < aabb_size;)
	{
		world->addObstacle(stuff[i], stuff[i + 1], stuff[i + 2],
			stuff[i + 3], stuff[i + 4], stuff[i + 5]);
		i += 6;
	}

	SteerSuite::SteerSuite steerSuite;

	steerSuite.init(world);

	steerSuite.init_visibilityGraph(gridBound1, gridBound2, gridNumX, gridNumZ, height);
	steerSuite.add_visibilityRegion(regionBound1, regionBound2);
	steerSuite.setup_visibilityGraph();
	meanDegree = steerSuite.get_meanDegree();
	//meanDegree = steerSuite.get_meanDegree(pointsOfInterest);
	std::cout << ">>>Mean Degree = " << meanDegree << std::endl;
	std::vector<Util::Point> pointList = steerSuite.get_visibilityNodes();
	for (auto itr = pointList.begin(); itr != pointList.end(); itr++)
		std::cout << "POINTS_______" << *itr << "----------------------" << std::endl;
	std::vector<std::vector<Util::Point>> lineList = steerSuite.get_visibilityLines();
	for (auto itr = lineList.begin(); itr != lineList.end(); itr++)
		std::cout << (*itr)[0] << "__" << (*itr)[1] << std::endl;
	steerSuite.simulate();
	steerSuite.finish();

	return 0;
}

int main(int argc, char **argv)
{
	// save the original cerr streambuf, so that we can restore it on an exception.
	std::string graphFileName(argv[1]);

	int initGraph = 0;

	if (initGraph == 0)
	{
		return graphInit(graphFileName);
	}
	else
	{
		return oldInit();
	}
	
}
