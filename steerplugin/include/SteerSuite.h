/*
 * SteerSuite.h
 *
 *  Created on: Oct 6, 2015
 *      Author: Glen
 */

#ifndef STEERPLUGIN_INCLUDE_STEERSUITE_H_
#define STEERPLUGIN_INCLUDE_STEERSUITE_H_

#include "SimWorld.h"
#include "SteerPlugin.h"
#include "spacesyntax.h"
#include "interfaces/EngineControllerInterface.h"
#include <iostream>
#include "testcaseio/Behaviour.h"
#include "interfaces/ModuleInterface.h"



namespace SteerSuite {

enum STEERPLUG_API MetricName : int
{
	MeanDegree = 1,
	MeanTreeDepth = 2,
	MeanTreeEntropy = 3
};

class STEERPLUG_API SteerSuite {
public:
	SteerSuite();
	SteerSuite(std::string configFileName);
	virtual ~SteerSuite();

	virtual double simulate();
	virtual double simulate(const double *x, const size_t length);
	virtual double simulateSteeringAlgorithm(const double *x, const size_t length, SteerLib::Behaviour behave);
	// Init the simulation without Rendering GLFW window
	virtual void init(SimWorld * world);
	// Init with a GLWF window
	virtual void initRender(SimWorld * world);
	// virtual void init(SimWorld * world, SteerLib::SimulationOptions &);
	virtual LogData * getSimData();
	virtual void finish();
	/// Basically removes everything from the running simulation
	virtual void resetSimulation();
	/// Basically removes everything from the running simulation
	virtual void restartSimulation(SimWorld * world);
	virtual void loadSimulation();
	virtual void unloadSimulation();

	virtual void setConfigFileName(std::string configFileName)
	{
		this->_configFileName = configFileName;
	}
	virtual const SteerLib::OptionDictionary & getModuleOptions(const std::string moduleName);

	virtual void setSteeringAlgorithmBehaviour(SteerLib::Behaviour behave);

	/*
		Geometry stuff
	*/
	/*
	virtual double computeIntersections(bool debugDraw);
	virtual double alignmentPenalty();
	*/

	// spacesyntax wrappers
	
	// create the grid plan and set some initial values
	virtual void init_visibilityGraph(Util::Point pMin, Util::Point pMax, unsigned int gridNumX, unsigned int gridNumZ, float height = 0);
	
	// add query nodes in the given region based on the current grid size
	virtual void add_visibilityRegion(Util::Point pMin, Util::Point pMax);

	// add query nodes in the given region based on the current grid size
	virtual void add_queryRegion(Util::Point pMin, Util::Point pMax);

	// add reference nodes in the given region based on the current grid size
	virtual void add_refRegion(Util::Point pMin, Util::Point pMax);

	// add one visibility node, regardless of the grid
	virtual void add_visibilityPoint(Util::Point point);

	// clear all visibility nodes (you can add again, no need to init_visibilityGraph)
	virtual void clear_visibilityGraph();

	// setup and calculate the visibility graph, given current state of the world and the visibility nodes
	virtual void setup_visibilityGraph();
	virtual void generate_visibilityGraph_cpu() { _visibilityGraph->generate_graph(); }
#ifdef CUDA_ENABLED
	virtual void generate_visibilityGraph_gpu() { _visibilityGraph->generate_graph_cuda(); }
	virtual void fetch_visibilityGraph_gpu() { _visibilityGraph->fetch_graph_cuda(); }
	virtual void output_visibilityGraph_gpu(float& mDegree, float& mDepth, float& mEntropy) { _visibilityGraph->get_output_cuda(mDegree, mDepth, mEntropy); }
#endif

	// return average node degree of the visibility graph over all current visibility nodes
	virtual float get_meanDegree();
#ifdef CUDA_ENABLED
	virtual float get_meanDegree_gpu() { return _visibilityGraph->get_meanDegree_cuda(); }
#endif
	virtual float get_meanTreeDepth(); // legacy: to be removed
	virtual float get_meanTreeEntropy(); // legacy: to be removed
	virtual std::vector<float> get_meanTreeProp(); // use this method for parallel and efficient results: returns [depth, entropy]
	virtual bool check_connection() { return _visibilityGraph->check_connection(); } // returns true if connected graph, false otherwise (nodes under walls are ignored)

	// return location of all visibility nodes (the indices can be used to get corresponding degrees from get_nodeDegrees)
	virtual std::vector<Util::Point> get_visibilityNodes();
	virtual std::vector<Util::Point> get_queryNodes();
	virtual std::vector<Util::Point> get_refNodes();

	// return the 4 points of a cell containing the point specified by index (topLeft, botLeft, botRight, topRight) - MORE EFFICIENT
	virtual std::vector<Util::Point> get_cellPoints(unsigned int index) { return _visibilityGraph->get_cellPoints(index); }

	// return the 4 points of a cell containing the point specified by location - LESS EFFICIENT
	virtual std::vector<Util::Point> get_cellPoints(Util::Point target) { return _visibilityGraph->get_cellPoints(target); }

	// return a 2D vector, where each column is consisted of two connected points in the visibility graph
	virtual std::vector<std::vector<Util::Point>> get_visibilityLines();

	// return the metrics of the node identified by the index (find the index based on the results of get_visibilityNodes) - MORE EFFICIENT
	virtual float get_nodeDegree(unsigned int index);
	virtual float get_nodeTreeDepth(unsigned int index); // legacy: to be removed
	virtual float get_nodeTreeEntropy(unsigned int index); // legacy: to be removed
	virtual std::vector<float> get_nodeTreeProp(unsigned int index); // use this method for parallel and efficient results: returns [depth, entropy]

	// return the metrics of the node identified by the location - LESS EFFICIENT: logN
	virtual float get_nodeDegree(Util::Point target);
	virtual float get_nodeTreeDepth(Util::Point target); // legacy: to be removed
	virtual float get_nodeTreeEntropy(Util::Point target); // legacy: to be removed
	virtual std::vector<float> get_nodeTreeProp(Util::Point target); // use this method for parallel and efficient results: returns [depth, entropy]

public:
	SimWorld * _world;
	SteerLib::EngineControllerInterface * _driver;
	SteerLib::SimulationOptions * _simulationOptions;
	SpaceSyntax::VisibilityGraph* _visibilityGraph;
	std::string _configFileName;
};

} /* namespace SteerSuite */

#endif /* STEERPLUGIN_INCLUDE_STEERSUITE_H_ */
