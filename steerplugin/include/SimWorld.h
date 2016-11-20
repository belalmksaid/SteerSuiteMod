/*
 * SimWorld.h
 *
 *  Created on: 2015-10-27
 *      Author: gberseth
 */

#ifndef SIMWORLD_H_
#define SIMWORLD_H_

#include <vector>
#include "interfaces/ObstacleInterface.h"
#include "interfaces/AgentInterface.h"
#include "SteerPlugin.h"
// #include "Graph.h"

namespace SteerSuite {

class STEERPLUG_API SimWorld {
public:
	SimWorld();
	virtual ~SimWorld();

	virtual void addObstacle(double xmin,double  xmax,double  ymin,double  ymax,double  zmin,double  zmax);
	virtual void addOrientedObstacle(Util::Point centerPosition, float lengthX, float lengthZ, float ymin, float ymax, float thetaY);
	virtual void addOrientedObstacle(std::vector<double> centerPosition, float lengthX, float lengthZ, float ymin, float ymax, float thetaY);
	// virtual void addGraph(Graphing::Graph graph);
	virtual void addAgent();
	virtual void addAgentRegion();
	virtual void clearWorld();
public:
	virtual std::vector<SteerLib::ObstacleInterface *> getObstacles()
	{
		return this->_obstacles;
	}

private:
	std::vector<SteerLib::ObstacleInterface* > _obstacles;
	std::vector<SteerLib::AgentInterface* > _agents;
	// std::vector<SteerLib::AgentRegion> _agentRegions;

	friend class SteerSuite;

};

}

#endif /* SIMWORLD_H_ */
