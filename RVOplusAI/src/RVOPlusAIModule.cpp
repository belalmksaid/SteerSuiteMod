//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#include <iostream>
#include <algorithm>
#include <string>
#include <assert.h>
#include "../include/RVOPlusAIModule.h"
#include "../include/RVOPlusAgent.h"
#include "../libs/AStarLite/AStarLite.h"
#include "../libs/AStarLite/GridAStar.h"
#include "../libs/RVO/vector2.h"

#include "SimulationPlugin.h"

#include "../../steerlib/include/Util/DrawLib.h"

using namespace std;
using namespace Util;
using namespace SteerLib;
using namespace RVO;

// find better values for these ...
const int ERROR_ID = -1;

// RVO sim constants
const int velSampleCount = 171;
const float neighborDist = 10.0f;
const int maxNeighbors = 30;
const float goalRadius = 0.5f;
const float extraSpeed = 0.5f;
const float safetyFactor = 2.0f;
const float maxAcc = 1.2f;
const float crazyCoor[2] = {-10000.0,-10000.0};

// fun constants
const float pi = acos(-1.0f);
const float degToRad = pi/360.0f;

// bottleneck constants
const Vector2 xVec(1.0f,0.0f);
const Vector2 zVec(0.0f,1.0f);
const float waiting = 0.05f;
const float noWaiting = 1.0f;
const int zDir = 0;
const int xDir = 1;

// agent types
const int standardType = 0; 
const int groupMetaType = 1;
const int obstMetaType = 2;

// group agents
const size_t numGroupAgents = 10;

// form agents
const size_t numFormAgents = 10;


// functions required to deal with DLLs
PLUGIN_API SteerLib::ModuleInterface* createModule()
{
	return new RVOPlusAIModule;
}


PLUGIN_API void destroyModule(SteerLib::ModuleInterface *module)
{
	if (module) delete module;
}


RVOPlusAIModule::RVOPlusAIModule() :
	m_RVOsimPtr(RVOSimulator::Instance()),
	m_numAgents(0)
{
	// make sure we built something
	assert(NULL != m_RVOsimPtr.get());
}


RVOPlusAIModule::~RVOPlusAIModule()
{}


int RVOPlusAIModule::addGoal(const Util::Point &goal)
{
	// try to find the point
	map<Point,int,pointCmp>::const_iterator it = m_goals.find(goal);
	if(it != m_goals.end())
		return it->second;

	// not in database, so add it
	const int goalID = m_RVOsimPtr->addGoal(Vector2(goal[0], goal[2]));
	m_goals.insert(make_pair(goal,goalID));

	// add this to reverse lookup database
	m_goalsReverse.insert(make_pair(goalID, goal));

	// return the new ID
	return goalID;
}


bool RVOPlusAIModule::findGoal(int id, Util::Point &p) const
{
	map<int,Point>::const_iterator it = m_goalsReverse.find(id);
	if(it != m_goalsReverse.end())
	{
		p = it->second;
		return true;
	}
	else
		return false;
}


int RVOPlusAIModule::findGoal(const Util::Point &goal) const
{
	// try to find the point
	map<Point,int,pointCmp>::const_iterator it = m_goals.find(goal);
	if(it != m_goals.end())
		return it->second;
	
	assert(false);
	return ERROR_ID;
}


void RVOPlusAIModule::addBoxObst(const BoxObstacleInitialConditions &o, float maxAgentRad)
{
	// draw 4 line segments to form a box
	m_RVOsimPtr->addObstacle(Vector2(o.xmin,o.zmin), Vector2(o.xmin,o.zmax));
	m_RVOsimPtr->addObstacle(Vector2(o.xmax,o.zmin), Vector2(o.xmax,o.zmax));
	m_RVOsimPtr->addObstacle(Vector2(o.xmin,o.zmax), Vector2(o.xmax,o.zmax));
	m_RVOsimPtr->addObstacle(Vector2(o.xmin,o.zmin), Vector2(o.xmax,o.zmin));

	// make the roadmap vertexes
	int id1 = m_RVOsimPtr->addRoadmapVertex(Vector2(o.xmin-maxAgentRad,o.zmin-maxAgentRad));
	int id2 = m_RVOsimPtr->addRoadmapVertex(Vector2(o.xmin-maxAgentRad,o.zmax+maxAgentRad));
	int id3 = m_RVOsimPtr->addRoadmapVertex(Vector2(o.xmax+maxAgentRad,o.zmax+maxAgentRad));
	int id4 = m_RVOsimPtr->addRoadmapVertex(Vector2(o.xmax+maxAgentRad,o.zmin-maxAgentRad));

	// draw maps clockwise and counter-clockwise
	m_RVOsimPtr->addRoadmapEdge(id1, id2);
	m_RVOsimPtr->addRoadmapEdge(id2, id3);
	m_RVOsimPtr->addRoadmapEdge(id3, id4);
	m_RVOsimPtr->addRoadmapEdge(id4, id1);

	m_RVOsimPtr->addRoadmapEdge(id1, id4);
	m_RVOsimPtr->addRoadmapEdge(id4, id3);
	m_RVOsimPtr->addRoadmapEdge(id3, id2);
	m_RVOsimPtr->addRoadmapEdge(id2, id1);
}


bool RVOPlusAIModule::clearPath(const Util::Point &start, const Util::Point &goal, float agentRad) const
{
	// make offsets on each side to account for agent radius
	float rad = atan2(goal.z-start.z, goal.x-start.x) + 90.0f*degToRad;
	Vector off(agentRad*cos(rad), 0, agentRad*sin(rad));

	// if can see to both then OK
	return (m_spDb->hasLineOfSight(start+off,goal+off,NULL,NULL) && m_spDb->hasLineOfSight(start-off,goal-off,NULL,NULL));
}

void RVOPlusAIModule::init(const SteerLib::OptionDictionary &options, SteerLib::EngineInterface *engineInfo)
{
	// MUBBASIR 
	//assert(NULL != engineInfo && NULL != engineInfo->m_testCase);
	assert(NULL != engineInfo);

	// save engine pointer for later
	m_engineInfo = engineInfo;

	std::cout << " rvo init \n";

}

void RVOPlusAIModule::resetRVOSimulator ()
{
	m_RVOsimPtr->resetSimulation ();
}


void RVOPlusAIModule::preprocessObstacles (std::vector<SteerLib::BoxObstacle> & obstacles)
{
	// add all the obstacles

	std::vector<SteerLib::BoxObstacle>::iterator obstacleIter;
	for (obstacleIter = obstacles.begin(); obstacleIter != obstacles.end(); ++obstacleIter) 
	{
		//_simulationWriter->addObstacleBoundingBox((*obstacleIter)->getBounds());
		BoxObstacleInitialConditions oi;
		oi.xmax = (*obstacleIter).getBounds().xmax;
		oi.xmin = (*obstacleIter).getBounds().xmin;
		oi.ymax = (*obstacleIter).getBounds().ymax;
		oi.ymin = (*obstacleIter).getBounds().ymin;
		oi.zmax = (*obstacleIter).getBounds().zmax;
		oi.zmin = (*obstacleIter).getBounds().zmin;

		// add that obstacle to simulation
		std::cout << " adding obstacle \n";
		addBoxObst(oi, 0.5f);
	}

}


void  RVOPlusAIModule::preprocessAgents (const std::vector<SteerLib::AgentInitialConditions> &agents)
{
	// save the number of agents
	m_numAgents = agents.size ();

	m_agentIDs.clear ();
	m_agentGoalSeries.clear ();


	std::cout << " number of agents " << m_numAgents << "\n";

	// setup A* parameters for test case
	m_spDb = m_engineInfo->getSpatialDatabase();
	AStarLite searcher;
	GridEnvironment gridEnv(m_spDb);


	// add all the agents
	vector<int> formMembers;
	float maxAgentRad = 0;
	vector<goalInfo> goalSeries;
	for(int id = 0; id < m_numAgents; ++id)
	{
		cout << "A* planning for agent # " << id << endl;

		// point to the agent's description
		const AgentInitialConditions &currAgentPtr = agents[id];

		// clear out the goal series
		goalSeries.clear();

		// add all the goals to the database
		int startIndex, goalIndex;
		float theta;
		Util::Point startPoint, goalPoint;
		vector<Util::Point> wayPoints, tempPoints;
		vector<AgentGoalInfo>::const_iterator lastI = currAgentPtr.goals.begin();
		for(vector<AgentGoalInfo>::const_iterator i = currAgentPtr.goals.begin(); i != currAgentPtr.goals.end(); ++i)
		{
			// last point is either previous goal or start
			startPoint = (lastI != i) ? lastI->targetLocation : currAgentPtr.position;

			// if this is a random goal fill it in
			if(false == i->targetIsRandom)
				goalPoint = i->targetLocation;
			else
			{		
				std::cerr << "RVOPlusAI currently does not support random goals \n";
				//const AxisAlignedBox &worldBox = m_engineInfo->m_testCase->getWorldBounds();
				//goalPoint = m_spDb->randomPositionInRegionWithoutCollisions(worldBox,currAgentPtr.radius,false);
			}

			// do the A* search
			startIndex = m_spDb->getCellIndexFromLocation(startPoint[0],startPoint[2]);
			goalIndex = m_spDb->getCellIndexFromLocation(goalPoint[0], goalPoint[2]);
			bool worked = searcher.findPath(gridEnv, startIndex, goalIndex);
			assert(worked == true);

			// convert the indexes to locations
			tempPoints.clear();
			const vector<int> &result = searcher.getPath();
			Point tempPt;
			for(vector<int>::const_iterator ii = result.begin(); ii != result.end(); ++ii)
			{
				m_spDb->getLocationFromIndex(*ii,tempPt);
				tempPoints.push_back(tempPt);
			}

			// remove any extra points we don't need
			wayPoints.clear();
			wayPoints.push_back(tempPoints.front());
			for(size_t p = 1; p < tempPoints.size()-1; ++p)
			{
				Util::Vector dist(wayPoints.back() - tempPoints[p+1]);
				if(false == clearPath(wayPoints.back(), tempPoints[p+1], currAgentPtr.radius) /*|| dist.length() > 3.0f*/)
					wayPoints.push_back(tempPoints[p]);
			}

			// add all waypoints
			goalInfo info;
			int lastID = -1;
			reverse(wayPoints.begin(),wayPoints.end());
			for(vector<Util::Point>::const_iterator j = wayPoints.begin(); j != wayPoints.end(); ++j)
			{
				// save all the information
				info.m_desiredSpeed = i->desiredSpeed;
				info.m_goal = false;
				info.m_loc = *j;
				info.m_id = m_RVOsimPtr->addRoadmapVertex(Vector2(j->x,j->z));

				// push that onto back of goal series
				goalSeries.push_back(info);

				// if had previous roadmap vertex, connect it
				if(-1 != lastID)
					m_RVOsimPtr->addRoadmapEdge(lastID, info.m_id);

				// update the last roadmap vertex ID
				lastID = info.m_id;
			}

			// add goal itself (don't add to roadmap because it confuses the RVO sim)
			info.m_desiredSpeed = i->desiredSpeed;
			info.m_goal = true;
			info.m_id = addGoal(goalPoint);
			info.m_loc = goalPoint;
			goalSeries.push_back(info);

			// save pointer to this location
			lastI = i;
		}

		// update the max agent rad
		maxAgentRad = max(currAgentPtr.radius, maxAgentRad);

		// figure out direction we are pointing
		theta = atan2(currAgentPtr.direction.z,currAgentPtr.direction.x);

		// add agent
		int agentID = m_RVOsimPtr->addAgent(
			Vector2(currAgentPtr.position.x,currAgentPtr.position.z),
			goalSeries.back().m_id,
			velSampleCount,
			neighborDist,
			maxNeighbors,
			currAgentPtr.radius,
			goalRadius,
			currAgentPtr.goals.front().desiredSpeed,
			currAgentPtr.goals.front().desiredSpeed + extraSpeed,
			safetyFactor,
			maxAcc,
			Vector2(cos(theta)*currAgentPtr.speed,sin(theta)*currAgentPtr.speed),
			theta,
			standardType);

		// add that agent ID to list
		m_agentIDs.push_back(agentID);

		// add agent's goal series to list
		m_agentGoalSeries.push_back(goalSeries);

		if(id < 10)
		   formMembers.push_back(agentID);
	}

	m_RVOsimPtr->setFormAgentMembers(formMembers);

	// reverse the two vectors so the agents start correctly
	reverse(m_agentIDs.begin(),m_agentIDs.end());
	reverse(m_agentGoalSeries.begin(),m_agentGoalSeries.end());

}
void RVOPlusAIModule::initializeSimulation()
{

	std::cout << "rvo initalize simulation IS CURRENTLY BROKEN\n";

	// do not automatically make edges between waypoints
	m_RVOsimPtr->setRoadmapAutomatic(-1);

	// MUBBASIR 
	//_preprocessObstacles ();
	//std::cout << "obstacles added \n";

	/*
	const int numObs = static_cast<int>(m_engineInfo->m_testCase->getNumObstacles());
	for(int id = 0; id < numObs; ++id)
	{
		cout << "Added obstacle # " << id << endl;

		// get the next object
		const ObstacleInitialConditions &currObsPtr = m_engineInfo->m_testCase->getObstacleInitialConditions(id);

		// add that obstacle to simulation
		addBoxObst(currObsPtr, 0.5f);
	}
	*/


	// find all the bottlenecks
	// MUBBASIR TODO 
	//findBottlenecks(*(m_engineInfo->m_testCase), 0.5f, 2.0f);

	// goal at removed location
	const int crazyGoalID = addGoal(Point(crazyCoor[0],0,crazyCoor[1]));

	// reserve group and form meta-agents
	m_RVOsimPtr->reserveMetaAgents(numGroupAgents, numFormAgents,
		Vector2(crazyCoor[0],crazyCoor[1]),
		crazyGoalID,
		velSampleCount,
		neighborDist,
		maxNeighbors,
		0.5f,
		goalRadius,
		0.5f,
		0.5f + extraSpeed,
		safetyFactor,
		maxAcc,
		Vector2(1.0f,0.0f),
		0.0f);

	// save vector of group agent IDs
	m_groupMetaAgentIDs = m_RVOsimPtr->getGroupMetaAgentIDs();
	m_formMetaAgentIDs = m_RVOsimPtr->getFormMetaAgentIDs();

	// add an obstacle meta-agent at each bottleneck
	for(size_t i = 0; i < m_bottlenecks.size(); ++i)
	{
		Vector2 bottleCenter((m_bottlenecks[i].first.xmax + m_bottlenecks[i].first.xmin)/2,
			(m_bottlenecks[i].first.zmax + m_bottlenecks[i].first.zmin)/2);

		float bottleRad = pow(m_bottlenecks[i].first.xmax - m_bottlenecks[i].first.xmin,2);
		bottleRad += pow(m_bottlenecks[i].first.zmax - m_bottlenecks[i].first.zmin,2);
		bottleRad = sqrt(bottleRad)/2;

		// add agent
		int agentID = m_RVOsimPtr->addAgent(
			bottleCenter,
			crazyGoalID,
			velSampleCount,
			neighborDist,
			50,
			bottleRad*1.5f,
			goalRadius,
			0.0f,
			0.0f,
			safetyFactor,
			0.0001f,
			Vector2(0.00001f,0.00001f),
			0.0f,
			obstMetaType);

		// add to vector of obsticle meta-agents
		m_obstMetaAgentIDs.push_back(agentID);

		// set as obsticle meta-agents
		m_RVOsimPtr->setObstAgent(agentID);
	}

	// add all agents 
	//_preprocessAgents ();


}

// MUBBASIR TODO 

/*
void RVOPlusAIModule::findBottlenecks(const SteerLib::TestCaseReader &testCase, float maxAgentRad, float sizeMulti)
{
	vector<float> xEnds(4);
	vector<float> zEnds(4);

	// for each obstacle
	for(int id1 = 0; id1 < static_cast<int>(testCase.getNumObstacles()); ++id1)
	{
		// get pointer to it
		const ObstacleInitialConditions &obsPtr1 = testCase.getObstacleInitialConditions(id1);

		// for every other obstacle
		for(int id2 = id1+1; id2 < static_cast<int>(testCase.getNumObstacles()); ++id2)
		{
			// get pointer to it
			const ObstacleInitialConditions &obsPtr2 = testCase.getObstacleInitialConditions(id2);

			// construct the box between the two obstacles
			ObstacleInitialConditions box;

			xEnds[0] = obsPtr1.xmax;
			xEnds[1] = obsPtr1.xmin;
			xEnds[2] = obsPtr2.xmax;
			xEnds[3] = obsPtr2.xmin;
			sort(xEnds.begin(),xEnds.end());
			box.xmin = xEnds[1];
			box.xmax = xEnds[2];

			zEnds[0] = obsPtr1.zmax;
			zEnds[1] = obsPtr1.zmin;
			zEnds[2] = obsPtr2.zmax;
			zEnds[3] = obsPtr2.zmin;
			sort(zEnds.begin(),zEnds.end());
			box.zmin = zEnds[1];
			box.zmax = zEnds[2];

			bool zBottleneck = false;
			if(box.zmax - box.zmin < sizeMulti*2.0f*maxAgentRad && box.zmax - box.zmin > 2.0f*maxAgentRad &&
				(true == inside(obsPtr1, (box.xmax+box.xmin)/2.0f, box.zmax+0.001f) || true == inside(obsPtr2, (box.xmax+box.xmin)/2.0f, box.zmax+0.001f)) &&
				(true == inside(obsPtr1, (box.xmax+box.xmin)/2.0f, box.zmin-0.001f) || true == inside(obsPtr2, (box.xmax+box.xmin)/2.0f, box.zmin-0.001f)))
			{
				zBottleneck = true;
			}
			
			bool xBottleneck = false;
			if(box.xmax - box.xmin < sizeMulti*2.0f*maxAgentRad && box.xmax - box.xmin > 2.0f*maxAgentRad &&
				(true == inside(obsPtr1, box.xmax+0.001f, (box.zmax+box.zmin)/2.0f) || true == inside(obsPtr2, box.xmax+0.001f, (box.zmax+box.zmin)/2.0f)) &&
				(true == inside(obsPtr1, box.xmin-0.001f, (box.zmax+box.zmin)/2.0f) || true == inside(obsPtr2, box.xmin-0.001f, (box.zmax+box.zmin)/2.0f)))
			{
				xBottleneck = true;
			}

			// make sure at least one side forms
			if(false == zBottleneck && false == xBottleneck)
				continue;

			// make sure no obstacles are in the box
			bool overlapFlag = false;
			for(int id3 = 0; id3 < static_cast<int>(testCase.getNumObstacles()) && false == overlapFlag; ++id3)
			{
				const ObstacleInitialConditions obsPtr3 = testCase.getObstacleInitialConditions(id3);
				overlapFlag = overlap(box, obsPtr3);
			}
			if(true == overlapFlag)
				continue;

			// ok, have a legit bottleneck, add agent rad
			int dir = -1;
			if(true == zBottleneck)
			{
				box.xmax += 4.0f*maxAgentRad;
				box.xmin -= 4.0f*maxAgentRad;
				box.zmax += 2.0f*maxAgentRad;
				box.zmin -= 2.0f*maxAgentRad;
				dir = zDir;
			}

			if(true == xBottleneck)
			{
				box.zmax += 4.0f*maxAgentRad;
				box.zmin -= 4.0f*maxAgentRad;
				box.xmax += 2.0f*maxAgentRad;
				box.xmin -= 2.0f*maxAgentRad;
				dir = xDir;
			}

			// push the bottleneck into the vector
			m_bottlenecks.push_back(make_pair(box,dir));

			// inform user
			cout << "found a bottleneck ..." << endl;
		}
	}
}

*/

bool RVOPlusAIModule::inside(const SteerLib::BoxObstacleInitialConditions &o, float x, float z) const
{
	return (x < o.xmax && x > o.xmin && z < o.zmax && z > o.zmin);
}


bool RVOPlusAIModule::overlap(const SteerLib::BoxObstacleInitialConditions &a, const SteerLib::BoxObstacleInitialConditions &b) const
{
	return (true == inside(a,b.xmin,b.zmin) || true == inside(a,b.xmin,b.zmax)
		|| true == inside(a,b.xmax,b.zmin) || true == inside(a,b.xmax,b.zmax)
		|| true == inside(b,a.xmin,a.zmin) || true == inside(b,a.xmin,a.zmax)
		|| true == inside(b,a.xmax,a.zmin) || true == inside(b,a.xmax,a.zmax)
		|| (a.xmax == b.xmax && a.xmin == b.xmin && a.zmin == b.zmin && a.zmax == b.zmax));
}


void RVOPlusAIModule::finish()
{
	// possibly add de-allocation code here
}


SteerLib::AgentInterface* RVOPlusAIModule::createAgent()
{
	std::cout << " agent ids " << m_agentIDs.size () << "\n";

	// make sure we're not making extra agents
	assert(false == m_agentIDs.empty());

	// capture and remove the last ID
	const int id = m_agentIDs.back();
	m_agentIDs.pop_back();

	// make an agent with that ID
	RVOPlusAgent *age = new RVOPlusAgent(id,this,m_spDb);

	// set the agent's way-point vector
	age->m_wayPoints = m_agentGoalSeries.back();
	m_agentGoalSeries.pop_back();

	return age;
}


void RVOPlusAIModule::setRVOAgentInfo(RVOPlusAgent &agent, bool reset)
{
	// if restarting reset positions
	if(true == reset)
	{
		// set initial position and direction
		m_RVOsimPtr->setAgentPosition(agent.m_agentID, Vector2(agent.m_pos[0],agent.m_pos[2]));
		m_RVOsimPtr->setAgentOrientation(agent.m_agentID, atan2(agent.m_faceDir[2],agent.m_faceDir[0]));
	}

	// find the next possible goal
	size_t goalIndex = agent.m_wayPointIndex;
	while(goalIndex < agent.m_wayPoints.size() && false == agent.m_wayPoints[goalIndex].m_goal)
		++goalIndex;

	// set the goal in RVO sim
	m_RVOsimPtr->setAgentGoal(agent.m_agentID, agent.m_wayPoints[goalIndex].m_id);

	// set the preferred speed
	agent.m_desiredSpeed = agent.m_wayPoints[agent.m_wayPointIndex].m_desiredSpeed;
	m_RVOsimPtr->setAgentPrefSpeed(agent.m_agentID, agent.m_desiredSpeed);
	m_RVOsimPtr->setAgentMaxSpeed(agent.m_agentID, agent.m_desiredSpeed+extraSpeed);
}


void RVOPlusAIModule::getRVOAgentInfo(RVOPlusAgent &agent)
{
	// set the position
	Vector2 temp = m_RVOsimPtr->getAgentPosition(agent.m_agentID);
	agent.m_pos[0] = temp.x();
	agent.m_pos[2] = temp.y();

	// set the orientation	
	const float deg = m_RVOsimPtr->getAgentOrientation(agent.m_agentID);
	agent.m_faceDir[0] = cos(deg);
	agent.m_faceDir[2] = sin(deg);

	// set the speed
	Vector2 vel(m_RVOsimPtr->getAgentVelocity(agent.m_agentID));
	agent.m_speed = sqrt(vel.x()*vel.x() + vel.y()*vel.y());

	// get the radius
	agent.m_radius = m_RVOsimPtr->getAgentRadius(agent.m_agentID);
}


bool RVOPlusAIModule::goalReached(RVOPlusAgent &agent) const
{
	// just call RVO to see if current goal is reached
	return m_RVOsimPtr->getAgentReachedGoal(agent.m_agentID);
}


void RVOPlusAIModule::destroyAgent( SteerLib::AgentInterface *agent )
{
	if(NULL != agent)
		delete agent;
}


void RVOPlusAIModule::preprocessFrame(float timeStamp, float dt, unsigned int frameNumber)
{
	std::cout << " \n\n rvo preprocess frame " << frameNumber << "\n";

	if(1 == frameNumber)
	{
		std::cout << "coming here in rvo preprocess " << m_RVOsimPtr.get() << "\n";

		// setup sim timestep
		m_RVOsimPtr->setTimeStep(dt);

		// finished setting up simulation
		m_RVOsimPtr->initSimulation();
		std::cout << " done rvo sim init sim\n";
	}

	// step forward in the RVO simulation
	m_RVOsimPtr->doStep();
}


void RVOPlusAIModule::removeAgent(RVOPlusAgent *age)
{
	// disable agent in RVO simulation
	m_RVOsimPtr->setAgentActive(age->m_agentID, false);
	
	// remove this from the spacial database
	m_spDb->removeObject(dynamic_cast<SpatialDatabaseItemPtr>(age), age->m_bounds);
}


void RVOPlusAIModule::draw()
{
	// draw all the bottleneck areas
	//for(vector< pair<ObstacleInitialConditions, int> >::const_iterator i = m_bottlenecks.begin(); i != m_bottlenecks.end(); ++i)
	//{
	//	DrawLib::drawQuad(Point(i->first.xmax,0,i->first.zmax), Point(i->first.xmax,0,i->first.zmin),
	//		Point(i->first.xmin,0,i->first.zmin),Point(i->first.xmin,0,i->first.zmax), Color(0,1,0));
	//}

	// draw all the group meta-agents
	for(int i = 0; i < static_cast<int>(m_groupMetaAgentIDs.size()); ++i)
	{
		// only draw if active
		if(true == m_RVOsimPtr->getAgentActive(m_groupMetaAgentIDs[i]))
		{
			// get required info from sim
			Vector2 pos = m_RVOsimPtr->getAgentPosition(m_groupMetaAgentIDs[i]);
			float or = m_RVOsimPtr->getAgentOrientation(m_groupMetaAgentIDs[i]);
			float rad = m_RVOsimPtr->getAgentRadius(m_groupMetaAgentIDs[i]);

			// draw the disc
			DrawLib::drawAgentDisc(Point(pos.x(),0,pos.y()), Vector(cos(or),0,sin(or)), rad, Color(0,0,1));
		}
	}

	// draw the traffic controller meta-agents
	for(int i = 0; i < static_cast<int>(m_obstMetaAgentIDs.size()); ++i)
	{
		// get required info from sim
		Vector2 pos = m_RVOsimPtr->getAgentPosition(m_obstMetaAgentIDs[i]);
		float or = m_RVOsimPtr->getAgentOrientation(m_obstMetaAgentIDs[i]);
		float rad = m_RVOsimPtr->getAgentRadius(m_obstMetaAgentIDs[i]);
	
		// draw the disc
		DrawLib::drawAgentDisc(Point(pos.x(),0,pos.y()), Vector(cos(or),0,sin(or)), rad, Color(1,0,0));
	}

	// draw the form meta-agents
	for(int i = 0; i < static_cast<int>(m_formMetaAgentIDs.size()); ++i)
	{
		// only draw if active
		if(true == m_RVOsimPtr->getAgentActive(m_formMetaAgentIDs[i]))
		{
			// get required info from sim
			Vector2 pos = m_RVOsimPtr->getAgentPosition(m_formMetaAgentIDs[i]);
			float or = m_RVOsimPtr->getAgentOrientation(m_formMetaAgentIDs[i]);
			float rad = m_RVOsimPtr->getAgentRadius(m_formMetaAgentIDs[i]);

			// draw the disc
			DrawLib::drawAgentDisc(Point(pos.x(),0,pos.y()), Vector(cos(or),0,sin(or)), rad, Color(0,1,0));
		}
	}
}


std::string RVOPlusAIModule::getDependencies()
{
	//return "testCasePlayer";
	return "";
}


std::string RVOPlusAIModule::getConflicts()
{
	return "";
}

