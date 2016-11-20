//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#ifndef __RVOPlusAIModule_h__
#define __RVOPlusAIModule_h__

#include <memory>
#include <map>
#include <vector>
#include <utility>
#include "../../steerlib/include/SteerLib.h"
#include "../../steerlib/include/testcaseio/ObstacleInitialConditions.h"
#include "../libs/RVO/RVOSimulator.h"
#include "goalInfo.h"

//#include <Windows.h>


class RVOPlusAgent;

class RVOPlusAIModule : public SteerLib::ModuleInterface
{
public:

	// comparison struct for points (only care about x and z)
	struct pointCmp
	{
		bool operator()(const Util::Point &A, const Util::Point &B) const
		{
			if(A[0] < B[0])
				return true;
			else if(A[0] == B[0])
			{
				if(A[2] < B[2])
					return true;
				else
					return false;
			}
			else
				return false;
		}
    };

	// constructor & destructor
	RVOPlusAIModule();
	~RVOPlusAIModule();

	// steerlib required functions
	std::string getDependencies();
	std::string getConflicts();
	std::string getData() { return ""; }
	LogData * getLogData() { return new LogData(); }
	void init(const SteerLib::OptionDictionary &options, SteerLib::EngineInterface *engineInfo);
	void finish();
	SteerLib::AgentInterface* createAgent();
	void destroyAgent(SteerLib::AgentInterface *agent);
	void preprocessFrame(float timeStamp, float dt, unsigned int frameNumber);
	void initializeSimulation();
	void draw();

	// functions to sync agent data with RVO simulation
	void setRVOAgentInfo(RVOPlusAgent &agent, bool reset = false);
	void getRVOAgentInfo(RVOPlusAgent &agent);
	bool goalReached(RVOPlusAgent &agent) const;
	void removeAgent(RVOPlusAgent *agent);

	// functions dealing with goals in the RVO simulation
	int findGoal(const Util::Point &goal) const;
	 bool findGoal(int id, Util::Point &p) const;


	__declspec(dllexport) void preprocessObstacles (std::vector<SteerLib::BoxObstacle> & obstacles);
	__declspec(dllexport) void preprocessAgents (const std::vector<SteerLib::AgentInitialConditions> &agents);

	__declspec(dllexport) void resetRVOSimulator ();


private:

	// RVO simulator object
	std::auto_ptr<RVO::RVOSimulator> m_RVOsimPtr;

	// map of goals and roadmap vertices (lookup by location)
	std::map<Util::Point,int,pointCmp> m_goals;

	// map of goals and roadmap vertices (lookup by ID)
	std::map<int,Util::Point> m_goalsReverse;

	// vector of all the agent IDs
	std::vector<int> m_agentIDs;

	// vector of group meta-agent IDs
	std::vector<int> m_groupMetaAgentIDs;

	// vector of obstacle meta-agent IDs
	std::vector<int> m_obstMetaAgentIDs;

	// vector of form meta-agent IDs
	std::vector<int> m_formMetaAgentIDs;

	// vector of agent goal vectors
	std::vector< std::vector<goalInfo> > m_agentGoalSeries;

	// vector of all the bottlenecks
	std::vector< std::pair<SteerLib::BoxObstacleInitialConditions, int> > m_bottlenecks;

	// pointer to space database
	SteerLib::GridDatabase2D *m_spDb;

	// pointer to the engine
	SteerLib::EngineInterface *m_engineInfo;

	// total number of agents
	int m_numAgents;

	// adds a goal to the module's database
	int addGoal(const Util::Point &goal);

	// add box obstacles
	void addBoxObst(const SteerLib::BoxObstacleInitialConditions &o, float maxAgentRad);

	// find the possible bottlenecks
	void findBottlenecks(const SteerLib::TestCaseReader &testCase, float maxAgentRad, float sizeMulti);

	// checks if a point is in an obstacle
	bool inside(const SteerLib::BoxObstacleInitialConditions &o, float x, float z) const;

	// checks if two obstacles overlap
	bool overlap(const SteerLib::BoxObstacleInitialConditions &a, const SteerLib::BoxObstacleInitialConditions &b) const;

	// checks if agent can see to next goal
	bool clearPath(const Util::Point &start, const Util::Point &goal, float agentRad) const;
};

#endif
