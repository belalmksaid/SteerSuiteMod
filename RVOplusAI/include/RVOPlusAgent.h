//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#ifndef __RVOPlusagent_h__
#define __RVOPlusagent_h__

#include <vector>
#include "../../steerlib/include/SteerLib.h"
#include "goalInfo.h"

class RVOPlusAIModule;

class RVOPlusAgent : public SteerLib::AgentInterface
{
	friend class RVOPlusAIModule;

public:

	// constructor & destructor
	RVOPlusAgent(int id, RVOPlusAIModule *pMod, SteerLib::GridDatabase2D *gdb);
	~RVOPlusAgent();

	// functions required by the agent interface
	void reset(const SteerLib::AgentInitialConditions &initialConditions, SteerLib::EngineInterface *engineInfo);
	void updateAI(float timeStamp, float dt, unsigned int frameNumber);
	void draw();
	bool enabled();
	void disable();
	Util::Point position();
	Util::Vector forward();
	
	Util::Vector velocity () { 
		
		//return forward () * m_speed; 
		std::cerr << "RVOPlusAgent::This function needs to be implemented. \n"; 
	
	}


	float radius();
	const SteerLib::AgentGoalInfo& currentGoal();
	size_t id() { return 0;}
	const std::queue<SteerLib::AgentGoalInfo> & agentGoals() { throw Util::GenericException("agentGoals() not implemented yet"); }

	// functions required by some other interface
	bool intersects(const Util::Ray &r, float &t);
	bool overlaps(const Util::Point & p, float radius);
	float computePenetration(const Util::Point & p, float radius);

	// need to implement these
	void addGoal(const SteerLib::AgentGoalInfo & newGoal);
	void clearGoals();

private:

	// vector of the goals from framework
	std::vector<SteerLib::AgentGoalInfo> m_goals;

	// vector of all the waypoints (including goals)
	std::vector<goalInfo> m_wayPoints;

	// current position
	Util::Point m_pos;

	// starting position
	Util::Point m_startPos;

	// direction agent is facing
	Util::Vector m_faceDir;

	// current goal position
	Util::Point m_goal;
	Util::Point m_prevGoal;
	Util::Point m_nextGoal;

	// current bounding box
	Util::AxisAlignedBox m_bounds;

	// agent radius
	float m_radius;

	// agent speed
	float m_speed;

	// desired agent speed
	float m_desiredSpeed;

	// index of the current waypoint
	size_t m_wayPointIndex;

	// index of the current goal
	size_t m_goalIndex;

	// pointer to the parent AI Module
	RVOPlusAIModule *m_parentMod;

	// pointer to spacial database
	SteerLib::GridDatabase2D *m_spDb;

	// agent enabled?
	bool m_enabled;

	// ID of agent in RVO simulator
	const int m_agentID;
};

#endif
