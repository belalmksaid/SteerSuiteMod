//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#include "../include/RVOPlusAgent.h"
#include "../include/RVOPlusAIModule.h"
#include "../../steerlib/include/util/DrawLib.h"

using namespace Util;
using namespace SteerLib;
using namespace std;


RVOPlusAgent::RVOPlusAgent(int id, RVOPlusAIModule *pMod, SteerLib::GridDatabase2D *gdb) :
	m_agentID(id),
	m_parentMod(pMod),
	m_spDb(gdb)
{
	// make sure we didn't get a NULL
	assert(NULL != pMod && NULL != gdb);
}


RVOPlusAgent::~RVOPlusAgent()
{}


void RVOPlusAgent::reset(const SteerLib::AgentInitialConditions &initialConditions, SteerLib::EngineInterface *engineInfo)
{
	// save initial position
	m_startPos = initialConditions.position;

	// save goals from framework
	m_goals = initialConditions.goals;
	m_goalIndex = 0;

	// reset waypoint index
	m_wayPointIndex = 0;
	while(m_wayPointIndex < m_wayPoints.size() && false == m_wayPoints[m_wayPointIndex].m_goal)
		++m_wayPointIndex;

	// save starting location
	m_pos = initialConditions.position;

	// everyone starts facing the same way
	m_faceDir[0] = 1.;
	m_faceDir[1] = 0;
	m_faceDir[2] = 0;

	// save the radius
	m_radius = initialConditions.radius;

	// save the speeds
	m_speed = initialConditions.speed;
	m_desiredSpeed = m_wayPoints.front().m_desiredSpeed;

	// calculate the bounds box
	AxisAlignedBox newBounds(m_pos[0]-m_radius, m_pos[0]+m_radius, 0.0f, 0.0f, m_pos[2]-m_radius, m_pos[2]+m_radius);

	// add this to the database
	if(false == m_enabled)
		m_spDb->addObject(dynamic_cast<SpatialDatabaseItemPtr>(this), newBounds);
	else
		m_spDb->updateObject(dynamic_cast<SpatialDatabaseItemPtr>(this), m_bounds, newBounds);

	// save the bounds box
	m_bounds = newBounds;

	// sync this info with the RVO agent
	m_parentMod->setRVOAgentInfo(*this,true);

	// enable agent
	m_enabled = true;
}


void RVOPlusAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
	// if disabled do nothing
	if(false == m_enabled)
		return;

	// update from RVO sim
	m_parentMod->getRVOAgentInfo(*this);

	// if reached a goal, then set a new one
	if(true == m_parentMod->goalReached(*this))
	{
		// increment goal index
		++m_goalIndex;

		// increment way-point index past current goal
		do
			++m_wayPointIndex;
		while (m_wayPointIndex < m_wayPoints.size() && false == m_wayPoints[m_wayPointIndex].m_goal);

		// if this was last goal then disable
		if(m_wayPointIndex == m_wayPoints.size())
		{
			// remove agent from future consideration
			m_parentMod->removeAgent(this);

			// disable agent
			m_enabled = false;

			return;
		}

		// sync with RVO sim
		m_parentMod->setRVOAgentInfo(*this);
	}

	// calculate the bounds box
	AxisAlignedBox newBounds(m_pos[0]-m_radius, m_pos[0]+m_radius, 0.0f, 0.0f, m_pos[2]-m_radius, m_pos[2]+m_radius);

	// update the database
	m_spDb->updateObject(dynamic_cast<SpatialDatabaseItemPtr>(this), m_bounds, newBounds);

	// save the bounds box
	m_bounds = newBounds;
}


void RVOPlusAgent::draw()
{
	// just draw a disc
	DrawLib::drawAgentDisc(m_pos, m_faceDir, m_radius, Color(0.9f,0.4f,0.1f));

	// draw first segment of the A* path
	Color c;
	Point start(m_startPos);
	Point end(m_wayPoints[0].m_loc);
	c = (0 < m_wayPointIndex) ? Color(0,0,0) : Color(1,1,1);

	//DrawLib::glColor(c);
	DrawLib::drawLine(start, end);

	DrawLib::drawStar(start, Util::Vector(0,0,1), 0.3f, c);

	// draw the other segments of the A* path
	size_t i;
	for(i = 0; i+1 < m_wayPoints.size() && i < m_wayPointIndex; ++i)
	{
		// find start and end points of the segment
		start = m_wayPoints[i].m_loc;
		end = m_wayPoints[i+1].m_loc;
		
		// draw the line segment
		c = (i+1 < m_wayPointIndex) ? Color(0,0,0) : Color(1,1,1);
		//DrawLib::glColor(c);
		DrawLib::drawLine(start, end);

		// draw a star at the beginning
		DrawLib::drawStar(start, Util::Vector(0,0,1), 0.3f, c);	
	}

	// draw the last flag
	DrawLib::drawFlag(m_wayPoints[i].m_loc);
}


bool RVOPlusAgent::intersects(const Util::Ray &r, float &t)
{
	return Util::rayIntersectsCircle2D(m_pos, m_radius, r, t);
}


bool RVOPlusAgent::overlaps(const Util::Point & p, float radius)
{
	return Util::circleOverlapsCircle2D(m_pos, m_radius, p, radius);
}


float RVOPlusAgent::computePenetration(const Util::Point & p, float radius)
{
	return Util::computeCircleCirclePenetration2D(m_pos, m_radius, p, radius);
}


bool RVOPlusAgent::enabled()
{
	return m_enabled;
}


void RVOPlusAgent::disable()
{
	// *** NOT FULLY IMPLEMENTED ***
	assert(m_enabled==true); 

	m_enabled = false;
}


Util::Point RVOPlusAgent::position()
{
	return m_pos;
}


Util::Vector RVOPlusAgent::forward()
{
	return m_faceDir;
}


float RVOPlusAgent::radius()
{
	return m_radius;
}


const SteerLib::AgentGoalInfo& RVOPlusAgent::currentGoal()
{
	return m_goals[m_goalIndex];
}


void RVOPlusAgent::addGoal(const SteerLib::AgentGoalInfo & newGoal)
{
	assert(false);
}


void RVOPlusAgent::clearGoals()
{
	assert(false);
}
