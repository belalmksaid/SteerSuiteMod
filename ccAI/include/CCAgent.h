//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#ifndef __CC_AGENT__
#define __CC_AGENT__

/// @file CCAgent.h
/// @brief Declares the CCAgent class.

#include <queue>
#include <string>
#include "SteerLib.h"
#include "CCAIModule.h"



/**
 * @brief An example agent with very basic AI, that is part of the ccAI plugin.
 *
 * This agent performs Continuum Crowd AI using forces and Euler integration, simply
 * steering towards static goals while being told from the Crowd Continuum algorithm
 * the next frame velocity.
 *
 * This class is instantiated when the engine calls CCAIModule::createAgent().
 *
 */
class CCAgent : public SteerLib::AgentInterface
{
public:
	CCAgent();
	~CCAgent();
	void reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo);
	void updateAI(float timeStamp, float dt, unsigned int frameNumber);
	void draw();

	bool enabled() const { return _enabled; }
	void disable();
	Util::Point position() const { return _position; }
	Util::Vector forward() const { return _forward; }
	float radius() const { return _radius; }
	const SteerLib::AgentGoalInfo & currentGoal() const { return _goalQueue.front(); }
	size_t id() const { return _id; }
	const std::queue<SteerLib::AgentGoalInfo> & agentGoals() const { throw Util::GenericException("agentGoals() not implemented yet"); }
	void addGoal(const SteerLib::AgentGoalInfo & newGoal) { throw Util::GenericException("addGoals() not implemented yet for CCAgent"); }
	void clearGoals() { throw Util::GenericException("clearGoals() not implemented yet for CCAgent"); }

	void insertAgentNeighbor(const SteerLib::AgentInterface *agent, float &rangeSq) { throw Util::GenericException("insertAgentNeighbor not implemented yet for BenchmarkAgent"); }
	void setParameters(SteerLib::Behaviour behave)
	{
		throw Util::GenericException("setParameters() not implemented yet for this Agent");
	}

	/// @name The SteerLib::SpatialDatabaseItemInterface
	/// @brief These functions are required so that the agent can be used by the SteerLib::SpatialDataBaseInterface spatial database;
	/// The Util namespace helper functions do the job nicely for basic circular agents.
	//@{
	bool intersects(const Util::Ray &r, float &t) { return Util::rayIntersectsCircle2D(_position, _radius, r, t); }
	bool overlaps(const Util::Point & p, float radius) { return Util::circleOverlapsCircle2D( _position, _radius, p, radius); }
	float computePenetration(const Util::Point & p, float radius) { return Util::computeCircleCirclePenetration2D( _position, _radius, p, radius); }
	//@}

	Util::Vector velocity() const { return _velocity; }
	void setPlanVelocity(const Util::Vector& vec) { _planned_velocity = vec;}
	void setName(const size_t name) { _name = name;}
	const size_t getName() { return _name; }
protected:
	/// Updates position, velocity, and orientation of the agent, given the force and dt time step.
	void _doEulerStep(const Util::Vector & steeringDecisionForce, float dt);

	bool _enabled;
	Util::Point _position;
	Util::Vector _velocity;
	Util::Vector _forward; // normalized version of velocity
	float _radius;
	std::queue<SteerLib::AgentGoalInfo> _goalQueue;

  ///////////////////////
	size_t _name;
	Util::Vector _planned_velocity;
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;

	friend class CCAIModule;
	virtual SteerLib::EngineInterface * getSimulationEngine();

};

#endif
