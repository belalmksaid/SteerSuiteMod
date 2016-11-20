//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#ifndef __SPACE_TIME_AGENT__
#define __SPACE_TIME_AGENT__

/// @file SpaceTimeAgent.h
/// @brief Declares the SpaceTimeAgent class.

#include <queue>
#include "SteerLib.h"
#include "SpaceTimeAIModule.h"

using namespace Util;

struct AgentState {
	Point position;
	Vector velocity;
	Vector forward;
	double time;
};

class SpaceTimePoint {
public:
	Point point;
	double time;
	bool operator==(const SpaceTimePoint &pt) const { return ((point == pt.point) && (time == pt.time)); }
	bool operator<(const SpaceTimePoint &pt) const {
		double threshold = 0.01;
		if (fabs(point.x - pt.point.x) > threshold ) {
			return point.x < pt.point.x;
		} else if (fabs(point.y - pt.point.y) > threshold) {
			return point.y < pt.point.y;
		} else if (fabs(point.z - pt.point.z) > threshold) {
			return point.z < pt.point.z;
		} else {
			return time < pt.time;
		}
	}
};


/**
 * @brief An example agent with very basic AI, that is part of the spaceTimeAI plugin.
 *
 * This agent performs extremely simple AI using forces and Euler integration, simply
 * steering towards static goals without avoiding any other agents or static obstacles.
 * Agents that are "selected" in the GUI will have some simple annotations that
 * show how the spatial database and engine interface can be used.
 *
 * This class is instantiated when the engine calls SpaceTimeAIModule::createAgent().
 *
 */
class SpaceTimeAgent : public SteerLib::AgentInterface
{
public:
	SpaceTimeAgent();
	~SpaceTimeAgent();
	void init(int maxNumNodesToExpand, bool useHermiteInterpolation);
	void reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo);
	void updateAI(float timeStamp, float dt, unsigned int frameNumber);
	void draw();
	Util::Point SpaceTimeAgent::HermitePosition(Util::Point a, Util::Vector aTangent, Util::Point b, Util::Vector bTangent, double s);
	Util::Vector SpaceTimeAgent::HermiteVelocity(Util::Point a, Util::Vector aTangent, Util::Point b, Util::Vector bTangent, double s);
	Util::Point Bezier(Util::Point a, Util::Point b, Util::Point c, Util::Point d, double s);
	bool enabled() { return _enabled; }
	void disable();
	Util::Point position() { return _position; }
	Util::Vector forward() { return _forward; }
	Util::Vector velocity() { 
		
		//return _currentSimulatedVelocity; // MUBBASIR TODO: Check if this is what is expected
		std::cerr << "SpaceTimeAgent::This function needs to be implemented \n";
		return Util::Vector();
	}


	float radius() { return _radius; }
	const SteerLib::AgentGoalInfo & currentGoal() { return _goalQueue.front(); }
	size_t id() { return 0;}
	const std::queue<SteerLib::AgentGoalInfo> & agentGoals() { throw Util::GenericException("agentGoals() not implemented yet"); }
	void addGoal(const SteerLib::AgentGoalInfo & newGoal) { throw Util::GenericException("addGoals() not implemented yet for SpaceTimeAgent"); }
	void clearGoals() { throw Util::GenericException("clearGoals() not implemented yet for SpaceTimeAgent"); }

	/// @name The SteerLib::SpatialDatabaseItemInterface
	/// @brief These functions are required so that the agent can be used by the SteerLib::GridDatabase2D spatial database;
	/// The Util namespace helper functions do the job nicely for basic circular agents.
	//@{
	bool intersects(const Util::Ray &r, float &t) { return Util::rayIntersectsCircle2D(_position, _radius, r, t); }
	bool overlaps(const Util::Point & p, float radius) { return Util::circleOverlapsCircle2D( _position, _radius, p, radius); }
	float computePenetration(const Util::Point & p, float radius) { return Util::computeCircleCirclePenetration2D( _position, _radius, p, radius); }
	//@}
	Point* GetLocationAtTime(double time);


protected:
	/// Updates position, velocity, and orientation of the agent, given the force and dt time step.
	void _doEulerStep(const Util::Vector & steeringDecisionForce, float dt);


	bool _enabled;
	Util::Point _position;
	Util::Vector _velocity;
	Util::Vector _forward; // normalized version of velocity
	float _radius;
	std::queue<SteerLib::AgentGoalInfo> _goalQueue;
	std::vector<AgentState> guaranteedPlan;
	std::stack<SpaceTimePoint> longTermPathForDrawing;
	int guaranteedPlanIndex;
	int guaranteedPlanSize;
	int guaranteedPlanBufferSize;
	bool planReachedGoal;
	int maxNumNodesToExpand;
	bool useHermiteInterpolation;
	bool noPathFound;
};

#endif
