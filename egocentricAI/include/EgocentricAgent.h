//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#ifndef __EGOCENTRIC_AGENT__
#define __EGOCENTRIC_AGENT__

/// @file EgocentricAgent.h
/// @brief Declares the EgocentricAgent class.

#include <queue>
#include <set>

#include "SteerLib.h"
#include "EgocentricAgent.h"
#include "EgocentricAIModule.h"
#include "util/Geometry.h"

#include "EgocentricConstants.h"
#include "astar/AStarLite.h"

using namespace Util;

//
// SteeringCommand - the data used in the AI to give the final steer command.
// a final phase of steering will convert this command into a steering force.
// see comments to understand how its used.
//
struct SteeringCommand {

	enum LocomotionType { LOCOMOTION_MODE_COMMAND, LOCOMOTION_MODE_DYNAMICS, LOCOMOTION_MODE_SPACETIME };

	void clear() {
		steeringMode = LOCOMOTION_MODE_COMMAND;
		aimForTargetDirection = false;
		targetDirection = Vector(1.0f, 0.0f, 0.0f);
		turningAmount = 0.0f;
		aimForTargetSpeed = false;
		targetSpeed = 0.0f;
		acceleration = 0.0f;
		scoot = 0.0f;
		dynamicsSteeringForce = Vector(0.0f, 0.0f, 0.0f);
	}

	LocomotionType steeringMode;

	// space-time steering mode - no data here, use the space-time path data.

	// dynamics steering mode
	Vector dynamicsSteeringForce;

	// command steering mode

	// if aimForTargetDirection is true, then steering will aim for the targetDirection, with a turning speed dependent on turningAmount
	// if it is false, then it will just turn with turning speed dependent on turningAmount.
	bool aimForTargetDirection;
	Vector targetDirection;
	float turningAmount;

	// if aimForTargetSpeed is true, then steering will aim for the targetSpeed (NOT dependent on float acceleration given by the command.)
	// if it is false, then it will just accelerate by "acceleration".
	bool aimForTargetSpeed;
	float targetSpeed;
	float acceleration;

	// side-to-side motion
	float scoot;
};



/**
 * @brief An egocentric agent, that is part of the egocentricAI plugin.
 *
 * This is the source code for the steering AI presented in the technical publication 
 * "Egocentric Affordance Fields in Pedestrian Steering" which was presented at the conference, 
 * Interactive 3D Graphics and Games 2009 (I3D) which was held in Boston, MA. 
 *
 * This framework can be used for local path-planning and steering and can be easily extended
 * to perform high-level behaviors. Our framework is based on the concept of affordances - 
 * the possible ways an agent can interact with its environment. Each agent perceives 
 * the environment using a set of scalar and vector fields that are represented in the agent's 
 * local space. This egocentric property allows us to compute local a space-time plan. Perception 
 * fields are used to compute a fitness measure for every possible action, known as affordance fields. 
 * The action that has the optimal value in the affordance field is the agent's steering decision. 

 * Using this framework, we demonstrate autonomous virtual pedestrians that perform steering and
 * path-planning in unknown environments along with the emergence of high-level responses to never 
 * seen before situations. 
 * 
 * For more information, please visit www.cs.ucla.edu/~mubbasir
 *
 * This class is instantiated when the engine calls EgocentricAIModule::createAgent().
 *
 */

using namespace Util;

class EgocentricMap;

class EgocentricAgent : public SteerLib::AgentInterface
{
public:
	EgocentricAgent();
	~EgocentricAgent();

	void init (const int & numberOfNodesPerLevel, const int & numberOfLevels);

	void reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo);
	void updateAI(float timeStamp, float dt, unsigned int frameNumber);
	void draw();

	bool enabled() const { return _enabled; }
	Util::Point position() const { return _position; }
	Util::Vector forward() const { return _forward; }
	Util::Vector velocity() const { return _velocity ; }
	float radius() const { return _radius; }
	const SteerLib::AgentGoalInfo & currentGoal() const { return _goalQueue.front(); }
	size_t id() const { return 0;}
		const std::queue<SteerLib::AgentGoalInfo> & agentGoals() const { throw Util::GenericException("agentGoals() not implemented yet"); }
	void addGoal(const SteerLib::AgentGoalInfo & newGoal) { throw Util::GenericException("addGoal() not implemented yet for EgocentricAgent"); }
	void clearGoals() { throw Util::GenericException("clearGoals() not implemented yet for EgocentricAgent"); }

	void insertAgentNeighbor(const SteerLib::AgentInterface *agent, float &rangeSq) { throw Util::GenericException("insertAgentNeighbor not implemented yet for BenchmarkAgent"); }
	void setParameters(SteerLib::Behaviour behave)
	{
		throw Util::GenericException("setParameters() not implemented yet for this Agent");
	}
	/// @name The SteerLib::SpatialDatabaseItemInterface
	/// @brief These functions are required so that the agent can be used by the SteerLib::GridDatabase2D spatial database;  Unless
	/// you want something more complex than a circular agent, the Util namespace helper functions do the job nicely.
	//@{
	bool intersects(const Util::Ray &r, float &t) { return Util::rayIntersectsCircle2D(_position, _radius, r, t); }
	bool overlaps(const Util::Point & p, float radius) { return Util::circleOverlapsCircle2D( _position, _radius, p, radius); }
	float computePenetration(const Util::Point & p, float radius) { return Util::computeCircleCirclePenetration2D( _position, _radius, p, radius); }
	//@}
	EgocentricMap * _esm;
	std::set<SteerLib::SpatialDatabaseItem *> neighbors;


protected:

	virtual SteerLib::EngineInterface * getSimulationEngine();
	// old pedestrian functions:
	// **** - reset()
	void collectObjectsInVisualField();
	bool reachedCurrentLandmark();
	bool reachedWaypoint ();
	bool threatListContainsPedestrian();
	void disable();

	void runCognitivePhase();
	void runLongTermPlanningPhase();
	void runMidTermPlanningPhase();
	void runShortTermPlanningPhase();

	void runMicroPlanningPhase();
	void runPerceptivePhase();
	void runPredictivePhase();
	void runReactivePhase();
	void updatePhysics(float timeStamp, float dt, unsigned int frameNumber);
	void drawPlannedPath();

	bool reachedCurrentGoal ();
	bool reachedCurrentWaypoint();
	bool reachedLocalTarget();

	// given a steering command, these functions do the actual steering.
	//void doSteering();
	//void doSpaceTimeSteering();
	//void doDynamicsSteering();
	void doCommandBasedSteering(float dt);

	void doSimpleSteering (float dt);

	/// Updates position, velocity, and orientation of the agent, given the force and dt time step.
	void _doEulerStep(const Util::Vector & steeringDecisionForce, float dt);

	/*
	_queryGridDatabse ( )

	const int & levelLimit : the level limits the amount of area to be queried and the number of levels of the esm to loop through.
	const float & startAngle : it determines the starting node of the loop.
	const float & endAngle : it determines the ending node of the loop.

	Description : TODO 
	*/
	float queryGridDatabse ( const int & levelLimit, const float & startAngle, const float & endAngle );

	/*
	goal()

	return Point : returns the goal to steer to.
	Description : returns the goal to steer to. This is the function that handles all calls to :
		runLongTermPlanningPhase()
		runMidTermPlanningPhase()
		runShortTermPlanningPhase()
		runCognitivePhase()
	*/
	void calculateTime(const float & speed);
	void queryDynamicObjectsinTime(const int & startNeuronId, const int & endNeuronId, const int & level);
	float determineTargetSpeed();
	void evaluateDynamicObjects(int level, float startAngle, float endAngle);
	void queryDatabase(int level, float startAngle, float endAngle);
	float determineSpeedByDetectingPotentialCollisions ( const int & levelLimit, const float & startAngle, const float & endAngle );
	Point goal ();

	SteeringCommand _finalSteeringCommand;

	float *t;

	// LONG-TERM PLANNING PHASE
	int _currentWaypointIndex;

	// MID-TERM PLANNING PHASE
	// int * _midTermPath;
	int * _midTermPath; //[EGO_NEXT_WAYPOINT_DISTANCE];
	int _midTermPathSize;

	// SHORT-TERM PLANNING PHASE
	Point _localTargetLocation; // short-term planning communicates to reactive steering phase

	// ORIENTATION of the agent (mostly unrelated to the agent's physics)
	Vector _rightSide;

	// PHYSICS of the agent
	float _mass;
	float _maxSpeed;
	float _maxForce;

	// MUBBASIR TODO -- These were present in SteerSimInterfaces.h -- Where are they here?
	float _currentSpeed;
	float _desiredSpeed;

	// OTHER
	float _currentTimeStamp;
	float _dt;
	unsigned int _currentFrameNumber;

	// FOR determineSpeed ()
	bool maintainSpeed;

};


#endif
