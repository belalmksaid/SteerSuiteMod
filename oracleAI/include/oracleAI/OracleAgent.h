//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#ifndef __ORACLE_AGENT_H__
#define __ORACLE_AGENT_H__

#include "astar/AStarLite.h"
#include "SteerLib.h"

#include "footrec/FootRecIO.h"

//#define USE_ANNOTATIONS

#define AGENT_PTR(agent) (dynamic_cast<AgentInterface*>(agent))

//
// TODO: is there a better place to put these defines?
//
#ifndef INFINITY
#define INFINITY FLT_MAX
#endif




// CONSTANTS

#define M_PI            3.14159265358979323846264338328f
#define M_2_PI          6.28318530717958647692528676656f
#define M_PI_OVER_2     1.57079632679489661923132169164f
#define M_PI_OVER_180   0.0174532925199432957692369076849f
#define M_180_OVER_PI   57.2957795130823208767981548141f



// The optimal angle between leg and vertical axis that maximizes efficiency
// ORIGINALLY we tried to derive it based on a planar inverted pendulum model.
// efficiency was computed as (energy-per-step/distance-per-step)
// the value is in radians of course, approx. 35 degrees.
// However, this seems too large, and technically its a backwards optimization (maximize instead of minimize)
// #define OPTIMAL_STEP_ANGLE 0.615479708670387341067464589124f
// So instead just arbitrarily choosing a "reasonable" value here:
#define PREFERRED_STEP_ANGLE 0.41f

// WALKING PARAMETERS
#define DEFAULT_COM_HEIGHT  1.0f
#define DEFAULT_MASS 1.0f
#define DEFAULT_MIN_STEP_LENGTH  0.1f
#define DEFAULT_MAX_STEP_LENGTH  1.0f
#define DEFAULT_MIN_STEP_TIME   0.2f
#define DEFAULT_MAX_STEP_TIME   0.8f
#define DEFAULT_MAX_SPEED       1.3f
#define DEFAULT_BASE_RADIUS   0.1f
#define DEFAULT_TIME_COST_WEIGHT 0.4f
#define DEFAULT_TRAJECTORY_COST_WEGHT 0.1f

/*
// PENDULUM SIMULATION CONSTANTS
#define PENDULUM_KS 6.5f
#define PENDULUM_KD 13.0f

// INITIAL PENDULUM STATE
#define INITIAL_THETA  1.0f
#define INITIAL_THETA_DOT  10.0f
#define INITIAL_PHI  0.0f
#define INITIAL_PHI_DOT  10.0f
*/

// FOOTSTEP IDENTIFIER
#define LEFT_FOOT false
#define RIGHT_FOOT true


// ARBITRARY CONSTANTS
#define PED_REACHED_TARGET_DISTANCE_THRESHOLD 1.0f
#define FURTHEST_LOCAL_TARGET_DISTANCE 6
#define NEXT_WAYPOINT_DISTANCE 50
#define PED_MAX_NUM_WAYPOINTS 30
#define PED_QUERY_RADIUS 60.0f  // Should be comfortably bigger then mix size of scenario
#define PED_TORSO_RADIUS 0.115f
#define PED_NUM_STEPS_BEFORE_FORCED_PLAN 1
#define PED_INITIAL_STEP_VARIATION 0.6f
#define PED_REACHED_FOOTSTEP_GOAL_THRESHOLD 0.6f




namespace OracleGlobals {
	// NOTE this is forward declared for an inline function defined below
	// other declarations for this namespace belong in OracleAIModule.h
	extern SteerLib::EngineInterface * gEngineInfo;
}




struct WalkingParameters {
	// USER DEFINED PARAMETERS:

	// "height" of the character's center of mass, measured in meters
	float centerOfMassHeight;

	// mass of the character, in kilograms
	float mass;

	// min/max stride lengths
	float minStepLength;
	float maxStepLength;

	// min/max times until next step
	float minStepTime;
	float maxStepTime;

	// max speed the character can have - not a hard constraint, though...
	float maxSpeed;

	// the radius between center of mass and a foot when standing normally.
	float baseRadius;

	// CACHED AND COMPUTED PARAMETERS:
	float centerOfMassHeightInverse;
	float massInverse;
	float heuristicEnergyCostPerStep;
	float preferredStepLength;

	float timeCostWeight;
	float trajectoryCostWeight;
};



static inline std::ostream &operator<<(std::ostream &out, const DynamicState & state) {
	out << "dynamic COM state: (" << state.x << ", " << state.z << ", " << state.dx << ", " << state.dz << ")\n";
	return out; 
}


static inline std::ostream &operator<<(std::ostream &out, const Footstep & step) {
	out << ((step.whichFoot == LEFT_FOOT) ? "Left foot: " : "Right foot: ")
		<< "(" << step.parabolaX << ", " << step.parabolaZ << ") oriented at " << (step.parabolaOrientationPhi * M_180_OVER_PI)
		<< " from time " << step.startTime << " to " << step.endTime << "\n"
		<< "      cost " << step.energyCost << " to reach "<< step.outputCOMState // endline hidden in step.outputCOMState
		<< "      foot orientation would be between " << (step.innerFootOrientationPhi * M_180_OVER_PI) << " (inner) and " << (step.outerFootOrientationPhi * M_180_OVER_PI) << " (outer)\n"
		<< "      " << ((step.isAGoalState) ? "(GOAL STATE)  " : "" )
		<< ((step.state==FOOTSTEP_STATE_NORMAL) ? "normal step" : ((step.state==FOOTSTEP_STATE_STOPPING) ? "stopping" : ((step.state==FOOTSTEP_STATE_STARTING) ? "starting to move" : ((step.state==FOOTSTEP_STATE_STATIONARY) ? "remains stationary" : "ERROR! INVALID STATE" ) ) ))
		<< "\n"
		<< "      simulationA = " << step.simulationA << "\n";
	return out; 
}


//======================================================================================
// OracleAgent class
//======================================================================================




class OracleAgent : public SteerLib::AgentInterface
{

public:

	friend class OracleEnvironment;
	friend class OracleAIModule;

	// AgentInterface functionality:
	OracleAgent();
	~OracleAgent();
	void reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo);
	void updateAI(float timeStamp, float dt, unsigned int frameNumber);
	void disable();
	void draw();

	bool enabled() const { return _enabled; }
	Util::Point position() const { return _position; }
	Util::Vector forward() const { return _forward; }
	Util::Vector velocity() const { throw Util::GenericException("velocity() not implemented yet"); }
	float radius() const { return _radius; }
	const SteerLib::AgentGoalInfo & currentGoal() const { return _currentGoal; }
	void addGoal(const SteerLib::AgentGoalInfo & newGoal);
	size_t id() const { return 0;}
	const std::queue<SteerLib::AgentGoalInfo> & agentGoals() const { throw Util::GenericException("agentGoals() not implemented yet"); }
	void clearGoals() { while (!_goalQueue.empty()) _goalQueue.pop(); }

	void insertAgentNeighbor(const SteerLib::AgentInterface *agent, float &rangeSq) { throw Util::GenericException("insertAgentNeighbor not implemented yet for BenchmarkAgent"); }
	void setParameters(SteerLib::Behaviour behave)
	{
		throw Util::GenericException("setParameters() not implemented yet for this Agent");
	}

	SteerLib::GridDatabase2D * getSpatialDataBaseInterface();
	bool intersects(const Util::Ray &r, float &t) { return Util::rayIntersectsCircle2D(_position, _radius, r, t); }
	bool overlaps(const Util::Point & p, float radius) { 
		if(Util::circleOverlapsCircle2D( _position, _radius, p, radius)) {
			return true;
		}
		else {
			return false;
		}
	}
	float computePenetration(const Util::Point & p, float radius) { return Util::computeCircleCirclePenetration2D( _position, _radius, p, radius); }


	// Specialized footstep functionality:
	bool collidesAtTimeWith(const Util::Point & p1, const Util::Vector & rightSide, float otherAgentRadius, float timeStamp, float footX, float footZ);

	//special consideration for IDA* planning
	void flushStepCache(void) {
		unsigned int oldSize = _cachedFootstepOptions.size();
		_cachedFootstepOptions.resize(2);	//the start and goal SHOULD be in the 0th and 1st element so this will preserve them and cut the rest away
		_cachedFootstepOptions.reserve(oldSize);
	}

	void printStepCacheSize(void) {
		cout << _cachedFootstepOptions.capacity() << endl;
	}

protected:

	// returns FALSE if the input parameters create an invalid footstep.
	// step duration (time), orientation, and new velocity are the control parameters.
	bool _createFootstepAction(float stepDuration, float parabolaOrientationPhi, bool phiIsIdeal, float desiredVelocity, const Footstep & previousStep, Footstep & nextStep, FootStateEnum nextState);

	void _disable();
	void _runCognitivePhase();
	void _runLongTermPlanningPhase();
	void _runMidTermPlanningPhase();
	void _runShortTermPlanningPhase();
	void _runPerceptivePhase();
	void _planFootstepsToShortTermGoal(bool useHardCodedPlan);
	void _updateCharacterFollowingPath(float currentTime);
	bool _reachedCurrentGoal();
	bool _reachedCurrentWaypoint();
	bool _reachedLocalTarget();



	// USER-CONTROLLABLE WALKING PROPERTIES
	WalkingParameters _walkParameters;

	// STATE OF THE CHARACTER
	Footstep _currentStep;
	Footstep _previousStep;
	Util::Point _currentSimulatedPosition;
	Util::Vector _currentSimulatedVelocity;

	// VECTOR OF CACHED STATES USED DURING PLANNING
	std::vector<Footstep> _cachedFootstepOptions;

	// THE PLAN, STORED IN "CORRECT" ORDER (the astar search stores it in backwards order initially)
	std::vector<unsigned int> _footstepPlan;

	// VECTOR OF STEPS ALREADY TAKEN
	std::vector<Footstep> _stepHistory;

	// LONG-TERM PLANNING PHASE
	int _currentWaypointIndex;

	// MID-TERM PLANNING PHASE
	int _midTermPath[NEXT_WAYPOINT_DISTANCE+2];
	int _midTermPathSize;

	// SHORT-TERM PLANNING PHASE
	Util::Point _localTargetLocation; // short-term planning communicates to reactive steering phase

	// PERCEPTION PHASE
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors; // reactive phase communicates to itself and predictive phase

	// misc stuff - todo, organize these later!
	bool _needToRunFootstepPlanner;
	unsigned int _currentLocationInPath;
	int planAge;
};


#endif
