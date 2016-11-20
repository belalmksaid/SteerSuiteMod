//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

//
// Copyright (c) 2009 Shawn Singh, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#ifndef __FOOTSTEP_AGENT_H__
#define __FOOTSTEP_AGENT_H__

#include "astar/AStarLite.h"
#include "SteerLib.h"

#include "shadowRec/ShadowRecIO.h"
#include "C5Reader/TreeInterface.h"
#include "../../../steergen/include/StateConfig.h"
#include "trackReader/TrackReader.h"

//#define USE_ANNOTATIONS

#define AGENT_PTR(agent) (dynamic_cast<AgentInterface*>(agent))


// CONSTANTS


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
#define NEXT_WAYPOINT_DISTANCE 30
#define PED_MAX_NUM_WAYPOINTS 20
#define PED_QUERY_RADIUS 10.0f
#define CONTEXT_QUERY_RADIUS 30.0f
#define PED_TORSO_RADIUS 0.15f
#define PED_NUM_STEPS_BEFORE_FORCED_PLAN 1	//This forced the A* to only have a 1-step effect, so it doesn't throw off the anomaly detection
#define PED_INITIAL_STEP_VARIATION 0.6f
#define PED_REACHED_FOOTSTEP_GOAL_THRESHOLD 0.6f



namespace ShadowAIGlobals {
	// NOTE this is forward declared for an inline function defined below
	// other declarations for this namespace belong in ShadowAIModule.h
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
// ShadowAgent class
//======================================================================================




class ShadowAgent : public SteerLib::AgentInterface
{

public:
	typedef TreeInterface::stepInfo stepInfo;

	struct agentInfo {
		float COMx;
		float COMz;
		float velx;	//may end up using their footstep info instead of COM info
		float velz;
		unsigned int id;	//might allow for fast culling of agents but performance gains may be minimal; will integrate later
	};

	//slightly redundant as there is an identical data structure in FootRecIO, but we copy information into this and transform it
	struct obstacleInfo {
		float xmin;
		float xmax;
		float zmin;
		float zmax;
		float rayCorrectionAngle;
	};

	struct subjectInfo {
		Footstep step;
		float theta;
		float subjectPrevX;
		float subjectPrevZ;
		Util::Point subjectGoal;
	};

	friend class FootstepEnvironment;
	friend class ShadowAIModule;

	// AgentInterface functionality:
	ShadowAgent(TreeInterface*, StateConfig*, TrackReader*, unsigned int);
	~ShadowAgent(void);
	void reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo);
	void updateAI(float timeStamp, float dt, unsigned int frameNumber);
	void draw();

	bool enabled(void) { return _enabled; }
	void disable();
	bool finished(void);
	Util::Point position() { return _position; }
	Util::Vector forward() { return _forward; }

	Util::Vector velocity() { 
		
		//return _currentSimulatedVelocity; // MUBBASIR TODO: Check if this is what is expected
		std::cerr << "ShadowAgent::This function needs to be implemented \n";
		return Util::Vector();
	}

	float radius() { return _radius; }
	const SteerLib::AgentGoalInfo & currentGoal() { return _currentGoal; }
	size_t id() { return 0;}
	const std::queue<SteerLib::AgentGoalInfo> & agentGoals() { throw Util::GenericException("agentGoals() not implemented yet"); }
	void addGoal(const SteerLib::AgentGoalInfo & newGoal);
	void clearGoals() { while (!_landmarkQueue.empty()) _landmarkQueue.pop(); }

	bool intersects(const Util::Ray &r, float &t) { return Util::rayIntersectsCircle2D(_position, _radius, r, t); }
	bool overlaps(const Util::Point & p, float radius) { return Util::circleOverlapsCircle2D( _position, _radius, p, radius); }
	float computePenetration(const Util::Point & p, float radius) { return Util::computeCircleCirclePenetration2D( _position, _radius, p, radius); }
	void setParameters(SteerLib::Behaviour behave)
	{
		throw Util::GenericException("setParameters() not implemented yet for this Agent");
	}


	// Specialized footstep functionality:
	bool collidesAtTimeWith(const Util::Point & p1, const Util::Vector & rightSide, float otherAgentRadius, float timeStamp, float footX, float footZ);

protected:

	// returns FALSE if the input parameters create an invalid footstep.
	// step duration (time), orientation, and new velocity are the control parameters.
	bool _createFootstepAction(float stepDuration, float parabolaOrientationPhi, bool phiIsIdeal, float desiredVelocity, const Footstep & previousStep, Footstep & nextStep, FootStateEnum nextState, bool forced) const;

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


	// OBLIGATORY STEERSUITE STUFF
	bool _enabled;
	Util::Point _position;
	Util::Vector _forward;
	float _radius;
	SteerLib::AgentGoalInfo _currentGoal;
	std::queue<SteerLib::AgentGoalInfo> _landmarkQueue;


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
	std::vector<Util::Point> _waypoints;
	int _currentWaypointIndex;

	// MID-TERM PLANNING PHASE
	int _midTermPath[NEXT_WAYPOINT_DISTANCE+2];
	int _midTermPathSize;

	// SHORT-TERM PLANNING PHASE
	Util::Point _localTargetLocation; // short-term planning communicates to reactive steering phase

	// PERCEPTION PHASE
	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors; // reactive phase communicates to itself and predictive phase
	std::set<SteerLib::SpatialDatabaseItemPtr> _longView;

	// misc stuff - todo, organize these later!
	bool _needToRunFootstepPlanner;
	unsigned int _currentLocationInPath;
	bool usingDataStep;
	bool firstRun;
	static volatile unsigned int numData;
	static volatile unsigned int numPlanner;

	static const float CONFIDENCE_THRESHOLD;

	TreeInterface* classifier;
	StateConfig* featureSets;

	void handleSlice(const StateConfig::slice wedge, vector<agentInfo>& agents, vector<obstacleInfo>& obstacles, stringstream& result);
	void handleDensity(const StateConfig::density area, vector<agentInfo>& agents, vector<obstacleInfo>& obstacles, stringstream& result);
	void handleFlow(const StateConfig::flow area, vector<agentInfo>& agents, stringstream& result);
	void handleObstacles(const StateConfig::obstaclesPresent, std::vector<obstacleInfo>&, std::stringstream&);
	std::string extractContextSamples(vector<agentInfo>& agents, vector<obstacleInfo>& obstacles);
	std::string extractSpecializedSamples(unsigned int contextNumber, vector<agentInfo>& agents, vector<obstacleInfo>& obstacles);
	void localSpaceXForm(vector<agentInfo>& agents, vector<obstacleInfo>& obstacles);
	void dumpNearbyInfo(vector<agentInfo>&, vector<obstacleInfo>&, bool);

private:
	int ID;
	bool anomaly;
	float score;
	TrackReader* track_reader;
	void checkDeviation(void);
	float wakeTime;
	bool hasRun;
	bool justWoke;
	float lastCertainty;

	static const float DEVIATION_THRESHOLD;
	static const float ANOMALY_THRESHOLD;
	static const float NORMAL_THRESHOLD;
	static const float DECAY_AMOUNT;
};


#endif
