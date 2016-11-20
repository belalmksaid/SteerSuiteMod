//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#ifndef __HIDAC_AGENT__
#define __HIDAC_AGENT__

/// @file SimpleAgent.h
/// @brief Declares the SimpleAgent class.

#include <queue>
#include <list>
#include "SteerLib.h"
// #include "SimpleAgent.h"
// #include "HIDACAIModule.h"
#include "Obstacle.h"
#include "Vector2.h"
#include "HIDAC_Parameters.h"


/**
 * @brief HiDAC Agent stuff
 *
 * look for calculateSteps()
 *
 */

#define DRAW_ANNOTATIONS 1


class HIDACAgent : public SteerLib::AgentInterface
{
public:
	HIDACAgent();
	~HIDACAgent();
	void reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo);
	void updateAI(float timeStamp, float dt, unsigned int frameNumber);
	void disable();
	void draw();

	bool enabled() const { return _enabled; }
	Util::Point position() const { return _position; }
	Util::Vector forward() const { return _forward; }
	Util::Vector velocity() const {return _velocity; }
	float radius() const { return _radius; }
	const SteerLib::AgentGoalInfo & currentGoal() const { return _goalQueue.front(); }
	size_t id() const { return id_;}
	const std::queue<SteerLib::AgentGoalInfo> & agentGoals() const { return _goalQueue; }
	void addGoal(const SteerLib::AgentGoalInfo & newGoal) { throw Util::GenericException("addGoals() not implemented yet for HIDACAgent"); }
	void clearGoals() { throw Util::GenericException("clearGoals() not implemented yet for HIDACAgent"); }
	void setParameters(SteerLib::Behaviour behave);
	/// @name The SteerLib::SpatialDatabaseItemInterface
	/// @brief These functions are required so that the agent can be used by the SteerLib::GridDatabase2D spatial database;
	/// The Util namespace helper functions do the job nicely for basic circular agents.
	//@{
	bool intersects(const Util::Ray &r, float &t) { return Util::rayIntersectsCircle2D(_position, _radius, r, t); }
	bool overlaps(const Util::Point & p, float radius) { return Util::circleOverlapsCircle2D( _position, _radius, p, radius); }
	float computePenetration(const Util::Point & p, float radius) { return Util::computeCircleCirclePenetration2D( _position, _radius, p, radius); }
	//@}

	// bool collidesAtTimeWith(const Util::Point & p1, const Util::Vector & rightSide, float otherAgentRadius, float timeStamp, float footX, float footZ);
	void insertAgentNeighbor(const SteerLib::AgentInterface * agent, float &rangeSq) {throw Util::GenericException("clearGoals() not implemented yet for HIDACAgent");}
	// bool compareDist(SteerLib::AgentInterface * a1, SteerLib::AgentInterface * a2 );

protected:

	virtual SteerLib::EngineInterface * getSimulationEngine();
	/// Updates position, velocity, and orientation of the agent, given the force and dt time step.
	// void _doEulerStep(const Util::Vector & steeringDecisionForce, float dt);

	HIDACParameters _HIDACParams;


	/**
		 * \brief   Updates the three-dimensional position and three-dimensional velocity of this agent.
		 */
	void update(float timeStamp, float dt, unsigned int frameNumber);

	std::queue<SteerLib::AgentGoalInfo> _goalQueue;


	// Stuff specific to RVO
	// should be normalized
	// Util::Vector prefVelocity_; // This is the velocity the agent wants to be at
	// Util::Vector newVelocity_;
	size_t id_;
	size_t maxNeighbors_;
	float maxSpeed_;
	float neighborDist_;
	float timeHorizon_;
	float timeHorizonObst_;
	int next_waypoint_distance_;
	std::vector<std::pair<float, const SteerLib::AgentInterface *> > agentNeighbors_;
	std::vector<std::pair<float, const Obstacle *> > obstacleNeighbors_;
	std::vector<Util::Plane> orcaPlanes_;
	std::vector<Line> orcaLines_;
	SteerLib::ModuleInterface * rvoModule;

	std::vector<Util::Point> _waypoints;

	/*************************HiDAC stuff**********************************************/
	// Util::Vector calculateStepsSocialForces();
	/*
	Util::Vector calculateStepsRuleBased(float timeStamp, float dt, unsigned int frameNumber);

	int		m_humID;

	// I'll be using hash tables instead of lists
	//map<string,int>	m_blockedTable; // Hash table with blocked cells. The number indecates the level in which i know about that cell
											// 0 means i don't know, 1 means i know because i saw it and 2 means somebody told me


	bool	m_knowledge;			// 1 Knows the building, 0 naive agent (in the future could be a percentage of knowledge)
	int		m_myCell;			// Cell towards which i'm walking
	int 	m_prevCell;			// Cell in which i am currently
	int		m_nextPortal;		// ID of the next portal i'm planning to cross
	// For walking withing the cell:
	Util::Vector	m_attractor;
	Util::Vector	m_prevAttractor;   // will be the attractor before crossing the portal, so we can calculate distances to both

	// I need to know the last attractor to remove the agents from the crossing portal list
	Util::Vector m_lastAttracPos;		// attractor of the last portal crossed, needed to update m_crossingState
	int  m_lastAttracID;
	int  m_portalIDcoll;

	//Cmaze	*m_maze;				// The humanoid needs to have access to the environment.
	// Cbuilding	*m_building;				// The humanoid needs to have access to the environment.

	SteerLib::SpatialDatabaseItemPtr	m_pcell;		// cell towards which i'm going
	SteerLib::SpatialDatabaseItemPtr	m_pprevCell;	// cell in which i am currently
	SteerLib::SpatialDatabaseItemPtr	m_pLastCell;	// last cell i was in (i need it for collision detection with other agents around portals)
	int		m_levelKnowledge;	// How much i know about the environment TAMPOCO SE USA, Y CREO Q ESTA REPETIDO


	// When the humanoid is lost, he'll start exploring the maze using a DFS algorithm:
	//map<string, DFSdata>	m_visitedCells;
	// map<int, DFSdata>	m_visitedCells;	// hash table that for each cell i know about, i can access it's info and it's adjacent cells
	// int						timeDFS;

	//
	std::list<Util::Vector> m_route;	// List of steps walked by the humanoid

	// Personality factor:
	// sLeadingBehavior m_personality;

	// Elements needed to apply helbing's model:
	double  m_speed;
	double  m_maxSpeed;
	double  m_densityAhead;		//	Density ahead of the agent. Is calculated as the number of agents in the semicircle ahead of the
								// agent (dist<R, and vec(agent,others)dotVel > 0) and divided by PIxRxR/2
	double m_dt;				// diferential of time between simulations
	//float  m_time;
	long m_time;


	// For drawing purposes only:
	float	m_color[4];		// color to render the humanoid
	float	m_colorTrousers[4];		// color to render the humanoid
	float   m_colorHairOriginal[4];		// color hair //original color of hair
	float   m_colorLegs[4];

	float	m_incrx, m_incrz;
public: //funda
	Util::Vector	mInitialPosition;
	Util::Vector	mPos;
	Util::Vector	m_prevpos;
	std::list<Util::Vector> m_listPrevPos;
private:
	Util::Vector    m_desiredVel;	// Vector in the desired velocity
	Util::Vector	m_orientation; // Orientation of the Agent, to avoid using the m_desiredVel to specify the orientation, since that one has a lot of shaking
	Util::Vector	m_lookAheadA, m_lookAheadB;	// the 2 extremes of the cone in which i'm looking for collisions ahead.
	Util::Vector	m_vecA, m_vecB;
	int		m_pointSize;
	float	humBodyWidth, humBodyHeight;
	float	humRes, humHeadHeight;
	float	humLegsHeight, humLegsWidth;
	int		m_step_legs;
	bool m_doorCrossed;	// to know whether a door has been crossed and therefore the next cell neds to be calculated
	bool m_crossingPortal;
	bool m_tryingToCross;
	bool m_reached_goal;
	int m_crossingState;
	Util::Vector m_forceAvoidPerson;
	HIDACAgent *m_closerHum;
	Util::Vector m_lastUP;	// This is very usefull to mantain consistency when calculating tangential forces



// Personality properties:
	int m_letOthersFirst;
	int m_maxLetOthersFirst;	// Max num of timesteps that will wait for others to walk first

	bool male;		// Whether it's female or male
	int m_stopShaking;

	int m_bottleNeck;  // Last door where i've seen a bottleNeck


// For OCEAN model:

	int m_waiting_timer_curr;	// Waiting timer used during the simulation, when we start it it will be = m_waiting_timer
*/

private:
	bool runLongTermPlanning2();




	void calcNextStep(float dt);

	Util::Vector calcRepulsionForce(float dt);
	Util::Vector calcProximityForce(float dt);

	Util::Vector calcAgentRepulsionForce(float dt);
	Util::Vector calcWallRepulsionForce(float dt);

	Util::Vector calcWallNormal(SteerLib::ObstacleInterface* obs);
	std::pair<Util::Point, Util::Point> calcWallPointsFromNormal(SteerLib::ObstacleInterface* obs, Util::Vector normal);
	Util::Vector calcObsNormal(SteerLib::ObstacleInterface* obs);

	friend class KdTree;
	friend class HIDACAIModule;
};


#endif
