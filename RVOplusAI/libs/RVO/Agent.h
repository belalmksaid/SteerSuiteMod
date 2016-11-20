//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

/* \file Agent.h Contains the class Agent. */

#ifndef __AGENT_H__
#define __AGENT_H__

#define AGENT 0
#define OBSTACLE 1

#include "RVODef.h"

namespace RVO {
  /* The class defining a planning agent -- uses Reciprocal Velocity Obstacles to actually plan the state for the next time step */
  class Agent {
  private:
    /* Constructor. */
    Agent();

    /* Constructor. Constructs an agent with default parameters. 
     \param p The start position of the agent
     \param goalID The ID of the goal of the agent
     */
    Agent( const Vector2& p, int goalID );
    
    /* Constructor. Constructs an agent with specified parameters. 
     \param p The start position of the agent
     \param goalID The ID of the goal of the agent
     \param velSampleCount Determines the number of candidate velocities sampled when the agent is finding a new velocity. The running time of the simulation increases linearly with this number.
     \param neighborDist The maximum distance from the agent at which an object will be considered for velocity obstacles. The larger this number, the larger the running time of the simulation. If the number is too low, the simulation will not be safe.
     \param maxNeighbors Maximum number of neighbors considered for velocity obstacles. The larger this number, the larger the running time of the simulation. If the number is too low, the simulation will not be safe.
     \param classID The class of the agent; the class of an agent does not affect the simulation, but can be used in external applications to distinguish among agents 
     \param r The radius of the agent
     \param v The initial velocity of the agent
     \param maxAccel The maximum acceleration of the agent
     \param gR The goal radius of the agent; an agent is said to have reached its goal when it is within a distance goalRadius from its goal position.
     \param prefSpeed The preferred speed of the agent
     \param maxSpeed The maximum speed of the agent
     \param o The initial orientation of the agent
     \param safetyFactor The safety factor of the agent; the higher the safety factor, the less 'aggressive' an agent is
     */
    Agent( const Vector2& p, int goalID, int velSampleCount, float neighborDist, int maxNeighbors, int classID, float r, const Vector2& v, float maxAccel, float gR, float prefSpeed, float maxSpeed, float o, float safetyFactor );
    

    /* Deconstructor */
    ~Agent();

    /* Computes the new velocity for the next time step */
    void computeNewVelocity(float timeStep);

    /* Computes the neighbors which will be considered in finding the next velocity */
    void computeNeighbors(void);

    /* Computes the preferred velocity (based on the roadmap) */
    void computePreferredVelocity(void);

    /* Updates the state of the agent having selected a new velocity */
    void update(void);

    void insertAgentNeighbor(int id, float& rangeSq);
    void insertObstacleNeighbor(int id, float& rangeSq);

	/*** MWS - added functionality ***/
	float uniformRand(void) const { return float(std::rand())/float(RAND_MAX); };
	float intersection(const Vector2 &p1, const Vector2 &p2, const Vector2 &p3, const Vector2 &p4) const;
	bool sameSubGoal(const Agent &a) const;
	bool sameGoal(const Agent &a) const;
	bool samePrefV(const Agent &a) const;
	/*********************************/

    /* The 2D position of the agent */
    Vector2 _p;

    /* The 2D velocity of the agent */
    Vector2 _v;

    /* The radius of the agent */
    float _r;

    /* An arbitray specification of class for the agent
       - it can be used by a visualizer to associate agent class with a visualization
         method */
    int _class;

    /* The orientation of the agent */;
    float _o;

    /* The ID of the goal of the agent */
    int _goalID;

    /* The goal radius of the agent; an agent is said to have reached its goal when it is within a distance _gR from its goal position. */
    float _gR;

    /* The preferred speed of the agent */
    float _prefSpeed;

    /* The maximum speed of the agent */
    float _maxSpeed;

    /* The maximum acceleration of the agent */
    float _maxAccel;

    /* The safety factor of the agent (weight for penalizing candidate velocities - the higher the safety factor, the less 'aggressive' an agent is) */
    float _safetyFactor;

    /* Determines the number of candidate velocities sampled when the agent is finding a new velocity. The
        running time of the simulation increases linearly with this number. */
    int  _velSampleCount;

    /* The maximum distance from the agent at which an object will be considered for velocity obstacles. The larger this number, the larger the running
        time of the simulation. If the number is too low, the simulation will not be safe.*/
    float  _neighborDist;
    
    /* Maximum number of neighbors considered for velocity obstacles. The larger this number, the larger the running time of
        the simulation. If the number is too low, the simulation will not be safe. */
    int  _maxNeighbors;

    /* The preferred velocity of the agent */
    Vector2 _vPref;

    /* The new velocity of the agent */
    Vector2 _vNew;

    /* The neighbors of the agent, in order of increasing distance. The float stores the distance to the neigbor. The first int stores the type of the neighbor (AGENT or OBSTACLE). The second int stores the ID of the neighbor. */
    std::multimap<float, std::pair<int, int> > _neighbors;
    
    /* The collision status of the agent */
    bool _collision;

    /* Boolean which reports if the agent is at its goal during this time step. */
    bool _atGoal;

    /* The subgoal of the agent (is an ID of a roadmap vertex, or -1 if it is the goal) */
    int _subGoal;

	/*** MWS - added alpha, waiting, and bully for RVO ***/
	bool _groupLeadership;
	bool _reAdjusted;
	bool _breakForm;
	bool _reforming;
	bool _active;
	bool _obstAvoid;
	int _metaId[4];
	int _posGroupMembers;
	int _updatesToLive;
	float _alpha;
	float _storedPrefSpeed;
	float _storedSafetyFactor;
	float _storedR;
	Vector2 _formOffset;
	Vector2 _vPrefOrig;
	Vector2 _lastLeaderV;
	/*****************************************/

  protected:
    /* A reference to the singleton simulator. */
    static RVOSimulator*  _sim;

    friend class KDTree;
    friend class RVOSimulator;
  };

}    // RVO namespace

#endif