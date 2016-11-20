//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include "RVOSimulator.h"
#include "Obstacle.h"
#include "Agent.h"
#include "Goal.h"
#include "RoadmapVertex.h"
#include "KDTree.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#if HAVE_OPENMP
#include <omp.h>
#endif
#if _OPENMP
#include <omp.h>
#endif

#include <iostream>
#include <limits>
using namespace std;

const int numGroupAgentsToAdd = 1;

namespace RVO {

  //---------------------------------------------------------------
  //      Implementation of RVO Simulator
  //---------------------------------------------------------------

  RVOSimulator*  RVOSimulator::_pinstance = NULL;

  //---------------------------------------------------------------

  RVOSimulator::RVOSimulator() {
    _globalTime = 0;
    _timeStep = 0;
    _automaticRadius = -1;
    _defaultAgent = new Agent();
    _allAtGoals = false;

    _agentDefaultsHaveBeenSet = false;
    _simulationHasBeenInitialized = false;

	/*** MWS - added defaults for meta-agents ***/
	_numGroupMetaAgents = 0;
	_numFormMetaAgents = 0;
	/********************************************/
  }

  RVOSimulator::~RVOSimulator() {
    delete _defaultAgent;
    for (int i = 0; i < (int) _agents.size(); ++i) {
      delete _agents[i];
    }
    for (int i = 0; i < (int) _obstacles.size(); ++i) {
      delete _obstacles[i];
    }
    for (int i = 0; i < (int) _goals.size(); ++i) {
      delete _goals[i];
    }
    for (int i = 0; i < (int) _roadmapVertices.size(); ++i) {
      delete _roadmapVertices[i];
    }
  }

  //---------------------------------------------------------------

  RVOSimulator* RVOSimulator::Instance() {
    if (_pinstance == NULL)  {
	  /*** MWS - always seed same value ***/
      //srand((unsigned int) time(NULL));
	  srand(500);
	  /************************************/
      _pinstance = new RVOSimulator(); // create sole instance
    }
    return _pinstance; // address of sole instance
  }

  void RVOSimulator::resetSimulation ()
  {

	  _agents.clear ();
	  _goals.clear ();
	  _roadmapVertices.clear ();

	// cleanup
    delete _defaultAgent;
	/*
    for (int i = 0; i < (int) _agents.size(); ++i) {
      delete _agents[i];
    }
    for (int i = 0; i < (int) _obstacles.size(); ++i) {
      delete _obstacles[i];
    }
    for (int i = 0; i < (int) _goals.size(); ++i) {
      delete _goals[i];
    }
    for (int i = 0; i < (int) _roadmapVertices.size(); ++i) {
      delete _roadmapVertices[i];
    }
	*/

	// init
    _globalTime = 0;
    _timeStep = 0;
    _automaticRadius = -1;
    _defaultAgent = new Agent();
    _allAtGoals = false;

    _agentDefaultsHaveBeenSet = false;
    _simulationHasBeenInitialized = false;

	/*** MWS - added defaults for meta-agents ***/
	_numGroupMetaAgents = 0;
	_numFormMetaAgents = 0;
	/********************************************/

  }

  //---------------------------------------------------------------

  int RVOSimulator::doStep()
  {

	std::cout << "rvo sim do step \n";

    if ( ! _simulationHasBeenInitialized )
	{
      std::cerr << "Simulation is not initialized when attempt is made to do a step." << std::endl;
      return RVO_SIMULATION_NOT_INITIALIZED_WHEN_DOING_STEP;
    }
    if ( _timeStep == 0 )
	{
      std::cerr << "Time step has not been set when attempt is made to do a step." << std::endl;
      return RVO_TIME_STEP_NOT_SET_WHEN_DOING_STEP;
    }

#ifdef SITUATION_AGENT
	/*** MWS - setup group meta-agents *************/
	// get all the possible groups
	std::vector< std::pair<int,int> > posGroups;
	const int numAgents = (int)_agents.size();
	for(int i = 0; i < numAgents; ++i)
	{
		if(0 == _agents[i]->_class && true == _agents[i]->_active)
			posGroups.push_back(make_pair(_agents[i]->_posGroupMembers,i));
	}
	sort(posGroups.begin(), posGroups.end());
	reverse(posGroups.begin(), posGroups.end());

	// re-active the metaAgents as required
	if(false == posGroups.empty())
	{
		for(int i = 0; i < numGroupAgentsToAdd; ++i)
		{
			// see if we have enough agents
			if(posGroups[i].first < 1)
				break;

			// find an empty spot
			int emptyIndex = -1;
			for(int j = 0; j < _numGroupMetaAgents; ++j)
			{
				if(false == _agents[j]->_active)
				{
					emptyIndex = j;
					break;
				}
			}

			// if found nothing then done
			if(-1 == emptyIndex)
				break;

			// activate the group meta-agent
			setGroupAgent(emptyIndex);

			// set a group meta-agent near the largest group
			Vector2 agentPos(getAgentPosition(posGroups[i].second));
			agentPos += Vector2(0.001f,0.001f);
			setAgentPosition(emptyIndex, agentPos);

			// set agent's goal
			_agents[emptyIndex]->_subGoal = _agents[posGroups[i].second]->_subGoal;
			_agents[emptyIndex]->_goalID = _agents[posGroups[i].second]->_goalID;

			// set agent characteristics
			setAgentPrefSpeed(emptyIndex, getAgentPrefSpeed(posGroups[i].second));
			setAgentVelocity(emptyIndex, getAgentVelocity(posGroups[i].second));
			setAgentMaxSpeed(emptyIndex, getAgentMaxSpeed(posGroups[i].second));
		}
	}
	/***********************************************/
#endif

	std::cout << "rvo sim do step 0\n";
    _allAtGoals = true;

	std::cout << " rvo sim do step 1\n";
    // compute new velocities for agents
    _kdTree->buildAgentTree();

  #pragma omp parallel for
    for (int i = 0; i < (int)_agents.size(); ++i) {
		std::cout << " rvo sim do step agent loop \n";

      /*** MWS - don't bother with inactive agents ***/
	  if(false == _agents[i]->_active)
			continue;
      /***********************************************/

      _agents[i]->computePreferredVelocity();  // alters _allAtGoals
      _agents[i]->computeNeighbors();
      _agents[i]->computeNewVelocity(_timeStep);
    }

    // update positions and velocity of _agents
  #pragma omp parallel for
    for (int i = 0; i < (int)_agents.size(); ++i) {
      _agents[i]->update();
    }

    _globalTime += _timeStep;

    return RVO_SUCCESS;
  }

  //----------------------------------------------------------------------------------
  // Global Getters/Setters
  bool RVOSimulator::getReachedGoal() const { return _allAtGoals; }
  float RVOSimulator::getGlobalTime() const { return _globalTime; }
  float RVOSimulator::getTimeStep() const { return _timeStep; }
  void RVOSimulator::setTimeStep( float stepSize ) { _timeStep = stepSize; }

  // Agent Getters/Setters
  int RVOSimulator::getNumAgents() const { return (int) _agents.size(); }
  bool RVOSimulator::getAgentReachedGoal(int i) const { return _agents[i]->_atGoal; }
  const Vector2& RVOSimulator::getAgentPosition(int i) const { return _agents[i]->_p; }
  void RVOSimulator::setAgentPosition(int i, const Vector2& p) { _agents[i]->_p = p; }
  const Vector2& RVOSimulator::getAgentVelocity(int i) const { return _agents[i]->_v; }
  void RVOSimulator::setAgentVelocity(int i, const Vector2& v) { _agents[i]->_v = v; }
  float RVOSimulator::getAgentRadius(int i) const { return _agents[i]->_r; }
  void RVOSimulator::setAgentRadius(int i, float r)
  {
	  _agents[i]->_r = r;
	  _agents[i]->_storedR = r;
  }
  int RVOSimulator::getAgentVelSampleCount(int i) const { return _agents[i]->_velSampleCount; }
  void RVOSimulator::setAgentVelSampleCount(int i, int samples) { _agents[i]->_velSampleCount = samples; }
  float RVOSimulator::getAgentNeighborDist(int i) const { return _agents[i]->_neighborDist; }
  void RVOSimulator::setAgentNeighborDist(int i, float distance) { _agents[i]->_neighborDist = distance; }
  int RVOSimulator::getAgentMaxNeighbors(int i) const { return _agents[i]->_maxNeighbors; }
  void RVOSimulator::setAgentMaxNeighbors(int i, int maximum) { _agents[i]->_maxNeighbors = maximum; }
  int RVOSimulator::getAgentClass(int i) const { return _agents[i]->_class; }
  void RVOSimulator::setAgentClass(int i, int classID) { _agents[i]->_class = classID; }
  float RVOSimulator::getAgentOrientation(int i) const { return _agents[i]->_o; }
  void RVOSimulator::setAgentOrientation(int i, float o) { _agents[i]->_o = o; }
  int RVOSimulator::getAgentGoal(int i) const { return _agents[i]->_goalID; }
  void RVOSimulator::setAgentGoal(int i, int goalID) { _agents[i]->_goalID = goalID; }
  float RVOSimulator::getAgentGoalRadius(int i) const { return _agents[i]->_gR; }
  void RVOSimulator::setAgentGoalRadius(int i, float gR) { _agents[i]->_gR = gR; }
  float RVOSimulator::getAgentPrefSpeed(int i) const { return _agents[i]->_prefSpeed; }
  void RVOSimulator::setAgentPrefSpeed(int i, float prefSpeed)
  {
	  _agents[i]->_prefSpeed = prefSpeed;
	  _agents[i]->_storedPrefSpeed = prefSpeed;
  }
  float RVOSimulator::getAgentMaxSpeed(int i) const { return _agents[i]->_maxSpeed; }
  void RVOSimulator::setAgentMaxSpeed(int i, float maxSpeed) { _agents[i]->_maxSpeed = maxSpeed; }
  float RVOSimulator::getAgentMaxAccel(int i) const { return _agents[i]->_maxAccel; }
  void RVOSimulator::setAgentMaxAccel(int i, float maxAccel) { _agents[i]->_maxAccel = maxAccel; }
  float RVOSimulator::getAgentSafetyFactor(int i) const { return _agents[i]->_safetyFactor; }
  void RVOSimulator::setAgentSafetyFactor(int i, float safetyFactor)
  {
	  _agents[i]->_safetyFactor = safetyFactor;
	  _agents[i]->_storedSafetyFactor = safetyFactor;
  }

  /*** MWS - added alpha getter/setter ***************************************/
  void RVOSimulator::setAgentAlpha(int i, float alpha) { _agents[i]->_alpha = alpha; }
  float RVOSimulator::getAgentAlpha(int i) const { return _agents[i]->_alpha; }
  void RVOSimulator::setGroupAgent(int i)
  {
	 _agents[i]->_class = 1;
	 _agents[i]->_metaId[1] = i;
	 _agents[i]->_updatesToLive = 9;
	 _agents[i]->_maxNeighbors = 100;
	 _agents[i]->_neighborDist = 40;
	 _agents[i]->_maxAccel = 0.8f;
	 _agents[i]->_r = 1.0f;
	 _agents[i]->_gR = 1.0f;
	 _agents[i]->_groupLeadership = true;
	 _agents[i]->_reAdjusted = false;
	 _agents[i]->_active = true;
  };
  void RVOSimulator::setObstAgent(int i)
  {
	 _agents[i]->_class = 2;
	 _agents[i]->_metaId[2] = i;
	 _agents[i]->_maxAccel = 0.0f;
	 _agents[i]->_maxSpeed = 0.0f;
  };
  bool RVOSimulator::isObstGroupAgent(int i) const { return (2 == _agents[i]->_class); };
  bool RVOSimulator::setFormAgentMembers(const std::vector<int> &agents)
  {
     // if nothing in formation do nothing
     if(true == agents.empty())
	    return false;

	 // find any empty form agent
	 int formAgent = -1;
	 for(int ii = _numGroupMetaAgents; ii < _numGroupMetaAgents + _numFormMetaAgents; ++ii)
	 {
	    if(false == _agents[ii]->_active)
		{
		   formAgent = ii;
		   break;
		}
	 }
	 if(-1 == formAgent)
		 return false;

     // re-position the formation agent in the center of the group with pref speed of slowest
	 float minSpeed = std::numeric_limits<float>::max();
	 Vector2 averagePos(0,0);
     for(size_t i = 0; i < agents.size(); ++i)
	 {
		minSpeed = min(minSpeed,_agents[agents[i]]->_prefSpeed);
        averagePos += _agents[agents[i]]->_p;
	 }
	 averagePos = averagePos/static_cast<float>(agents.size());

     // setup the offsets in all the agents in the formation
	 float maxRad = 0;
     for(size_t i = 0; i < agents.size(); ++i)
	 {
        _agents[agents[i]]->_metaId[3] = formAgent;
	    _agents[agents[i]]->_formOffset = _agents[agents[i]]->_p - averagePos;
		_agents[agents[i]]->_storedPrefSpeed = _agents[agents[i]]->_prefSpeed;
		maxRad = max(maxRad, _agents[agents[i]]->_formOffset*_agents[agents[i]]->_formOffset);
	 }

	 // setup formation agent's info
	 _agents[formAgent]->_class = 3;
	 _agents[formAgent]->_metaId[3] = formAgent;
	 _agents[formAgent]->_p = averagePos;
	 _agents[formAgent]->_prefSpeed = minSpeed;
	 _agents[formAgent]->_goalID = _agents[agents[0]]->_goalID;
	 _agents[formAgent]->_subGoal = _agents[agents[0]]->_subGoal;
	 _agents[formAgent]->_maxSpeed = minSpeed + 0.5f;
	 _agents[formAgent]->_v = _agents[agents[0]]->_v;
	 _agents[formAgent]->_r = sqrt(maxRad) + 0.5f;
	 _agents[formAgent]->_gR = _agents[formAgent]->_r;
	 _agents[formAgent]->_neighborDist = 20.0f;
	 _agents[formAgent]->_groupLeadership = true;
	 _agents[formAgent]->_maxNeighbors = 100;
	 _agents[formAgent]->_storedPrefSpeed = minSpeed;

	 // now active
	 _agents[formAgent]->_active = true;

	 return true;
  };
  void RVOSimulator::setAgentActive(int id, bool act) { _agents[id]->_active = act; };
  bool RVOSimulator::getAgentActive(int id) const { return _agents[id]->_active; };
  void RVOSimulator::reserveMetaAgents(int numGroupMetaAgents, int numFormMetaAgents, const Vector2& startPosition, int goalID,
	  int velSampleCount, float neighborDist, int maxNeighbors, float radius, float goalRadius, float prefSpeed, float maxSpeed,
	  float safetyFactor, float maxAccel, const Vector2 &velocity, float orientation)
  {
     // save the values
     _numGroupMetaAgents = numGroupMetaAgents;
	 _numFormMetaAgents = numFormMetaAgents;

     // add all group meta-agents up front
	 for(int i = 0; i < numGroupMetaAgents; ++i)
	 {
	    // add another meta-agent
        int id = addAgent(startPosition, goalID, velSampleCount, neighborDist, maxNeighbors, radius, goalRadius, prefSpeed,
			maxSpeed, safetyFactor, maxAccel, velocity, orientation, 1);
		setAgentActive(id,false);
	 }

	 // add all form meta-agents up front
	 for(int i = 0; i < numFormMetaAgents; ++i)
	 {
	    // add another meta-agent
        int id = addAgent(startPosition, goalID, velSampleCount, neighborDist, maxNeighbors, radius, goalRadius, prefSpeed,
			maxSpeed, safetyFactor, maxAccel, velocity, orientation, 3);
		setAgentActive(id,false);
	 }
  }
  std::vector<int> RVOSimulator::getGroupMetaAgentIDs(void) const
  {
	  std::vector<int> temp;
	  for(int i = 0; i < _numGroupMetaAgents; ++i)
		  temp.push_back(i);

	  return temp;
  }
  std::vector<int> RVOSimulator::getFormMetaAgentIDs(void) const
  {
	  std::vector<int> temp;
	  for(int i = _numGroupMetaAgents; i < _numGroupMetaAgents + _numFormMetaAgents; ++i)
		  temp.push_back(i);

	  return temp;
  }
  /***************************************************************************/

  // Goal Getter/Setter 's
  int RVOSimulator::getNumGoals() const { return (int) _goals.size(); }
  const Vector2& RVOSimulator::getGoalPosition(int i) const { return _goals[i]->_vertex->_p; }
  int RVOSimulator::getGoalNumNeighbors(int i) const { return (int) _goals[i]->_vertex->_neighbors.size(); }
  int RVOSimulator::getGoalNeighbor(int i, int n) const { return _goals[i]->_vertex->_neighbors[n].second; }

  // Obstacle Getter/Setter 's
  int RVOSimulator::getNumObstacles() const { return (int) _obstacles.size(); }
  const Vector2& RVOSimulator::getObstaclePoint1(int i) const { return _obstacles[i]->_p1; }
  const Vector2& RVOSimulator::getObstaclePoint2(int i) const { return _obstacles[i]->_p2; }

  // Roadmap Getters/Setter 's
  int RVOSimulator::getNumRoadmapVertices() const { return (int) _roadmapVertices.size(); }
  const Vector2& RVOSimulator::getRoadmapVertexPosition(int i) const { return _roadmapVertices[i]->_p; }
  int RVOSimulator::getRoadmapVertexNumNeighbors(int i) const { return (int) _roadmapVertices[i]->_neighbors.size(); }
  int RVOSimulator::getRoadmapVertexNeighbor(int i, int n) const { return _roadmapVertices[i]->_neighbors[n].second; }

  // Agent adders
  void RVOSimulator::setAgentDefaults( int velSampleCountDefault, float neighborDistDefault, int maxNeighborsDefault, float rDefault, float gRDefault, float prefSpeedDefault, float maxSpeedDefault, float safetyFactorDefault, float maxAccelDefault, const Vector2& vDefault, float oDefault, int classDefault ) {
    _agentDefaultsHaveBeenSet = true;

    _defaultAgent->_velSampleCount = velSampleCountDefault;
    _defaultAgent->_neighborDist = neighborDistDefault;
    _defaultAgent->_maxNeighbors = maxNeighborsDefault;

    _defaultAgent->_class = classDefault;
    _defaultAgent->_r = rDefault;
    _defaultAgent->_v = vDefault;
    _defaultAgent->_maxAccel = maxAccelDefault;
    _defaultAgent->_gR = gRDefault;
    _defaultAgent->_prefSpeed = prefSpeedDefault;
    _defaultAgent->_maxSpeed = maxSpeedDefault;
    _defaultAgent->_o = oDefault;
    _defaultAgent->_safetyFactor = safetyFactorDefault;
  }

  int RVOSimulator::addAgent(const Vector2& startPosition, int goalID) {
    if (_simulationHasBeenInitialized) {
      std::cerr << "Simulation already initialized when adding agent." << std::endl;
      return RVO_SIMULATION_ALREADY_INITIALIZED_WHEN_ADDING_AGENT;
    }
    if ( ! _agentDefaultsHaveBeenSet ) {
      std::cerr << "Agent defaults have not been set when adding agent." << std::endl;
      return RVO_AGENT_DEFAULTS_HAVE_NOT_BEEN_SET_WHEN_ADDING_AGENT;
    }
    Agent* agent = new Agent(startPosition, goalID);
    _agents.push_back(agent);
	/*** MWS - added keeping metaId set ***/
	_agents.back()->_metaId[0] = _agents.size() - 1;
	/**************************************/
    return (int) _agents.size() - 1;
  }

  int RVOSimulator::addAgent(const Vector2& startPosition, int goalID, int velSampleCount, float neighborDist, int maxNeighbors, float r, float gR, float prefSpeed, float maxSpeed, float safetyFactor, float maxAccel, const Vector2& v, float o, int classID ) {
    if (_simulationHasBeenInitialized) {
      std::cerr << "Simulation already initialized when adding agent." << std::endl;
      return RVO_SIMULATION_ALREADY_INITIALIZED_WHEN_ADDING_AGENT;
    }
    Agent* agent = new Agent(startPosition, goalID, velSampleCount, neighborDist, maxNeighbors, classID, r, v, maxAccel, gR, prefSpeed, maxSpeed, o, safetyFactor);
    _agents.push_back(agent);
	/*** MWS - added keeping metaId set ***/
	_agents.back()->_metaId[0] = _agents.size() - 1;
	/**************************************/
    return (int) _agents.size() - 1;
  }

  // Goal adder
  int RVOSimulator::addGoal(const Vector2& position) {
    if (_simulationHasBeenInitialized) {
      std::cerr << "Simulation already initialized when adding goal." << std::endl;
      return RVO_SIMULATION_ALREADY_INITIALIZED_WHEN_ADDING_GOAL;
    }
    Goal* goal = new Goal(position);
    _goals.push_back(goal);
    return (int) _goals.size() - 1;
  }

  // Obstacle Adder
  int RVOSimulator::addObstacle(const Vector2& point1, const Vector2& point2) {
    if (_simulationHasBeenInitialized) {
      std::cerr << "Simulation already initialized when adding obstacle." << std::endl;
      return RVO_SIMULATION_ALREADY_INITIALIZED_WHEN_ADDING_OBSTACLE;
    }
    Obstacle* obstacle = new Obstacle(point1, point2);
    _obstacles.push_back(obstacle);
    return (int) _obstacles.size() - 1;
  }

  // Roadmap Adders
  void RVOSimulator::setRoadmapAutomatic(float automaticRadius) {
    _automaticRadius = automaticRadius;
  }

  int RVOSimulator::addRoadmapVertex(const Vector2& position) {
    if (_simulationHasBeenInitialized) {
      std::cerr << "Simulation already initialized when adding roadmap vertex." << std::endl;
      return RVO_SIMULATION_ALREADY_INITIALIZED_WHEN_ADDING_ROADMAP_VERTEX;
    }
    RoadmapVertex * vertex = new RoadmapVertex( position );
    _roadmapVertices.push_back(vertex);
    return (int) _roadmapVertices.size() - 1;
  }

  int RVOSimulator::addRoadmapEdge(int vertexID1, int vertexID2) {
    if (_simulationHasBeenInitialized) {
      std::cerr << "Simulation already initialized when adding roadmap edge." << std::endl;
      return RVO_SIMULATION_ALREADY_INITIALIZED_WHEN_ADDING_ROADMAP_EDGE;
    }

    float dist = abs(_roadmapVertices[vertexID1]->_p - _roadmapVertices[vertexID2]->_p);
    _roadmapVertices[vertexID1]->addNeighbor(dist, vertexID2);
    _roadmapVertices[vertexID2]->addNeighbor(dist, vertexID1);

    return RVO_SUCCESS;
  }

  // Initialize Simulation
  void RVOSimulator::initSimulation() {

	std::cout << " rvo simulator init simulation \n";
    _simulationHasBeenInitialized = true;

    _kdTree = new KDTree();
    _kdTree->buildObstacleTree();
	std::cout << " rvo simulator init simulation 2\n";

	std::cout << " road map size " << _roadmapVertices.size () << "\n";
	std::cout << " goals size " << _goals.size () << "\n";

    if (_automaticRadius >= 0) {
      #pragma omp parallel for
      for (int i = 0; i < (int) _roadmapVertices.size(); ++i) {
	std::cout << " rvo simulator init simulation 3\n";
        _roadmapVertices[i]->computeNeighbors(_automaticRadius);
      }
    }

    #pragma omp parallel for
    for (int i = 0; i < (int) _goals.size(); ++i) {
	std::cout << " rvo simulator init simulation 4\n";
      _goals[i]->computeShortestPathTree();
    }
  }
}
