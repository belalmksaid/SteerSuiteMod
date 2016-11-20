//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include "RVOSimulator.h"
#include "RoadmapVertex.h"
#include "Goal.h"
#include "Obstacle.h"
#include "Agent.h"
#include "KDTree.h"

#include <iostream>
#include <limits>
#include <assert.h>

using namespace std;

const float smallAlpha = 0.01f;
const float pi = acos(-1.0f);
const float degToRad = pi/180.0f;

// indexes for meta-agent types
const int groupMetaIndex = 1;
const int obstMetaIndex = 2;
const int formMetaIndex = 3;

// group meta-agent values
const float minMetaRad = 1.0f;
const float maxMetaRad = 3.0f;
const float maxSeperation = 3.0f;
const float prefTol = 0.2f;
const int numUpdatesToLive = 10;

// obstacle meta-agent values
const float obstLeaderAlpha = 0.1f;
const float obstSafetyFactor = 2.0f; //3.0f;
const float obstSafetyMulti = 0.6f;
const float obstMaxRad = 8.0f;
const float incrementRad = 1.1f;
const float perpAngle = 90.0f;

// formation meta-agent values
const float formAlpha = 0.05f;
const float formSafetyFactor = 0.05f; // 0.05f -> rigid formation & 0.25f -> loose
const float closeEnough = 1.0f;       // 1.0f -> rigid formation & 0.5f -> loose


namespace RVO {
  //-----------------------------------------------------------//
  //           Implementation for class: Agent                 //
  //-----------------------------------------------------------//

  RVOSimulator*  Agent::_sim = RVOSimulator::Instance();

  //-----------------------------------------------------------

  Agent::Agent() :
     /*** MWS - added default for my settings ***/
	 _posGroupMembers(0),
     _updatesToLive(-1),
	 _groupLeadership(false),
	 _reAdjusted(false),
	 _breakForm(false),
	 _active(true),
	 _obstAvoid(false),
     _alpha(0.5f + smallAlpha*uniformRand()),
	 _formOffset(0,0)
	 /*******************************************/
  {
	  /*** MWS - added default ***/
	 _metaId[0] = -1;
	 _metaId[1] = -1;
	 _metaId[2] = -1;
	 _metaId[3] = -1;
	 _storedSafetyFactor = _safetyFactor;
	 _storedPrefSpeed = _prefSpeed;
	 _storedR = _r;
	  /***************************/
  }

  //-----------------------------------------------------------

  // MWS -- added default alpha value
  Agent::Agent( const Vector2& p, int goalID ) :
     /*** MWS - added default for my settings ***/
     _posGroupMembers(0),
     _updatesToLive(-1),
  	 _groupLeadership(false),
	 _reAdjusted(false),
	 _breakForm(false),
	 _active(true),
	 _obstAvoid(false),
     _alpha(0.5f + smallAlpha*uniformRand()),
	_formOffset(0,0)
	 /*******************************************/
  {
    _atGoal = false;
    _subGoal = -2;

    _p = p;
    _goalID = goalID;

    _velSampleCount = _sim->_defaultAgent->_velSampleCount;
    _neighborDist = _sim->_defaultAgent->_neighborDist;
    _maxNeighbors = _sim->_defaultAgent->_maxNeighbors;

    _class = _sim->_defaultAgent->_class;
    _r = _sim->_defaultAgent->_r;
    _gR = _sim->_defaultAgent->_gR;
    _prefSpeed = _sim->_defaultAgent->_prefSpeed;
    _maxSpeed = _sim->_defaultAgent->_maxSpeed;
    _maxAccel = _sim->_defaultAgent->_maxAccel;
    _o = _sim->_defaultAgent->_o;
    _safetyFactor = _sim->_defaultAgent->_safetyFactor;
    _v = _sim->_defaultAgent->_v;

	/*** MWS - added default ***/
	_metaId[0] = -1;
	_metaId[1] = -1;
	_metaId[2] = -1;
	_metaId[3] = -1;
	_storedSafetyFactor = _safetyFactor;
	_storedPrefSpeed = _prefSpeed;
	_storedR = _r;
	/***************************/
  }

  // MWS -- added default alpha value
  Agent::Agent( const Vector2& p, int goalID, int velSampleCount, float neighborDist, int maxNeighbors, int classID, float r, const Vector2& v, float maxAccel, float gR, float prefSpeed, float maxSpeed, float o, float safetyFactor ) :
     /*** MWS - added default for my settings ***/
     _posGroupMembers(0),
     _updatesToLive(-1),
  	 _groupLeadership(false),
	 _reAdjusted(false),
	 _breakForm(false),
	 _active(true),
	 _obstAvoid(false),
     _alpha(0.5f + smallAlpha*uniformRand()),
	 _formOffset(0,0)
	 /*******************************************/
  {
    _atGoal = false;
    _subGoal = -2;

    _p = p;
    _goalID = goalID;

    _velSampleCount = velSampleCount;
    _neighborDist = neighborDist;
    _maxNeighbors = maxNeighbors;

    _class = classID;
    _r = r;
    _gR = gR;
    _prefSpeed = prefSpeed;
    _maxSpeed = maxSpeed;
    _maxAccel = maxAccel;
    _o = o;
    _safetyFactor = safetyFactor;
    _v = v;

	/*** MWS - added default ***/
	_metaId[0] = -1;
	_metaId[1] = -1;
	_metaId[2] = -1;
	_metaId[3] = -1;
	_storedSafetyFactor = _safetyFactor;
	_storedPrefSpeed = _prefSpeed;
	_storedR = _r;
	/***************************/
  }

  //-----------------------------------------------------------

  Agent::~Agent() {  }

  //-----------------------------------------------------------

  // Searching for the best new velocity
  void Agent::computeNewVelocity(float timeStep)
  {

#ifdef SITUATION_AGENT
	/*** MWS - update preferred velocity based on group meta agent **************/
	_posGroupMembers = 0;
    if(-1 != _metaId[groupMetaIndex])
	{
		// see if there is still a leader
		if(true == _sim->_agents[_metaId[groupMetaIndex]]->_groupLeadership && -1 == _metaId[obstMetaIndex])
		{
			// for normal agents
			if(0 == _class && numUpdatesToLive == _sim->_agents[_metaId[groupMetaIndex]]->_updatesToLive)
			{
				Agent *other = _sim->_agents[_metaId[groupMetaIndex]];

				// calculate the vector to the center of group
				Vector2 diff(other->_p - _p);

				// if we are within the radius then use agent's velocity instead
				if(diff*diff < (other->_r)*(other->_r) && (other->_vNew)*(other->_vNew) > 0.00001f)
					diff = other->_vNew;

				// scale speed to be preferred speed at most
				const float origSpeed = sqrt(diff*diff);
				diff /= origSpeed;
				diff *= min(_prefSpeed,origSpeed);

				// make sure don't intersect a boundary
				for(std::multimap<float, std::pair<int, int> >::iterator j = _neighbors.begin(); j != _neighbors.end(); ++j)
				{
					int type = j->second.first;
					int id = j->second.second;
				
					// if this is an obsticle and we are near it
					if(OBSTACLE == type)
					{
						Obstacle *obs = _sim->_obstacles[id];
						Vector2 agentDest = _p + diff*(timeStep + _r/sqrt(diff*diff));
						const float u = intersection(_p, agentDest, obs->_p1, obs->_p2);
						if(u > 0.0f && u < 1.0f)
							diff = diff*u;
					}
				}

				// modify the velocity to point to group center
				_vPref = _vPref*0.6f + diff*0.4f;
			}
			else if(1 == _class)
			{
				// find center of mass of agents in group
				float numAgents = 0;
				Vector2 centerP(0,0);
				for(std::multimap<float, std::pair<int, int> >::iterator j = _neighbors.begin(); j != _neighbors.end(); ++j)
				{
					int type = j->second.first;
					int id = j->second.second;

					// factor into the group center calc
					if(AGENT == type && 0 == _sim->_agents[id]->_class && _metaId[0] == _sim->_agents[id]->_metaId[groupMetaIndex])
					{
						centerP += _sim->_agents[id]->_p;
						++numAgents;
					}
					else if((AGENT == type && 1 == _sim->_agents[id]->_class && false == sameSubGoal(*_sim->_agents[id])) || 3 == _sim->_agents[id]->_class)
						_updatesToLive = numUpdatesToLive;
				}

				// if no agents, then just go with meta-agent location
				if(0 == numAgents)
					centerP = _p;
				else
					centerP /= numAgents;

				// after forming group center it
				if(false == _reAdjusted && 0 != numAgents)
				{
					_reAdjusted = true;
					_p = centerP;
				}

				// find the distance of each agent from the center
				float maxRad = _r*_r*(0.97f*0.97f);
				for(std::multimap<float, std::pair<int, int> >::iterator j = _neighbors.begin(); j != _neighbors.end(); ++j)
				{
					int type = j->second.first;
					int id = j->second.second;
				
					if(AGENT == type && 0 == _sim->_agents[id]->_class)
					{
						// get pointer to agent
						Agent *other = _sim->_agents[id];

						// find distance to that agent from center
						float distSqrd = (centerP-other->_p)*(centerP-other->_p);
						distSqrd += 2.0f*sqrt(distSqrd)*0.5f + 0.25f;

						// mark with this meta-agent's ID if inside rad
						if(_goalID == other->_goalID && -1 == other->_metaId[groupMetaIndex] && distSqrd < maxMetaRad*maxMetaRad && -1 == other->_metaId[formMetaIndex])
							other->_metaId[groupMetaIndex] = _metaId[0];

						// compare to the current max
						if(_metaId[0] == other->_metaId[groupMetaIndex])
						{
							maxRad = max(maxRad, distSqrd);
							maxRad = min(max(maxRad, minMetaRad*minMetaRad),maxMetaRad*maxMetaRad);
						}
					}
				}

				// adjust radius based on agent positions
				if(numAgents > 0)
				{
					_r = sqrt(maxRad);
					_gR = _r;
				}

				// make sure the meta-agent is not too far from the group center
				if((_p-centerP)*(_p-centerP) > maxSeperation*maxSeperation)
					_groupLeadership = false;
			}
		}
		else
		{
			// leader reached goal, stop following them
			_metaId[groupMetaIndex] = -1;

			// if all leader, then stop updating as well
			if(1 == _class)
				_active = false;
		}
	}
	else if(-1 == _metaId[formMetaIndex] && -1 == _metaId[obstMetaIndex] && 0 == _class)
	{
		// have no group leader, see how big a group we could make
		for(std::multimap<float, std::pair<int, int> >::iterator j = _neighbors.begin(); j != _neighbors.end(); ++j)
		{
			int type = j->second.first;
			int id = j->second.second;
				
			if(AGENT == type && 0 == _sim->_agents[id]->_class)
			{
				// get pointer to agent
				Agent *other = _sim->_agents[id];

				// count the possible group members
				if(true == sameSubGoal(*other) && -1 == other->_metaId[groupMetaIndex] && -1 == other->_metaId[formMetaIndex] &&
					-1 == other->_metaId[obstMetaIndex] && (_p-other->_p)*(_p-other->_p) < maxMetaRad*maxMetaRad &&
					fabs(_prefSpeed-other->_prefSpeed) < prefTol)
				{
					++_posGroupMembers;
				}
			}
		}
	}
    /*************************************************************************/

	/*** MWS - added obst class processing ***********************************/
	if(-1 != _metaId[obstMetaIndex])
	{
		if(2 == _class)
		{
			// start by updating _vPref
			_vPref = (-1 != _metaId[obstMetaIndex]) ? _sim->_agents[_metaId[obstMetaIndex]]->_vPref : Vector2(0,0);

			// loop over neighbors and reduce their velocities
			bool foundSameGoalAgent = false;
			bool foundSameVAgent = false;
			bool foundLeader = false;
			float maxRad = _storedR*_storedR;
			float minDistSqrdSameV = _r*_r + 200.0f;
			float minDistSqrdSame = _r*_r + 200.0f;
			float minDistSqrdDiff = _r*_r + 200.0f;
			int minAgentID = -1;
			for(std::multimap<float, std::pair<int, int> >::iterator j = _neighbors.begin(); j != _neighbors.end(); ++j)
			{
				int type = j->second.first;
				int id = j->second.second;

				// only care about regular agents
				if(AGENT == type && 0 == _sim->_agents[id]->_class && true == _sim->_agents[id]->_active)
				{
					// get pointer to agent
					Agent *other = _sim->_agents[id];

					// see how far this is
					const float distSqrd = (_p-other->_p)*(_p-other->_p);

					// if outside radius then just stop there
					if(distSqrd > (_r + incrementRad)*(_r + incrementRad))
						continue;

					// no one avoids by default
					other->_obstAvoid = false;

					// otherwise check to see if could be closest agent
					if((true == sameGoal(*other) && distSqrd < minDistSqrdSame) ||
					   (false == foundSameGoalAgent && true == samePrefV(*other) && distSqrd < minDistSqrdSameV) ||
					   (false == foundSameVAgent && false == foundSameGoalAgent && false == sameGoal(*other) && distSqrd < minDistSqrdDiff))
					{
						// have new next-in-line leader
						minAgentID = id;

						// save min dist
						if(true == sameGoal(*other))
						{
							minDistSqrdSame = distSqrd;
							foundSameGoalAgent = true;
						}
						else if(true == samePrefV(*other))
						{
							minDistSqrdSameV = distSqrd;
							foundSameVAgent = true;
						}
						else
							minDistSqrdDiff = distSqrd;
					}

					// mark new group members
					if(_metaId[0] != other->_metaId[obstMetaIndex])
					{
						other->_metaId[obstMetaIndex] = _metaId[0];
						other->_metaId[groupMetaIndex] = -1;
					}

					// process group members
					if(_metaId[obstMetaIndex] == id && _metaId[0] == other->_metaId[obstMetaIndex])
					{
						// if this is the leader of the group
						foundLeader = true;

						// copy the leader's goal to this agent
						_goalID = other->_goalID;
						_subGoal = other->_subGoal;

						// set behavior parameters
						other->_prefSpeed = other->_storedPrefSpeed;
						other->_safetyFactor = other->_storedSafetyFactor*obstSafetyMulti;
						other->_alpha = obstLeaderAlpha;
					}
					else if(_vPref*other->_vPref/(abs(_vPref)*abs(other->_vPref)) > 0.1f)
					{
						// member moving with leader get decrease in alpha & full speed
						other->_prefSpeed = other->_storedPrefSpeed;
						other->_safetyFactor = other->_storedSafetyFactor*obstSafetyMulti;
						other->_alpha = obstLeaderAlpha;
					}
					else /*if(_vPref*other->_vPref/(abs(_vPref)*abs(other->_vPref)) < -0.1f)*/
					{
						// members moving against leader get reduced speed & increase safety
						other->_safetyFactor = obstSafetyFactor;
						other->_alpha = 0.5f;
						other->_obstAvoid = true;

						// see if this is a new max
						maxRad = max(maxRad, distSqrd);
					}
				}
			}

			// if didn't find leader, then need new one
			if(false == foundLeader && -1 != minAgentID)
			{
				Agent *other = _sim->_agents[minAgentID];

				_metaId[obstMetaIndex] = minAgentID;
				_goalID = other->_goalID;
				_subGoal = other->_subGoal;

				other->_prefSpeed = other->_storedPrefSpeed;
				other->_safetyFactor = other->_storedSafetyFactor*obstSafetyMulti;
				other->_alpha = obstLeaderAlpha;
			}

			// can skip all the rest because won't move
			_vNew = Vector2(0,0);
			_vPrefOrig = _vPref;

			// increase or decrease rad as required
			_r = sqrt(maxRad);
			_neighborDist = _r + incrementRad;

			return;
		}
		else if(0 == _class)
		{
			// if outside rad, then reset
			Agent *obst = _sim->_agents[_metaId[obstMetaIndex]];
			const float dist = (_p-obst->_p)*(_p-obst->_p);
			if(dist > (obst->_r)*(obst->_r) + 0.01f)
			{
				_metaId[obstMetaIndex] = -1;
				_prefSpeed = _storedPrefSpeed;
				_safetyFactor = _storedSafetyFactor;
				_alpha = 0.5f + smallAlpha*uniformRand();
				_obstAvoid = false;
			}
			else if(true == _obstAvoid)
			{
				// calculate the perp velocity
				float rad = atan2(_p.y() - obst->_p.y(),_p.x()- obst->_p.x()) + perpAngle*degToRad;
				Vector2 dPerp(cos(rad),sin(rad));
				Vector2 diff(_vPref*0.4f + dPerp*_prefSpeed*0.6f);

				// make sure don't intersect a boundary
				for(std::multimap<float, std::pair<int, int> >::iterator j = _neighbors.begin(); j != _neighbors.end(); ++j)
				{
					int type = j->second.first;
					int id = j->second.second;
				
					// if this is an obsticle and we are near it
					if(OBSTACLE == type)
					{
						Obstacle *obs = _sim->_obstacles[id];
						Vector2 agentDest = _p + diff*(timeStep + _r/sqrt(diff*diff));
						const float u = intersection(_p, agentDest, obs->_p1, obs->_p2);
						if(u > 0.0f && u < 1.0f)
							diff = diff*u;
					}
				}

				_vPref = diff;
			}
		}
	}
	/****************************************************************************/

	/*** MWS - added formation processing ***************************************/
	if(-1 != _metaId[formMetaIndex])
	{
		if(true == _sim->_agents[_metaId[formMetaIndex]]->_groupLeadership)
		{
			// for now just deal with the regular agents
			if(0 == _class)
			{
				// make sure still leading
				if(true == _sim->_agents[_metaId[formMetaIndex]]->_groupLeadership && false == _sim->_agents[_metaId[formMetaIndex]]->_breakForm)
				{
					// figure out max allowable velocity to get back into form
					Vector2 formVel((_sim->_agents[_metaId[formMetaIndex]]->_p + _formOffset - _p)/timeStep);

					// this agent's velocity should be the same as the form agent
					_vPref = _sim->_agents[_metaId[formMetaIndex]]->_vNew;

					// add the velocity to restore form
					if(formVel*formVel > 0.0001f)
					{
						_vPref += formVel*closeEnough;
						float combSpeed = sqrt(_vPref*_vPref);
						_vPref /= combSpeed;
						_vPref *= min(_prefSpeed + _maxAccel*timeStep, combSpeed);
					}

					// alpha will be normal when the group is first forming
					if(formAlpha != _alpha)
						_alpha = formAlpha;

					// modify safety factor
					_safetyFactor = formSafetyFactor;
				}
				else if(true == _sim->_agents[_metaId[formMetaIndex]]->_breakForm)
				{
					// only taking a break, need to reset parameters
					if(formAlpha == _alpha)
						_alpha = 0.5f + smallAlpha*uniformRand();

					// normal safety & alpha
					_safetyFactor = _storedSafetyFactor*0.05f + _safetyFactor*0.95f;
				}
				else
				{
					// when leadership ends, reset parameters
					_safetyFactor = _storedSafetyFactor;
					_alpha = 0.5f + smallAlpha*uniformRand();
					_metaId[formMetaIndex] = -1;
				}
			}
			else if(3 == _class)
			{
				if(false == _sim->_agents[_metaId[formMetaIndex]]->_breakForm)
				{
					// if need to re-asign agents do that first
					if(true == _reforming)
					{
						// get positions and offsets of all agents in the group
						Vector2 frontDir(cos(_o),sin(_o));
						Vector2 sideDir(cos(_o+pi/2.0f),sin(_o+pi/2.0f));
						std::vector< std::pair<Vector2,Vector2> > offsets;
						std::vector< std::pair<Vector2,int> > positions;
						for(std::multimap<float, std::pair<int, int> >::iterator j = _neighbors.begin(); j != _neighbors.end(); ++j)
						{
							int type = j->second.first;
							int id = j->second.second;

							// only look at regualr agents which are members of the form
							if(AGENT == type && 0 == _sim->_agents[id]->_class && _metaId[0] == _sim->_agents[id]->_metaId[formMetaIndex])
							{
								// get pointer to agent
								Agent *other = _sim->_agents[id];

								float frontProj = frontDir*(other->_p-_p);
								float sideProj = sideDir*(other->_p-_p);
								positions.push_back(std::make_pair(Vector2(frontProj,sideProj),id));

								frontProj = frontDir*(other->_formOffset);
								sideProj = sideDir*(other->_formOffset);
								offsets.push_back(std::make_pair(Vector2(frontProj,sideProj),other->_formOffset));
							}
						}

						// sort offsets according to distance from the front of group agent
						std::sort(offsets.begin(),offsets.end());
						std::reverse(offsets.begin(),offsets.end());
						std::vector<bool> taken(positions.size(),false);

						// re-assign offsets based on distance
						for(size_t ii = 0; ii < offsets.size(); ++ii)
						{
							float minDist = std::numeric_limits<float>::max();
							size_t minIndex = 0;
							for(size_t jj = 0; jj < positions.size(); ++jj)
							{
								if(true == taken[jj])
									continue;

								float dist = abs(offsets[ii].first - positions[jj].first);

								if(dist < minDist)
								{
									minDist = dist;
									minIndex = jj;
								}
							}

							assert(minDist != std::numeric_limits<float>::max());

							_sim->_agents[positions[minIndex].second]->_formOffset = offsets[ii].second;
							taken[minIndex] = true;
						}

						// re-forming done
						_reforming = false;
					}

					// go through neighbors to find how well they are formed
					float maxDistSqrd = 0;
					for(std::multimap<float, std::pair<int, int> >::iterator j = _neighbors.begin(); j != _neighbors.end(); ++j)
					{
						int type = j->second.first;
						int id = j->second.second;

						// only look at regualr agents which are members of the form
						if(AGENT == type && 0 == _sim->_agents[id]->_class && _metaId[0] == _sim->_agents[id]->_metaId[formMetaIndex])
						{
							// get pointer to agent
							Agent *other = _sim->_agents[id];
							float distSqrd = (_p + other->_formOffset - other->_p)*(_p + other->_formOffset - other->_p);
							maxDistSqrd = max(distSqrd, maxDistSqrd);
						}
					}

					// if agents are out of form, then slow down so they can get back in
					_prefSpeed = (maxDistSqrd > 0.5f*0.5f) ? 0.8f*_storedPrefSpeed : _storedPrefSpeed;
				}
				else
					_prefSpeed = _storedPrefSpeed;
			}
		}
		else
		{
			// leader reached goal, stop following them
			_metaId[formMetaIndex] = -1;

			// if all leader, then stop updating as well
			if(3 == _class)
				_active = false;
		}
	}
	/****************************************************************************/
#endif

    float min_penalty = RVO_INFTY;
    Vector2 vCand;

    // Select num_samples candidate velocities within the circle of radius _maxSpeed
    for (int n = 0; n < _velSampleCount; ++n)
	{
      if (n == 0)
        vCand = _vPref;
	  /*** MWS - Added extra velocity selection tricks ***/
	  else if(n == 1)
	    vCand = Vector2((2.0f*rand())/((float)RAND_MAX)-1.0f,(2.0f*rand())/((float)RAND_MAX)-1.0f)*0.05f;
	  else if(n < 11)
	  {
	  	  float deg = (n-2)*16.0f*degToRad;
	  	  vCand = Vector2(cos(deg),sin(deg))*min(_maxSpeed,_r/4.0f);
	  }
	  else if(n < 21)
	  {
		float x1, x2, w, y1, y2;
		// get two normally distributed random numbers	
		do
		{
			x1 = (2.0f*rand())/((float)RAND_MAX) - 1.0f;
			x2 = (2.0f*rand())/((float)RAND_MAX) - 1.0f;
			w = x1*x1 + x2*x2;
		} while(w >= 1.0);
		w = sqrt( (-2.0f*log(w))/w );
		y1 = x1*w;
		y2 = x2*w;

		// scale to be centered at preferred velocity, able to go to max
		Vector2 a(y1,y2);
		a /= sqrt(a.x()*a.x() + a.y()*a.y());
		vCand= _vPref + a*0.5f*(_maxSpeed-_prefSpeed);

		// scale to max
		vCand *= _maxSpeed*_maxSpeed/(vCand.x()*vCand.x() + vCand.y()*vCand.y());
      }
	  /***************************************************/
	  else
	  {
		do
		{
          vCand = Vector2( 2.0f*rand() - RAND_MAX, 2.0f*rand() - RAND_MAX);
        } while (absSq(vCand) > sqr((float) RAND_MAX));
		vCand *= (_maxSpeed / RAND_MAX);
	  }

      float dV; // distance between candidate velocity and preferred velocity
      if (_collision)
        dV = 0;
	  else
        dV = abs(vCand - _vPref);

	  int currType = AGENT;
      float ct = RVO_INFTY; // time to collision

      // iterate over neighbors
      for(std::multimap<float, std::pair<int, int> >::iterator j = _neighbors.begin(); j != _neighbors.end(); ++j)
	  {
        float ct_j; // time to collision with agent j
        Vector2 Vab;
        int type = j->second.first;
        int id = j->second.second;

        if (type == AGENT)
		{
		  /*** MWS - if this is a meta-agent, then ignore ******/
		  Agent* other = _sim->_agents[id];
		  if(_class != other->_class && false == (1 == _class && 3 == other->_class))
			  continue;
		  else if(1 == _class && _goalID == other->_goalID)
			  continue;
		  /*****************************************************/

		  /*** MWS - generalize to take alpha's into account ***/
		  const float myAlpha = _alpha/(_alpha + other->_alpha);
		  Vab = vCand/myAlpha + _v*(1.0f - 1.0f/myAlpha) - other->_v;
		  /*****************************************************/

          float time = timeToCollision(_p, Vab, other->_p, _r + other->_r, _collision);
          if (_collision)
		  {
            ct_j = -std::ceil(time / _sim->_timeStep );
            ct_j -= absSq(vCand) / sqr(_maxSpeed);
          }
		  else
		  {
            ct_j = time;
          }
        }
		else if (type == OBSTACLE)
		{
          Obstacle* other;
          other = _sim->_obstacles[id];

		  /*** MWS - added check for class ***/
		  if(0 != _class)
			  continue;
		  /***********************************/

          float time_1, time_2, time_a, time_b;
          time_1 = timeToCollision(_p, vCand, other->_p1, _r, _collision);
          time_2 = timeToCollision(_p, vCand, other->_p2, _r, _collision);
          time_a = timeToCollision(_p, vCand, other->_p1 + _r * other->_normal, other->_p2 + _r * other->_normal, _collision);
          time_b = timeToCollision(_p, vCand, other->_p1 - _r * other->_normal, other->_p2 - _r * other->_normal, _collision);

          if (_collision)
		  {
            float time = std::max(std::max(std::max(time_1, time_2), time_a), time_b);
            ct_j = -std::ceil(time / _sim->_timeStep);
            ct_j -= absSq(vCand) / sqr(_maxSpeed);
          }
		  else
		  {
            float time = std::min(std::min(std::min(time_1, time_2), time_a), time_b);
            if (time < _sim->_timeStep || sqr(time) < absSq(vCand) / sqr(_maxAccel))
              ct_j = time;
			else
              ct_j = RVO_INFTY; // no penalty
          }
        }

        if (ct_j < ct)
		{
           ct = ct_j;

		   // MWS --- save type of obst as well
           currType = type;
		   
           // pruning search if no better penalty can be obtained anymore for this velocity
           if ( _safetyFactor / ct + dV >= min_penalty)
              break;
		}
      }

	  // MWS --- always use stored factor for obstacles
	  float penalty = (AGENT == currType ? _safetyFactor : _storedSafetyFactor) / ct + dV;
      if (penalty < min_penalty)
	  {
         min_penalty = penalty;
         _vNew = vCand;
      }
    }
  }

  //---------------------------------------------------------------
  void Agent::insertObstacleNeighbor(int id, float& rangeSq) {

    /*** MWS - added class check ***/
    if(0 != _class && 3 != _class)
		return;
    /*******************************/

    Obstacle* obstacle = _sim->_obstacles[id];
    float distSq = distSqPointLineSegment(obstacle->_p1, obstacle->_p2, _p);
    if (distSq < sqr(_r) && distSq < rangeSq) 
	{ // COLLISION!
      if (!_collision)
	  {
		/*** MWS - added class check *********/
		if(3 != _class)
		{
			_collision = true;
			_neighbors.clear();
			rangeSq = sqr(_r);
		}
		else
			_breakForm = true;
		/*************************************/
      }

      if (_neighbors.size() == _maxNeighbors) {
        _neighbors.erase(--_neighbors.end());
      }
      _neighbors.insert(std::make_pair(distSq, std::make_pair(OBSTACLE, id)));
      if (_neighbors.size() == _maxNeighbors) {
        rangeSq = (--_neighbors.end())->first;
      }
    } else if (!_collision && distSq < rangeSq) {
      if (_neighbors.size() == _maxNeighbors) {
        _neighbors.erase(--_neighbors.end());
      }
      _neighbors.insert(std::make_pair(distSq, std::make_pair(OBSTACLE, id)));
      if (_neighbors.size() == _maxNeighbors) {
        rangeSq = (--_neighbors.end())->first;
      }
    }
  }

  void Agent::insertAgentNeighbor(int id, float& rangeSq)
  {
	Agent* other = _sim->_agents[id];

	/*** MWS - added check to see if active ***/
	if(false == other->_active)
		return;
	/******************************************/

	// MWS -- added class check
    if(this != other && (0 == other->_class || other->_class == _class || (1 == _class && 3 == other->_class)))
	{
		float distSq = absSq(_p - other->_p);
		// MWS -- added class check
		if(distSq < sqr(_r + other->_r) && distSq < rangeSq &&
		   ((0 == _class && 0 == other->_class) || (1 == _class && 1 == other->_class && _goalID != other->_goalID)))
		{ // COLLISION!
			if (!_collision)
			{
				_collision = true;
				_neighbors.clear();
			}

			if (_neighbors.size() == _maxNeighbors)
			{
				_neighbors.erase(--_neighbors.end());
			}
			_neighbors.insert(std::make_pair(distSq, std::make_pair(AGENT, id)));
			if (_neighbors.size() == _maxNeighbors)
			{
				rangeSq = (--_neighbors.end())->first;
			}
		}
		else if (!_collision && distSq < rangeSq)
		{
			if (_neighbors.size() == _maxNeighbors)
			{
				_neighbors.erase(--_neighbors.end());
			}
			_neighbors.insert(std::make_pair(distSq, std::make_pair(AGENT, id)));
			if (_neighbors.size() == _maxNeighbors)
			{
				rangeSq = (--_neighbors.end())->first;
			}
		}
	}
  }

  void Agent::computeNeighbors() {
    // Compute new neighbors of agent;
    // sort them according to distance (optimized effect of pruning heuristic in search for new velocities);
    // seperate between colliding and near agents

    _collision = false;
    _neighbors.clear();

	/*** MWS - added reset for overlap ***/
	const bool oldBreakForm = _breakForm;
	_breakForm = false;
	/*************************************/

    // check obstacle neighbors
    float rangeSq = std::min(sqr(_neighborDist), sqr(std::max(_sim->_timeStep, _maxSpeed / _maxAccel)*_maxSpeed + _r));
    _sim->_kdTree->computeObstacleNeighbors(this, rangeSq);

    if(_collision)
		return;

	/*** MWS - added check to see if _breakForm has changed ***/
	_reforming = (true == oldBreakForm && false == _breakForm);
	/**********************************************************/

     // Check other agents
    if (_neighbors.size() != _maxNeighbors) {
      rangeSq = sqr(_neighborDist);
    }
    _sim->_kdTree->computeAgentNeighbors(this, rangeSq);
  }

  //---------------------------------------------------------------

  // Prepare for next cycle
  void Agent::computePreferredVelocity()
  {
    // compute subgoal
    Goal * goal = _sim->_goals[_goalID];

	/*** MWS - only allow automatic sub-goal change for normal agents ****/
	if(0 == _class)
	{
		if (_subGoal == -1)
		{ // sub_goal is goal
			if (!_sim->_kdTree->queryVisibility(goal->_vertex->_p, _p, 0))
			{
				_subGoal = -2;
			}
		}
		else if (_subGoal >= 0)
		{ // sub_goal is roadmap vertex
			if (_sim->_kdTree->queryVisibility(_sim->_roadmapVertices[_subGoal]->_p, _p, 0))
			{
				int try_advance_sub_goal = goal->_dist[_subGoal].second; // try to advance sub_goal
				if (try_advance_sub_goal == -1)
				{ // advanced sub_goal is goal
					if (_sim->_kdTree->queryVisibility(goal->_vertex->_p, _p, _r))
					{
						_subGoal = -1;
					} 
				}
				else if (_sim->_kdTree->queryVisibility(_sim->_roadmapVertices[try_advance_sub_goal]->_p, _p, _r))
				{ // advanced sub_goal is roadmap vertex
					_subGoal = try_advance_sub_goal; // set new sub_goal if visible
				}
			}
			else
			{ // sub_goal is not visible
				_subGoal = -2;
			}
		}

		if (_subGoal == -2)
		{ // sub_goal is not visible, search all vertices
			if (_sim->_kdTree->queryVisibility(goal->_vertex->_p, _p, _r))
			{
				_subGoal = -1;
			}
			else
			{
				float mindist = RVO_INFTY;
				for (int i = 0; i < (int) goal->_dist.size(); ++i)
				{
					float distance = goal->_dist[i].first + abs(_p - _sim->_roadmapVertices[i]->_p);
					if(distance < mindist && _sim->_kdTree->queryVisibility(_sim->_roadmapVertices[i]->_p, _p, _r))
					{
						mindist = distance;
						_subGoal = i;
					}
				}
			}      
		}
	}
	/*********************************************************************/

    if (_subGoal == -2)
	{ // no vertex is visible, move in direction of goal
      _subGoal = -1;
    }

    // Set preferred velocity
    Vector2 sub_g;
    if (_subGoal == -1)
      sub_g = goal->_vertex->_p;
	else
      sub_g = _sim->_roadmapVertices[_subGoal]->_p;

    float distSq2subgoal = absSq(sub_g - _p);
    
    // compute preferred velocity
    if (_subGoal == -1 && sqr(_prefSpeed * _sim->_timeStep) > distSq2subgoal)
      _vPref = (sub_g - _p) / _sim->_timeStep;
    else
      _vPref = _prefSpeed * (sub_g - _p) / std::sqrt(distSq2subgoal);

	/*** MWS - save another copy for orientation calls ***/
	_vPrefOrig = _vPref;
	/*****************************************************/
  }

  //---------------------------------------------------------------

  // update velocity and position of agent
  void Agent::update()
  {
	/*** MWS - added decrement of life counter ***/
	if(1 == _class)
	{
		if(_updatesToLive > -1)
			--_updatesToLive;
		else
			_groupLeadership = false;
	}
	/*********************************************/

    // Scale proposed new velocity to obey maximum acceleration
    float dv = abs(_vNew - _v);
    if (dv < _maxAccel * _sim->_timeStep) {
      _v = _vNew;
    } else {
      _v = (1 - (_maxAccel * _sim->_timeStep / dv)) * _v + (_maxAccel * _sim->_timeStep / dv) * _vNew;
    }

    // Update position
    _p += _v * _sim->_timeStep;

	/*** MWS - check for sub-goal completion ****/
	if(1 == _class || 3 == _class)
	{
		// if this is a real goal
		if(_subGoal < 0)
		{
			// if within the goal rad mark as done
			if(absSq(_sim->_goals[_goalID]->_vertex->_p - _p) < sqr(_gR))
				_groupLeadership = false;
		}
		else
		{
			if(absSq(_sim->_roadmapVertices[_subGoal]->_p - _p) < sqr(_gR))
				_groupLeadership = false;
		}
	}
	/********************************************/

    // Set reached goal
    if (absSq(_sim->_goals[_goalID]->_vertex->_p - _p) < sqr(_gR))
      _atGoal = true;
	else
	{
      _atGoal = false;
      _sim->_allAtGoals = false;
    }

    // Update orientation
    if (!_atGoal)
      _o = atan(_vPrefOrig);  //MWS - changed to saved version
  }


	float Agent::intersection(const Vector2 &p1, const Vector2 &p2, const Vector2 &p3, const Vector2 &p4) const
	{
		const float div = (p4.y()-p3.y())*(p2.x()-p1.x()) - (p4.x()-p3.x())*(p2.y()-p1.y());
		if(0 == div)
			return -50;
		else
			return ((p4.x()-p3.x())*(p1.y()-p3.y())-(p4.y()-p3.y())*(p1.x()-p3.x()))/div;
	}


	bool Agent::sameSubGoal(const Agent &a) const
	{
		// get the coordinates of the two goals
		Vector2 p1 = (_subGoal < 0) ? _sim->_goals[_goalID]->_vertex->_p : _sim->_roadmapVertices[_subGoal]->_p;
		Vector2 p2 = (a._subGoal < 0) ? _sim->_goals[a._goalID]->_vertex->_p : _sim->_roadmapVertices[a._subGoal]->_p;

		return (absSq(p1 - p2) < sqr(0.5f));
	}


	bool Agent::sameGoal(const Agent &a) const
	{
		Vector2 p1 = _sim->_goals[_goalID]->_vertex->_p;
		Vector2 p2 = _sim->_goals[a._goalID]->_vertex->_p;

		return (absSq(p1 - p2) < sqr(0.5f));
	}


	bool Agent::samePrefV(const Agent &a) const
	{
		return (fabs(_vPref*a._vPref)/(abs(_vPref)*abs(a._vPref)+0.00001f) > 0.2f);
	}

}    // RVO namespace
