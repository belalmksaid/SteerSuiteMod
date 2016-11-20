//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//




#include "HIDACAgent.h"
#include "HIDACAIModule.h"
#include "SteerLib.h"
#include "Definitions.h"
#include "HIDAC_Parameters.h"
// #include <math.h>

// #include "util/Geometry.h"

/// @file HIDACAgent.cpp
/// @brief Implements the HIDACAgent class.

#undef min
#undef max

#define AGENT_MASS 1.0f

using namespace Util;
using namespace HIDACGlobals;
using namespace SteerLib;

// #define _DEBUG_ENTROPY 1

HIDACAgent::HIDACAgent()
{
	_HIDACParams.hidac_acceleration = hidac_acceleration;
	_HIDACParams.hidac_personal_space_threshold = hidac_personal_space_threshold;
	_HIDACParams.hidac_agent_repulsion_importance = hidac_agent_repulsion_importance;
	_HIDACParams.hidac_query_radius = hidac_query_radius;
	_HIDACParams.hidac_body_force = hidac_body_force;
	_HIDACParams.hidac_agent_body_force = hidac_agent_body_force;
	_HIDACParams.hidac_sliding_friction_force = hidac_sliding_friction_force;
	_HIDACParams.hidac_agent_b = hidac_agent_b;
	_HIDACParams.hidac_agent_a = hidac_agent_a;
	_HIDACParams.hidac_wall_b = hidac_wall_b;
	_HIDACParams.hidac_wall_a = hidac_wall_a;
	_HIDACParams.hidac_max_speed = hidac_max_speed;
	// std::cout << "HiDAC agent max speed " << _HIDACParams.hidac_max_speed << std::endl;
	_enabled = false;
}

HIDACAgent::~HIDACAgent()
{
	// std::cout << this << " is being deleted" << std::endl;
	/*
	if (this->enabled())
	{
		Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
		// gSpatialDatabase->removeObject( this, bounds);
	}*/
	// std::cout << "Someone is removing an agent " << std::endl;
}

void HIDACAgent::setParameters(Behaviour behave)
{
	this->_HIDACParams.setParameters(behave);
}

SteerLib::EngineInterface * HIDACAgent::getSimulationEngine()
{
	return gEngine;
}

void HIDACAgent::disable()
{
	// DO nothing for now
	// if we tried to disable a second time, most likely we accidentally ignored that it was disabled, and should catch that error.
	// std::cout << "this agent is being disabled " << this << std::endl;
	assert(_enabled==true);


	//  1. remove from database
	AxisAlignedBox b = AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	gSpatialDatabase->removeObject(dynamic_cast<SpatialDatabaseItemPtr>(this), b);

	//  2. set enabled = false
	_enabled = false;


}

void HIDACAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	// compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
	// because the value is not used in that case.
	// std::cout << "resetting agent " << this << std::endl;
	_waypoints.clear();
	agentNeighbors_.clear();
	obstacleNeighbors_.clear();
	orcaPlanes_.clear();
	orcaLines_.clear();

	Util::AxisAlignedBox oldBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.5f, _position.z-_radius, _position.z+_radius);


	// initialize the agent based on the initial conditions
	/*
	position_ = Vector2(initialConditions.position.x, initialConditions.position.z);
	radius_ = initialConditions.radius;
	velocity_ = normalize(Vector2(initialConditions.direction.x, initialConditions.direction.z));
	velocity_ = velocity_ * initialConditions.speed;
*/
	// initialize the agent based on the initial conditions
	_position = initialConditions.position;
	_forward = normalize(initialConditions.direction);
	_radius = initialConditions.radius;
	_velocity = initialConditions.speed * _forward;

	// compute the "new" bounding box of the agent
	Util::AxisAlignedBox newBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.5f, _position.z-_radius, _position.z+_radius);

	if (!_enabled) {
		// if the agent was not enabled, then it does not already exist in the database, so add it.
		// std::cout
		gSpatialDatabase->addObject( dynamic_cast<SpatialDatabaseItemPtr>(this), newBounds);
	}
	else {
		// if the agent was enabled, then the agent already existed in the database, so update it instead of adding it.
		// std::cout << "new position is " << _position << std::endl;
		// std::cout << "new bounds are " << newBounds << std::endl;
		// std::cout << "reset update " << this << std::endl;
		gSpatialDatabase->updateObject( dynamic_cast<SpatialDatabaseItemPtr>(this), oldBounds, newBounds);
		// engineInfo->getSpatialDatabase()->updateObject( this, oldBounds, newBounds);
	}

	_enabled = true;

	if (initialConditions.goals.size() == 0)
	{
		throw Util::GenericException("No goals were specified!\n");
	}

	while (!_goalQueue.empty())
	{
		_goalQueue.pop();
	}

	// iterate over the sequence of goals specified by the initial conditions.
	for (unsigned int i=0; i<initialConditions.goals.size(); i++) {
		if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
			_goalQueue.push(initialConditions.goals[i]);
			if (initialConditions.goals[i].targetIsRandom)
			{
				// if the goal is random, we must randomly generate the goal.
				// std::cout << "assigning random goal" << std::endl;
				SteerLib::AgentGoalInfo _goal;
				_goal.targetLocation = gSpatialDatabase->randomPositionWithoutCollisions(1.0f, true);
				_goalQueue.push(_goal);
			}
		}
		else {
			throw Util::GenericException("Unsupported goal type; HIDACAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET.");
		}
	}

	runLongTermPlanning2();

	// std::cout << "first waypoint: " << _waypoints.front() << " agents position: " << position() << std::endl;
	/*
	 * Must make sure that _waypoints.front() != position(). If they are equal the agent will crash.
	 * And that _waypoints is not empty
	 */
	Util::Vector goalDirection;
	goalDirection = normalize( _goalQueue.front().targetLocation - position());

	_prefVelocity = Util::Vector(goalDirection.x, 0.0f, goalDirection.z) * PERFERED_SPEED;

	_velocity = _prefVelocity * _HIDACParams.hidac_acceleration;
#ifdef _DEBUG_ENTROPY
	std::cout << "goal direction is: " << goalDirection << " prefvelocity is: " << prefVelocity_ <<
			" and current velocity is: " << velocity_ << std::endl;
#endif


	// std::cout << "Parameter spec: " << _HIDACParams << std::endl;
	// gEngine->addAgent(this, rvoModule);
	assert(_forward.length()!=0.0f);
	assert(_goalQueue.size() != 0);
	assert(_radius != 0.0f);
}


void HIDACAgent::calcNextStep(float dt)
{

	float tmp_speed =  velocity().length() + (ACCELERATION * dt);
	if ( tmp_speed > maxSpeed_ )
	{
		tmp_speed = maxSpeed_;
	}

}

std::pair<float, Util::Point> minimum_distance(Util::Point l1, Util::Point l2, Util::Point p)
{
  // Return minimum distance between line segment vw and point p
  float lSq = (l1 - l2).lengthSquared();  // i.e. |l2-l1|^2 -  avoid a sqrt
  if (lSq == 0.0)
	  return std::make_pair((p - l2).length(),l1 );   // l1 == l2 case
  // Consider the line extending the segment, parameterized as l1 + t (l2 - l1).
  // We find projection of point p onto the line.
  // It falls where t = [(p-l1) . (l2-l1)] / |l2-l1|^2
  const float t = dot(p - l1, l2 - l1) / lSq;
  if (t < 0.0)
  {
	  return std::make_pair((p - l1).length(), l1);       // Beyond the 'l1' end of the segment
  }
  else if (t > 1.0)
  {
	  return std::make_pair((p - l2).length(), l2);  // Beyond the 'l2' end of the segment
  }
  const Util::Point projection = l1 + t * (l2 - l1);  // Projection falls on the segment
  return std::make_pair((p - projection).length(), projection) ;
}


Util::Vector HIDACAgent::calcProximityForce(float dt)
{

	Util::Vector agent_repulsion_force = Util::Vector(0,0,0);

	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
		gSpatialDatabase->getItemsInRange(_neighbors,
				_position.x-(this->_radius + _HIDACParams.hidac_query_radius),
				_position.x+(this->_radius + _HIDACParams.hidac_query_radius),
				_position.z-(this->_radius + _HIDACParams.hidac_query_radius),
				_position.z+(this->_radius + _HIDACParams.hidac_query_radius),
				dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	SteerLib::AgentInterface * tmp_agent;
	SteerLib::ObstacleInterface * tmp_ob;
	Util::Vector away = Util::Vector(0,0,0);
	Util::Vector away_obs = Util::Vector(0,0,0);

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin();  neighbour != _neighbors.end();  neighbour++)
	// for (int a =0; a < tmp_agents.size(); a++)
	{
		if ( (*neighbour)->isAgent() )
		{
			tmp_agent = dynamic_cast<SteerLib::AgentInterface *>(*neighbour);

			// direction away from other agent
			Util::Vector away_tmp = normalize(position() - tmp_agent->position());

			// Scale force
			// std::cout << "the exp of agent distance is " << exp((radius() + tmp_agent->radius()) -
				//	(position() - tmp_agent->position()).length()) << std::endl;


			// away = away + (away_tmp * ( radius() / ((position() - tmp_agent->position()).length() * B) ));
			away = away + (away_tmp * ( _HIDACParams.hidac_agent_a * (exp((radius() + tmp_agent->radius()) -
					(position() - tmp_agent->position()).length()) / _HIDACParams.hidac_agent_b) ));
		}
		else
		{
			// It is an obstacle
			tmp_ob = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbour);
			Util::Vector wall_normal = calcWallNormal( tmp_ob );
			std::pair<Util::Point,Util::Point> line = calcWallPointsFromNormal(tmp_ob, wall_normal);
			// Util::Point midpoint = Util::Point((line.first.x+line.second.x)/2, ((line.first.y+line.second.y)/2)+1,
				// 	(line.first.z+line.second.z)/2);
			std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
			// wall distance

			Util::Vector away_obs_tmp = normalize(position() - min_stuff.second);

			// away_obs = away_obs + ( away_obs_tmp * ( radius() / ((position() - min_stuff.second).length() * B ) ) );
			away_obs = away_obs + (away_obs_tmp * ( _HIDACParams.hidac_wall_a * (exp((radius()) -
					(position() - min_stuff.second).length()) / _HIDACParams.hidac_wall_b) ));
		}

	}
	return away + away_obs;
}

Util::Vector HIDACAgent::calcRepulsionForce(float dt)
{
	return calcWallRepulsionForce(dt) + (_HIDACParams.hidac_agent_repulsion_importance * calcAgentRepulsionForce(dt));
}

Util::Vector HIDACAgent::calcAgentRepulsionForce(float dt)
{

	Util::Vector agent_repulsion_force = Util::Vector(0,0,0);

	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
		gSpatialDatabase->getItemsInRange(_neighbors,
				_position.x-(this->_radius + _HIDACParams.hidac_query_radius),
				_position.x+(this->_radius + _HIDACParams.hidac_query_radius),
				_position.z-(this->_radius + _HIDACParams.hidac_query_radius),
				_position.z+(this->_radius + _HIDACParams.hidac_query_radius),
				dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	SteerLib::AgentInterface * tmp_agent;

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin();  neighbour != _neighbors.end();  neighbour++)
	// for (int a =0; a < tmp_agents.size(); a++)
	{
		if ( (*neighbour)->isAgent() )
		{
			tmp_agent = dynamic_cast<SteerLib::AgentInterface *>(*neighbour);
		}
		else
		{
			continue;
		}
		if ( ( id() != tmp_agent->id() ) &&
				(tmp_agent->computePenetration(this->position(), this->radius()) > 0.000001)
			)
		{
		agent_repulsion_force = agent_repulsion_force +
				((
					(
						(
							position() - tmp_agent->position()
						)
						*
						(
							radius() + tmp_agent->radius() +
							PERSONAL_SPACE_THRESHOLD -
							(
								position() - tmp_agent->position()
							).length()
						)
					)
					/
					(
						position() - tmp_agent->position()
					).length()
				) * _HIDACParams.hidac_agent_body_force * dt);
			// normalized tangential force
		/*
			agent_repulsion_force = agent_repulsion_force +
					(
						(
							(-1*position()) - tmp_agent->position()
						)
						/
						(
							(-1*position()) - tmp_agent->position()
						).length()

					)*0.2;
					*/
			agent_repulsion_force = agent_repulsion_force +
			(
				(
					cross(cross(tmp_agent->position() - position() , velocity()),
							tmp_agent->position() - position())
				)
				/
				(
					cross(cross(tmp_agent->position() - position(), velocity()),
									tmp_agent->position() - position())
				).length()

			)* _HIDACParams.hidac_sliding_friction_force * dt;
		}

	}
	return agent_repulsion_force;
}

Util::Vector HIDACAgent::calcWallRepulsionForce(float dt)
{

	Util::Vector wall_repulsion_force = Util::Vector(0,0,0);

	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
		gSpatialDatabase->getItemsInRange(_neighbors,
				_position.x-(this->_radius + _HIDACParams.hidac_query_radius),
				_position.x+(this->_radius + _HIDACParams.hidac_query_radius),
				_position.z-(this->_radius + _HIDACParams.hidac_query_radius),
				_position.z+(this->_radius + _HIDACParams.hidac_query_radius),
				dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	SteerLib::ObstacleInterface * tmp_ob;

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin();  neighbour != _neighbors.end();  neighbour++)
	// for (std::set<SteerLib::ObstacleInterface * >::iterator tmp_o = _neighbors.begin();  tmp_o != _neighbors.end();  tmp_o++)
	{
		if ( !(*neighbour)->isAgent() )
		{
			tmp_ob = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbour);
		}
		else
		{
			continue;
		}
		if (
				tmp_ob->computePenetration(this->position(), this->radius()) > 0.000001
			)
		{
			Util::Vector wall_normal = calcWallNormal( tmp_ob );
			std::pair<Util::Point,Util::Point> line = calcWallPointsFromNormal(tmp_ob, wall_normal);
			// Util::Point midpoint = Util::Point((line.first.x+line.second.x)/2, ((line.first.y+line.second.y)/2)+1,
				// 	(line.first.z+line.second.z)/2);
			std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
			// wall distance
			wall_repulsion_force = wall_repulsion_force +
				((
					(
						(
								wall_normal
						)
						*
						(
							radius() +
							_HIDACParams.hidac_personal_space_threshold -
							(
								min_stuff.first
							)
						)
					)
					/
					min_stuff.first
				)* _HIDACParams.hidac_body_force * dt);
			// tangential force
			// std::cout << "wall tangent " << rightSideInXZPlane(wall_normal) <<
				// 	" dot is " << dot(forward(),  rightSideInXZPlane(wall_normal)) <<
					// std::endl;
			wall_repulsion_force = wall_repulsion_force +
			(
				dot(forward(),  rightSideInXZPlane(wall_normal))
				*
				rightSideInXZPlane(wall_normal)
			)* _HIDACParams.hidac_sliding_friction_force * dt;
		}

	}
	return wall_repulsion_force;
}

std::pair<Util::Point, Util::Point> HIDACAgent::calcWallPointsFromNormal(SteerLib::ObstacleInterface* obs, Util::Vector normal)
{
	Util::AxisAlignedBox box = obs->getBounds();
	if ( normal.z == 1)
	{
		return std::make_pair(Util::Point(box.xmin,0,box.zmax), Util::Point(box.xmax,0,box.zmax));
		// Ended here;
	}
	else if ( normal.z == -1 )
	{
		return std::make_pair(Util::Point(box.xmin,0,box.zmin), Util::Point(box.xmax,0,box.zmin));
	}
	else if ( normal.x == 1)
	{
		return std::make_pair(Util::Point(box.xmax,0,box.zmin), Util::Point(box.xmax,0,box.zmax));
	}
	else // normal.x == -1
	{
		return std::make_pair(Util::Point(box.xmin,0,box.zmin), Util::Point(box.xmin,0,box.zmax));
	}
}

/**
 * Basically What side of the obstacle is the agent on use that as the normal
 * DOES NOT SUPPORT non-axis-aligned boxes
 *
 *
 * 			   \		   /
 * 				\		  /
 * 				 \	 a	 /
 *				  \		/
 * 					 _
 * 			a		| |       a
 * 					 -
 * 				  /     \
 * 				 /   a   \
 * 				/	      \
 * 			   /	       \
 *
 *
 */
Util::Vector HIDACAgent::calcWallNormal(SteerLib::ObstacleInterface* obs)
{
	Util::AxisAlignedBox box = obs->getBounds();
	if ( position().x > box.xmax )
	{
		if ( position().z > box.zmax)
		{
			if ( abs(position().z - box.zmax ) >
				abs( position().x - box.xmax) )
			{
				return Util::Vector(0, 0, 1);
			}
			else
			{
				return Util::Vector(1, 0, 0);
			}

		}
		else if ( position().z < box.zmin )
		{
			if ( abs(position().z - box.zmin ) >
				abs( position().x - box.xmax) )
			{
				return Util::Vector(0, 0, -1);
			}
			else
			{
				return Util::Vector(1, 0, 0);
			}

		}
		else
		{ // in between zmin and zmax
			return Util::Vector(1, 0, 0);
		}

	}
	else if ( position().x < box.xmin )
	{
		if ( position().z > box.zmax )
		{
			if ( abs(position().z - box.zmax ) >
				abs( position().x - box.xmin) )
			{
				return Util::Vector(0, 0, 1);
			}
			else
			{
				return Util::Vector(-1, 0, 0);
			}

		}
		else if ( position().z < box.zmin )
		{
			if ( abs(position().z - box.zmin ) >
				abs( position().x - box.xmin) )
			{
				return Util::Vector(0, 0, -1);
			}
			else
			{
				return Util::Vector(-1, 0, 0);
			}

		}
		else
		{ // in between zmin and zmax
			return Util::Vector(-1, 0, 0);
		}
	}
	else // between xmin and xmax
	{
		if ( position().z > box.zmax )
		{
			return Util::Vector(0, 0, 1);
		}
		else if ( position().z < box.zmin)
		{
			return Util::Vector(0, 0, -1);
		}
		else
		{ // What do we do if the agent is inside the wall?? Lazy Normal
			return calcObsNormal( obs );
		}
	}

}

/**
 * Treats Obstacles as a circle and calculates normal
 */
Util::Vector HIDACAgent::calcObsNormal(SteerLib::ObstacleInterface* obs)
{
	Util::AxisAlignedBox box = obs->getBounds();
	Util::Point obs_centre = Util::Point((box.xmax+box.xmin)/2, (box.ymax+box.ymin)/2,
			(box.zmax+box.zmin)/2);
	return normalize(position() - obs_centre);
}

/*
void HIDACAgent::computeNeighbors()
{
	agentNeighbors_.clear();

	if (_HIDACParams.rvo_max_neighbors > 0) {
		// std::cout << "About to segfault" << std::endl;
		dynamic_cast<HIDACAIModule *>(rvoModule)->kdTree_->computeAgentNeighbors(this, _HIDACParams.rvo_neighbor_distance * _HIDACParams.rvo_neighbor_distance);
		// std::cout << "Made it past segfault" << std::endl;
	}
}*/



void HIDACAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
	// std::cout << "_HIDACParams.rvo_max_speed " << _HIDACParams._HIDACParams.rvo_max_speed << std::endl;
	Util::AutomaticFunctionProfiler profileThisFunction( &HIDACGlobals::gPhaseProfilers->aiProfiler );
	if (!enabled())
	{
		return;
	}

	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);

	SteerLib::AgentGoalInfo goalInfo = _goalQueue.front();
	Util::Vector goalDirection;
	if ( ! _waypoints.empty() )
	{
		/*
		 * Check to make sure there is still a line of sight to the waypoint
		 * If there is not recompute smooth path.
		 */
		float dummyt;
		SpatialDatabaseItemPtr dummyObject;
		Ray lineOfSightRay;

		lineOfSightRay.initWithUnitInterval(position(), _waypoints.front() - position());
		// Ignore agents in trace
		if ( gSpatialDatabase->trace(lineOfSightRay,dummyt, dummyObject,
				dynamic_cast< SpatialDatabaseItemPtr>(dummyObject),true) )
		{
			runLongTermPlanning2();
		}
		if ( ! _waypoints.empty() )
		{
			goalDirection = normalize(_waypoints.front() - position());
		}
		else
		{
			goalDirection = normalize(goalInfo.targetLocation - position());
		}
	}
	else
	{
		goalDirection = normalize(goalInfo.targetLocation - position());
	}
	// _prefVelocity = goalDirection * PERFERED_SPEED;
	Util::Vector prefForce = ((goalDirection * PERFERED_SPEED) - velocity()) * (_HIDACParams.hidac_acceleration * dt); //assumption here
	prefForce = prefForce + velocity();

	Util::Vector repulsionForce = calcRepulsionForce(dt);
	Util::Vector proximityForce = calcProximityForce(dt);

	std::cout << "agent" << id() << " repulsion force " << repulsionForce << std::endl;
	std::cout << "agent" << id() << " proximity force " << proximityForce << std::endl;
	std::cout << "agent" << id() << " pref force " << prefForce << std::endl;

	// _velocity = _newVelocity;
	int alpha=1;
	if ( repulsionForce.length() > 0.0)
	{
		alpha=0;
	}

	// _velocity = (alpha*_prefVelocity) + repulsionForce + proximityForce;
	// _velocity = velocity() + repulsionForce + proximityForce;
	_velocity = prefForce + repulsionForce + proximityForce;

	_velocity = clamp(velocity(), _HIDACParams.hidac_max_speed);
	std::cout << "agent" << id() << " speed is " << velocity().length() << std::endl;
	_position = position() + (velocity() * dt);
	// A grid database update should always be done right after the new position of the agent is calculated
	/*
	 * Or when the agent is removed for example its true location will not reflect its location in the grid database.
	 * Not only that but this error will appear random depending on how well the agent lines up with the grid database
	 * boundaries when removed.
	 */
	Util::AxisAlignedBox newBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	gSpatialDatabase->updateObject( this, oldBounds, newBounds);


	if ( ( !_waypoints.empty() ) && (_waypoints.front() - position()).lengthSquared() < _radius * _radius)
	{
		_waypoints.erase(_waypoints.begin());
	}
	/*
	 * Now do the conversion from HIDACAgent into the SteerSuite coordinates
	 */
	// _velocity.y = 0.0f;

	if ((goalInfo.targetLocation - position()).length() < radius())
	{
		_goalQueue.pop();
		// std::cout << "Made it to a goal" << std::endl;
		if (_goalQueue.size() != 0) {
			// in this case, there are still more goals, so start steering to the next goal.
			goalDirection = _goalQueue.front().targetLocation - _position;
			_prefVelocity = Util::Vector(goalDirection.x, 0.0f, goalDirection.z);
		}
		else {
			// in this case, there are no more goals, so disable the agent and remove it from the spatial database.
			disable();
			/*
			AxisAlignedBox b = AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
			gSpatialDatabase->removeObject(dynamic_cast<SpatialDatabaseItemPtr>(this), b);

			//  2. set enabled = false
			_enabled = false;
			 */
			return;

		}
	}

	// Hear the 2D solution from RVO is converted into the 3D used by SteerSuite
	// _velocity = Vector(velocity().x, 0.0f, velocity().z);
	if ( velocity().length() > 0.0 )
	{
		// Only assign forward direction if agent is moving
		// Otherwise keep last forward
		_forward = normalize(_velocity);
	}
	// _position = _position + (_velocity * dt);

}


bool HIDACAgent::runLongTermPlanning2()
{

#ifndef USE_PLANNING
	return;
#endif
	_waypoints.clear();
	//==========================================================================

	// run the main a-star search here
	std::vector<Util::Point> agentPath;
	Util::Point pos =  position();

	if ( !gEngine->getPathPlanner()->findSmoothPath(pos, _goalQueue.front().targetLocation,
			agentPath, (unsigned int) 50000))
	{
		return false;
	}

	// Push path into _waypoints

	// Skip first node that is at location of agent
	for  (int i=1; i <  agentPath.size(); i++)
	{
		_waypoints.push_back(agentPath.at(i));

	}

	return true;

}


void HIDACAgent::draw()
{
#ifdef ENABLE_GUI
	// if the agent is selected, do some annotations just for demonstration
	if (gEngine->isAgentSelected(this))
	{
		Util::Ray ray;
		ray.initWithUnitInterval(_position, _forward);
		float t = 0.0f;
		SteerLib::SpatialDatabaseItem * objectFound;
		Util::DrawLib::drawLine(ray.pos, ray.eval(1.0f));
		if (gSpatialDatabase->trace(ray, t, objectFound, this, false))
		{
			Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gBlue);
		}
		else {
			Util::DrawLib::drawAgentDisc(_position, _forward, _radius);
		}
		Util::DrawLib::drawFlag( this->currentGoal().targetLocation, Color(0.5f,0.8f,0), 2);
	}
	else {
		Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gBlue);
	}
	if (_goalQueue.front().goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
		Util::DrawLib::drawFlag(_goalQueue.front().targetLocation);
	}

	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	gSpatialDatabase->getItemsInRange(_neighbors, _position.x-(this->_radius * 3), _position.x+(this->_radius * 3),
			_position.z-(this->_radius * 3), _position.z+(this->_radius * 3), dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin();  neighbor != _neighbors.end();  neighbor++)
	{
		if ( (*neighbor)->computePenetration(this->position(), this->_radius) > 0.00001f)
		{
			// Util::DrawLib::drawStar(this->position() + ((dynamic_cast<AgentInterface*>(*neighbor)->position() - this->position())/2), Util::Vector(1,0,0), 1.14f, gRed);
			Util::DrawLib::drawStar(this->position(), Util::Vector(1,0,0), 1.14f, gRed);
		}
	}

#ifdef DRAW_ANNOTATIONS

	for (int i=0; ( _waypoints.size() > 1 ) && (i < (_waypoints.size() - 1)); i++)
	{
		if ( gEngine->isAgentSelected(this) )
		{
			DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gYellow);
		}
		else
		{
			DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gBlack);
		}
	}

	for (int i=0; i < (_waypoints.size()); i++)
	{
		DrawLib::drawStar(_waypoints.at(i), Util::Vector(1,0,0), 0.34f, gBlue);
	}

	/*
	// draw normals and closest points on walls
	std::set<SteerLib::ObstacleInterface * > tmp_obs = gEngine->getObstacles();

	for (std::set<SteerLib::ObstacleInterface * >::iterator tmp_o = tmp_obs.begin();  tmp_o != tmp_obs.end();  tmp_o++)
	{
		Util::Vector normal = calcWallNormal( *tmp_o );
		std::pair<Util::Point,Util::Point> line = calcWallPointsFromNormal(* tmp_o, normal);
		Util::Point midpoint = Util::Point((line.first.x+line.second.x)/2, ((line.first.y+line.second.y)/2)+1,
				(line.first.z+line.second.z)/2);
		DrawLib::drawLine(midpoint, midpoint+normal, gGreen);

		// Draw the closes point as well
		std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
		DrawLib::drawStar(min_stuff.second, Util::Vector(1,0,0), 0.34f, gGreen);
	}
	*/

#endif

#endif
}

