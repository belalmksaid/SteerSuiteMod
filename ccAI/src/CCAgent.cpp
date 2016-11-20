//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#include "SteerLib.h"
#include "CCAgent.h"
#include "CCAIModule.h"

#include <limits> 

using namespace Util;
using namespace SteerLib;

/// @file CCAgent.cpp
/// @brief Implements the CCAgent class.

#define MAX_FORCE_MAGNITUDE 3.0f
#define AGENT_MASS 1.0f


CCAgent::CCAgent()
{
	_enabled = false;
}

CCAgent::~CCAgent()
{
	if (_enabled)
	{
		Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius,
				0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
		gSpatialDatabase->removeObject( this, bounds);
	}
}

SteerLib::EngineInterface * CCAgent::getSimulationEngine()
{
	return gEngine;
}

void CCAgent::draw()
{
#ifdef ENABLE_GUI
    CCGlobals::gPhaseProfilers->drawProfiler.start();

	// if the agent is selected, do some annotations just for demonstration
	if (gEngine->isAgentSelected(this)) {
		Util::Ray ray;
		ray.initWithUnitInterval(_position, _forward);
		float t = 0.0f;
		SteerLib::SpatialDatabaseItem * objectFound;
		Util::DrawLib::drawLine(ray.pos, ray.eval(1.0f));
		if (gSpatialDatabase->trace(ray, t, objectFound, this, false)) {
			Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gBlue);
		}
		else {
			Util::DrawLib::drawAgentDisc(_position, _forward, _radius);
		}
	}
	else {

		Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gGray40);
	}
	if (_goalQueue.front().goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
		Util::DrawLib::drawFlag(_goalQueue.front().targetLocation);
	}

	CCGlobals::gPhaseProfilers->drawProfiler.stop();
#endif
}


void CCAgent::reset(const SteerLib::AgentInitialConditions & initialConditions,
									  SteerLib::EngineInterface * engineInfo)
{
	// compute the "old" bounding box of the agent before it is reset.
	// its OK that it will be invalid if the agent was previously disabled
	// because the value is not used in that case.
	Util::AxisAlignedBox oldBounds(_position.x-_radius, _position.x+_radius,
			0.0f, 0.0f, _position.z-_radius, _position.z+_radius);

	// initialize the agent based on the initial conditions
	_position = initialConditions.position;
	_forward = initialConditions.direction;
	_radius = initialConditions.radius;
	// std::cout << "ccAI initial radius is: " << _radius << "\n";

	_velocity = initialConditions.speed * Util::normalize(initialConditions.direction);

	// _name = initialConditions.name;

	// compute the "new" bounding box of the agent
	Util::AxisAlignedBox newBounds(_position.x-_radius, _position.x+_radius,
			0.0f, 0.0f, _position.z-_radius, _position.z+_radius);

	if (!_enabled) {
		// if the agent was not enabled, then it does not already exist in
		// the database, so add it.
		gSpatialDatabase->addObject( this, newBounds);
	} else {
		// if the agent was enabled, then the agent already existed in the database,
		// so update it instead of adding it.
		gSpatialDatabase->updateObject( this, oldBounds, newBounds);
	}

	_enabled = true;

	if (initialConditions.goals.size() == 0) {
		throw Util::GenericException("No goals were specified!\n");
	}

	// iterate over the sequence of goals specified by the initial conditions.
	for (unsigned int i=0; i<initialConditions.goals.size(); i++) {
		if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET ||
		    initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_GROUP_TARGET) {
			_goalQueue.push(initialConditions.goals[i]);
			if (initialConditions.goals[i].targetIsRandom) {
				// if the goal is random, we must randomly generate the goal.
				SteerLib::AgentGoalInfo _goal;
				_goal.targetLocation = gSpatialDatabase->randomPositionWithoutCollisions(1.0f, true);
				_goalQueue.push(_goal);
			}
		} else {
			throw Util::GenericException("Unsupported goal type; CCAgent only "
					"supports GOAL_TYPE_GROUP_TARGET and GOAL_TYPE_SEEK_STATIC_TARGET.");
		}
	}

	assert(_forward.length()!=0.0f);
	assert(_goalQueue.size() != 0);
	assert(_radius != 0.0f);
}


void CCAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
  assert(_enabled);
  AutomaticFunctionProfiler profileThisFunction( &CCGlobals::gPhaseProfilers->aiProfiler );
	if (! _enabled) return;
	// for this function, we assume that all goals are of type:
        //   GOAL_TYPE_GROUP_TARGET and
	//   GOAL_TYPE_SEEK_STATIC_TARGET.
	// the error check for this was performed in reset().

  if (_goalQueue.front().goalType == SteerLib::GOAL_TYPE_GROUP_TARGET)
  {
		const Util::AxisAlignedBox& target_region = _goalQueue.front().targetRegion;
  	float penetration = Util::computeBoxCirclePenetration2D(target_region.xmin, target_region.xmax,
											  target_region.zmin, target_region.zmax,
											  _position, _radius);
  	if (penetration > 0.0f)
  	{
  		_goalQueue.pop();
	}
  }
  else
  {
  	Util::Vector vectorToGoal = _goalQueue.front().targetLocation - _position;
  	if (vectorToGoal.lengthSquared() < (_radius * _radius + GOAL_REACHED_DISTANCE))
  	{
  		_goalQueue.pop();
  	}
	}

	if (_goalQueue.size() == 0) {
  	// in this case, there are no more goals, so disable the agent and remove it from the spatial database.
	  Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
	  gSpatialDatabase->removeObject( this, bounds);
 		_enabled = false;
  	return;
 	}

	// After the module does all the computation for Crowd Continuum forces the planned
	// velocity vector is updated and used for the euler step.
	// The euler integration step will clamp this vector to a reasonable value, if needed.
	// also, the Euler step updates the agent's position in the spatial database.
	_doEulerStep(_planned_velocity, dt);
}


void CCAgent::_doEulerStep(const Util::Vector & steeringDecisionForce, float dt)
{
	// compute acceleration, _velocity, and newPosition by a CC Euler step
	const Util::Vector clippedForce = Util::clamp(steeringDecisionForce, MAX_FORCE_MAGNITUDE);
	// Acceleration is turned off and _velocity is set by the CCAIModule.
  //	Util::Vector acceleration = (clippedForce / AGENT_MASS);
  //	_velocity = _velocity + (dt*acceleration);
  //	_velocity = clamp(_velocity, MAX_SPEED);  // clamp _velocity to the max speed
	_velocity = clamp(clippedForce, MAX_SPEED);  // clamp _velocity to the max speed
	const Util::Point newPosition = _position + (dt*_velocity);

	// For this CC agent, we just make the orientation point along the agent's current velocity.
	if (_velocity.lengthSquared() != 0.0f) {
		_forward = normalize(_velocity);
	}

	// update the database with the new agent's setup
	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	Util::AxisAlignedBox newBounds(newPosition.x - _radius, newPosition.x + _radius, 0.0f, 0.0f, newPosition.z - _radius, newPosition.z + _radius);
	gSpatialDatabase->updateObject( this, oldBounds, newBounds);

	_position = newPosition;
}


void CCAgent::disable()
{
	assert(_enabled == true);

	Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius,
			0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
	gSpatialDatabase->removeObject( this, bounds);

	_enabled = false;
}
