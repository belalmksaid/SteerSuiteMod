//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//




/*
 *
 * Some ideas
 * It is possible to instantiate multiple steering algorithms with this now. What if two were used at the same time and the HybridAI
 * used a linear combination of the update steps from each.
 */

#include "SteerLib.h"
#include "HybridAgent.h"
#include "HybridAIModule.h"
#include "PPRAIModule.h"

/// @file HybridAgent.cpp
/// @brief Implements the HybridAgent class.

#define MAX_FORCE_MAGNITUDE 3.0f
#define MAX_SPEED 1.3f
#define AGENT_MASS 1.0f

HybridAgent::HybridAgent()
{
	_enabled = false;
	_switchLength = (rand() % 20) + 20;
	_agentState = PPR;
	_id = gEngine->getAgents().size();

}

HybridAgent::~HybridAgent()
{
	if (_enabled) {
		// Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
		// gSpatialDatabase->removeObject( this, bounds);
	}
}

SteerLib::EngineInterface * HybridAgent::getSimulationEngine()
{
	return gEngine;
}

void HybridAgent::disable()
{
	// DO nothing for now
	if ( _agentInterface->enabled())
	{
		_agentInterface->disable();
		// TODO Still need to delete this agent acording to its type.

	}
	if ( enabled() )
	{
		// Util::AxisAlignedBox bounds(position().x-_radius, position().x+_radius, 0.0f, 0.0f, position().z-_radius, position().z+_radius);
		// gSpatialDatabase->removeObject( this, bounds);
		_enabled = false;
	}

}

void HybridAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	// compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
	// because the value is not used in that case.
	Util::AxisAlignedBox oldBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);

	// std::cout << "PPR state " << PPR << std::endl;
	// std::cout << "ORCA state " << ORCA << std::endl;


	// initialize the agent based on the initial conditions
	_position = initialConditions.position;
	_forward = initialConditions.direction;
	_radius = initialConditions.radius;
	_velocity = initialConditions.speed * Util::normalize(initialConditions.direction);

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
			throw Util::GenericException("Unsupported goal type; HybridAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET.");
		}
	}

	_currentGoal = _goalQueue.front();
	// std::cout << "There are " << _goalQueue.size() << " goals for this agent" << std::endl;
	HybridAIModule * _agentModule = dynamic_cast<HybridAIModule *>(engineInfo->getModule("hybridAI"));

	if ( _currentGoal.targetBehaviour.getSteeringAlg() == "pprAI")
	{
		// std::cout << "switching to PPR" << std::endl;
		_agentInterface = _agentModule->_pprModule->createAgent();
		_agentInterface->reset( initialConditions, engineInfo);
		// std::cout << "The Behaviour for the current goal has " << _agentInterface->currentGoal().targetBehaviour.getParameters().size()
			//	<< " params for this agent" << std::endl;
		dynamic_cast<PPRAgent *>(_agentInterface)->setParameters(_agentInterface->currentGoal().targetBehaviour);
		_agentState == PPR;
	}
	else if ( _currentGoal.targetBehaviour.getSteeringAlg() == "footstep" )
	{
		// std::cout << "switching to PPR" << std::endl;
		_agentInterface = _agentModule->_footstepModule->createAgent();
		_agentInterface->reset( initialConditions, engineInfo);
		// std::cout << "The Behaviour for the current goal has " << _agentInterface->currentGoal().targetBehaviour.getParameters().size()
			//	<< " params for this agent" << std::endl;
		// dynamic_cast<PPRAgent *>(_agentInterface)->setParameters(_agentInterface->currentGoal().targetBehaviour);
		_agentState == FOOTSTEP;
	}
	/*
	else if ( _currentGoal.targetBehaviour.getSteeringAlg() == "cc" )
	{
		// std::cout << "switching to PPR" << std::endl;
		_agentInterface = _agentModule->_ccModule->createAgent();
		_agentInterface->reset( initialConditions, engineInfo);
		// std::cout << "The Behaviour for the current goal has " << _agentInterface->currentGoal().targetBehaviour.getParameters().size()
			//	<< " params for this agent" << std::endl;
		// dynamic_cast<PPRAgent *>(_agentInterface)->setParameters(_agentInterface->currentGoal().targetBehaviour);
		_agentState == CC;
	}*/
	else
	{
		// std::cout << "switching to ORCA" << std::endl;
		_agentInterface = _agentModule->_rvo2dModule->createAgent();
		_agentInterface->reset( initialConditions, engineInfo);
		dynamic_cast<RVO2DAgent *>(_agentInterface)->setParameters(_agentInterface->currentGoal().targetBehaviour);
		_agentState == ORCA;
	}
	_enabled = true;
	// if ( initialConditions.goals.at(0).targetBehaviour != NULL)
	/*
	{
		Behaviour behave = initialConditions.goals.at(0).targetBehaviour;
		int q;
		for (q=0; q < behave.getParameters().size(); q++ )
		{
			std::cout << " Parameter" << std::endl;
			std::cout << "\tkey: " << behave.getParameters().at(q).key
					<< " value " << behave.getParameters().at(q).value << std::endl;
		}
	}
	*/
	// std::cout << "agent " << _agentInterface->id() << " at mem loc " << *_agentInterface << std::endl;
}


void HybridAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
	// for this function, we assume that all goals are of type GOAL_TYPE_SEEK_STATIC_TARGET.
	// the error check for this was performed in reset().
	if ( !enabled() )
	{
		return;
	}


	/************************Do not seperate these*************************/
	_agentInterface->updateAI(timeStamp, dt, frameNumber);
	if ( !_agentInterface->enabled()  )
	{
		std::cout << "The owned agent is already disabled " << std::endl;
		// this->_enabled=false;
		// return;
	}
	this->_position = _agentInterface->position();
	this->_forward = _agentInterface->forward();
	this->_velocity = _agentInterface->velocity();
	/********************************************************************/

	// std::cout << (_currentGoal.targetLocation - position()).length() << " left till goal" << std::endl;

	// TODO Maybe I should change this logic to depend more on the owned agent...
	if ((_currentGoal.targetLocation - position()).lengthSquared() < _radius * _radius)
	// if ( !this->_agentInterface->enabled() )
	{
		_goalQueue.pop();
		if (_goalQueue.size() != 0)
		{
			_currentGoal = _goalQueue.front();
			std::cout << "steering alg for next state " << _currentGoal.targetBehaviour.getSteeringAlg() <<
					" current state is " << _agentState << std::endl;
			SteerLib::AgentInitialConditions ai = getAgentConditions( this );
			_switchSteeringMethod(ai);
			// if ( ( _agentState == PPR ) && _currentGoal.targetBehaviour.getSteeringAlg() == "rvo2dAI" )

		}
		else {
			// in this case, there are no more goals, so disable the agent and remove it from the spatial database.
			std::cout << "No more goals, I am sad." << std::endl;
			disable();
			return;

		}
	}

}

void HybridAgent::_switchSteeringMethod(SteerLib::AgentInitialConditions ai)
{
	// Delete old agent
	// TODO this should depend on the agent so it is deleted properly
	// But at least PPR does not like being deleted
	// if ( this->_agentState == PPR )
	{
		if ( _agentInterface->enabled() )
		{
			_agentInterface->disable();
		}
		// _agentModule->_pprModule->destroyAgent(_agentInterface);
	}

	// Create new agent
	HybridAIModule * _agentModule = dynamic_cast<HybridAIModule *>(gEngine->getModule("hybridAI"));
	if  ( _currentGoal.targetBehaviour.getSteeringAlg() == "pprAI")
	{
		_agentInterface = _agentModule->_pprModule->createAgent();
		_agentInterface->reset( ai, gEngine);
		dynamic_cast<PPRAgent *>(_agentInterface)->setParameters(_agentInterface->currentGoal().targetBehaviour);
		_agentState=PPR;
		std::cout << "done switching to PPR" << std::endl;
	}
	else if  ( _currentGoal.targetBehaviour.getSteeringAlg() == "rvo2dAI")
	{
		_agentInterface = _agentModule->_rvo2dModule->createAgent();
		_agentInterface->reset( ai, gEngine);
		dynamic_cast<RVO2DAgent *>(_agentInterface)->setParameters(_agentInterface->currentGoal().targetBehaviour);
		_agentState=ORCA;
		std::cout << "done switching to ORCA" << std::endl;
	}
	else if  ( _currentGoal.targetBehaviour.getSteeringAlg() == "footstep")
	{
		_agentInterface = _agentModule->_footstepModule->createAgent();
		_agentInterface->reset( ai, gEngine);
		// dynamic_cast<RVO2DAgent *>(_agentInterface)->setParameters(_agentInterface->currentGoal().targetBehaviour);
		_agentState=FOOTSTEP;
		std::cout << "done switching to Footstep" << std::endl;
	}
	else
	{
		std::cout << "I am so lost." << std::endl;
	}


}


void HybridAgent::draw()
{
#ifdef ENABLE_GUI

	if (gEngine->isAgentSelected(this))
	{
		Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gGreen);
	}
	else
	{
		_agentInterface->draw();
	}
	// if the agent is selected, do some annotations just for demonstration
	/*
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
	*/
#endif
}

