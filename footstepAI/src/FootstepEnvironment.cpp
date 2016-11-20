//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#include "SteerLib.h"
#include "math.h"

#include "footstepAI/FootstepEnvironment.h"
#include "footstepAI/FootstepAgent.h"
#include "footstepAI/FootstepAIModule.h"


//using namespace FootstepGlobals;
using namespace Util;
using namespace SteerLib;


#define ENABLE_RANDOM_CHOICES
#define ENABLE_INPLACE_TURNING


FootstepEnvironment::FootstepEnvironment( FootstepAgent * newAgent )
{ 
	_numNodesExpanded = 0;
	agent = newAgent; 
}


float FootstepEnvironment::getHeuristic(int start, int target) const
{
	const Footstep & currentStep = agent->_cachedFootstepOptions[start];
	const Footstep & targetStep = agent->_cachedFootstepOptions[target];


	float dist = sqrtf( (targetStep.outputCOMState.x-currentStep.outputCOMState.x)*(targetStep.outputCOMState.x-currentStep.outputCOMState.x)
		+ (targetStep.outputCOMState.z-currentStep.outputCOMState.z)*(targetStep.outputCOMState.z-currentStep.outputCOMState.z));

	float estimatedNumSteps = dist / agent->_walkParameters.preferredStepLength;

	float energyCost = estimatedNumSteps * agent->_walkParameters.heuristicEnergyCostPerStep;

	//float distToGoal = (agent->currentGoal().targetLocation - Util::Point(targetStep.outputCOMState.x, 0, targetStep.outputCOMState.z)).length();

	//energyCost = distToGoal;

	//std::cerr << "=========================\n";
	//std::cerr << "CURRENT STEP: \n" << currentStep;
	//std::cerr << " - ENERGY HEURISTIC from node " << start << " to goal is " << energyCost << "\n";

	//return dist;
	return energyCost;
}


inline bool FootstepEnvironment::_addFootstepIfValid(float stepDuration, float parabolaOrientationPhi, bool phiIsIdeal, float desiredVelocity, const Footstep & previousStep, FootStateEnum nextState, vector<Successor> & result) const
	//const Footstep & previousStep, Footstep & newStep, vector<Successor> & result) const
{
	Footstep newStep;
	
	// Backstepping hack:
	/*if (agent->_createFootstepAction(stepDuration, parabolaOrientationPhi, phiIsIdeal, -desiredVelocity, previousStep, newStep, nextState) == true) {
		//std::cerr << "CREATED FOOTSTEP # " << agent->_cachedFootstepOptions.size() << "\n";
		int successorID = agent->_cachedFootstepOptions.size();
		agent->_cachedFootstepOptions.push_back(newStep);
		Successor footstepOption(successorID, newStep.energyCost);
		//float dist = sqrtf(  (newStep.outputCOMState.x - previousStep.outputCOMState.x)*(newStep.outputCOMState.x - previousStep.outputCOMState.x) + (newStep.outputCOMState.z - previousStep.outputCOMState.z)*(newStep.outputCOMState.z - previousStep.outputCOMState.z)   );
		//Successor footstepOption(successorID, dist);
		result.push_back(footstepOption);
		//return true;
	}*/

	if (agent->_createFootstepAction(stepDuration, parabolaOrientationPhi, phiIsIdeal, desiredVelocity, previousStep, newStep, nextState) == true) {
		//std::cerr << "CREATED FOOTSTEP # " << agent->_cachedFootstepOptions.size() << "\n";
		int successorID = agent->_cachedFootstepOptions.size();
		agent->_cachedFootstepOptions.push_back(newStep);
		Successor footstepOption(successorID, newStep.energyCost);
		//float dist = sqrtf(  (newStep.outputCOMState.x - previousStep.outputCOMState.x)*(newStep.outputCOMState.x - previousStep.outputCOMState.x) + (newStep.outputCOMState.z - previousStep.outputCOMState.z)*(newStep.outputCOMState.z - previousStep.outputCOMState.z)   );
		//Successor footstepOption(successorID, dist);
		result.push_back(footstepOption);
		return true;
	}
	else {
		return false;
	}
}


void FootstepEnvironment::getSuccessors(int nodeId, int lastNodeId, vector<Successor> & result) const
{
	result.clear();

	// seems like the planner usually does not expand more than a few thousand nodes in bad cases
	// (the worst observed so far was about 3000 nodes expanded, 5000 nodes generated)
	// so, we can cut-off at 10,000 expanded nodes without worrying too much that we are loosing something important.
	// this prevents a pathological pedestrian from stalling the system.
	_numNodesExpanded++;

	// On average only 70 nodes are expanded so I really dropped the bound to help speed through bad scenarios.
	if (_numNodesExpanded > _maxNumNodesToExpand)
	{
		// std::cout << "Footsteps expanded over " << _numNodesExpanded << " nodes before giving up." << std::endl;
		return;
	}

	// NOTE CAREFULLY using an alias didn't work, because the STL vector probably 
	// gets re-allocated looses the reference.
	const Footstep previousStep = agent->_cachedFootstepOptions[nodeId];


	//std::cerr << "EXPANDING NODE " << nodeId << ":\n";
	//std::cerr << "     cost of that node was: " << previousStep.energyCost << "\n";

	if (previousStep.isAGoalState)
	{
		Successor finalState(1, 0.0f);
		result.push_back(finalState);
#ifdef _DEBUG_1
		std::cout << "Footsteps expanded over " << _numNodesExpanded << " nodes to reach goal." << std::endl;
#endif
		return;
	}

	/*
	bool ignoreCollisions = false;
	if (nodeId == 0) {
		// don't pay attention to collisions on the first footstep of the
		// plan, just in case we STARTED the plan in a colliding situation.
		ignoreCollisions = true; 
	}
*/
	float radiansInterval = (previousStep.whichFoot == LEFT_FOOT) ? (-M_PI_OVER_2) : (M_PI_OVER_2);
	float parabolaPhiStraight = atan2f(previousStep.outputCOMState.dz, previousStep.outputCOMState.dx);
	Vector dirToTarget = normalize(Vector(agent->_cachedFootstepOptions[1].outputCOMState.x - previousStep.outputCOMState.x, 0.0f, agent->_cachedFootstepOptions[1].outputCOMState.z - previousStep.outputCOMState.z));
	Vector initialDir = normalize(Vector(previousStep.outputCOMState.dx, 0.0f, previousStep.outputCOMState.dz));
	Vector initialRightSide = Vector( initialDir.z, initialDir.y, -initialDir.x );
	float frontDotProduct = dot(dirToTarget, initialDir);
	float rightDotProduct = dot( dirToTarget, initialRightSide);
	Vector idealDir = initialDir + dirToTarget;
	float idealParabolaPhi = atan2f( idealDir.z, idealDir.x ) + 0.025f*radiansInterval;
	//float idealParabolaPhi = atan2f( idealDir.z, idealDir.x ) - 0.025f*radiansInterval;

	//_addFootstepIfValid(0.6f, parabolaPhiStraight, false, 0.8f, previousStep, FOOTSTEP_STATE_BACKSTEP, result);

	if ((previousStep.state == FOOTSTEP_STATE_NORMAL) || (previousStep.state == FOOTSTEP_STATE_STARTING))
	{

		// valid state transitions are:
		// NORMAL --> NORMAL      (whichFoot alternates)	
		// NORMAL --> STOPPING      (whichFoot alternates)

		// options to generate:
		// 1. (NORMAL) precision towards goal
		// 2. (NORMAL) precision slightly displaced away from goal
		// 3. (NORMAL) large turns
		// 4. (STOPPING) single stop option that steps forward.


		// COMPUTE GOAL-DEPENDENT CHOICES.
		// 
		// goal-dependent choices depend on where the goal is w.r.t. the character.
		// there are 4 main choices:
		//   - goal oriented to outer side of foot, in front
		//   - goal oriented to outer side of foot, NOT in front (a slightly larger region than simply "from behind")
		//   - goal oriented to inner side of foot, in front
		//   - goal oriented to inner side of foot, NOT in front.

		if (previousStep.simulationA < 0.9f) { // this is uninitiallized sometimes. specifically simulationA

			if ( ((rightDotProduct < 0.0f) && (previousStep.whichFoot == RIGHT_FOOT)) || ((rightDotProduct > 0.0f) && (previousStep.whichFoot == LEFT_FOOT)) ) {
				if (  frontDotProduct > 0.2f) {
					// in this case goal is oriented to inner side of foot, in front
					_addFootstepIfValid(0.6f, idealParabolaPhi, true, 1.1f, previousStep, FOOTSTEP_STATE_NORMAL, result);
				}
				else {
					// in this case goal is oriented to inner side of foot, NOT in front
					_addFootstepIfValid(0.6f, idealParabolaPhi, true, 1.1f, previousStep, FOOTSTEP_STATE_NORMAL, result);
				}
			}
			else {
				if (  frontDotProduct > 0.2f) {
					// in this case goal is oriented to outer side of foot, in front
					// the primary reasonable choice in this case is to simply walk straight.
					_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.05f*radiansInterval, false, 1.0f, previousStep, FOOTSTEP_STATE_NORMAL, result);
				}
				else {
					// in this case goal is oriented to outer side of foot, NOT in front.
					// the character's "ideal" choice would be to actually "stop" and accelerate in the correct direction on the next step.
					_addFootstepIfValid(0.5f, parabolaPhiStraight + 0.05f*radiansInterval, false, 0.6f, previousStep, FOOTSTEP_STATE_NORMAL, result);
				}
			}


			// ALSO ALLOW SOME GENERIC TURNING CHOICES AS WELL...

			/*
			_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.05f*radiansInterval, 1.3f, previousStep, FOOTSTEP_STATE_NORMAL, result)
			_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.075f*radiansInterval, 1.3f, previousStep, FOOTSTEP_STATE_NORMAL, result)
			_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.1f*radiansInterval, 1.3f, previousStep, FOOTSTEP_STATE_NORMAL, result)
			*/
			/*
			_addFootstepIfValid(0.6f, parabolaPhiStraight - 0.04f*radiansInterval, false, 1.0f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.6f, parabolaPhiStraight - 0.15f*radiansInterval, false, 1.0f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.6f, parabolaPhiStraight - 0.23f*radiansInterval, false, 1.0f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.7f, parabolaPhiStraight - 0.48f*radiansInterval, false, 0.9f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.8f, parabolaPhiStraight - 0.96f*radiansInterval, false, 0.8f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			*/
			_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.04f*radiansInterval, false, 1.0f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.15f*radiansInterval, false, 1.0f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.23f*radiansInterval, false, 1.0f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.7f, parabolaPhiStraight + 0.48f*radiansInterval, false, 0.9f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.8f, parabolaPhiStraight + 0.96f*radiansInterval, false, 0.8f, previousStep, FOOTSTEP_STATE_NORMAL, result);
		}
		else {
			// AFTER A LARGE CURVE, the choices are actually slower.

			if ( ((rightDotProduct < 0.0f) && (previousStep.whichFoot == RIGHT_FOOT)) || ((rightDotProduct > 0.0f) && (previousStep.whichFoot == LEFT_FOOT)) ) {
				if (  frontDotProduct > 0.2f) {
					// in this case goal is oriented to inner side of foot, in front
					_addFootstepIfValid(0.57f, idealParabolaPhi, true, 0.6f, previousStep, FOOTSTEP_STATE_NORMAL, result);				}
				else {
					// in this case goal is oriented to inner side of foot, NOT in front
					// this means we can use a corrective step, but it needs to be wary of the foot orientation limits.
					// foot orientation limits are accounted for in _createFootstepAction.
					// turn step?
					_addFootstepIfValid(0.7f, idealParabolaPhi, true, 0.4f, previousStep, FOOTSTEP_STATE_NORMAL, result);
				}
			}
			else {
				if (  frontDotProduct > 0.2f) {
					// in this case goal is oriented to outer side of foot, in front
					// the primary reasonable choice in this case is to simply walk straight.
					_addFootstepIfValid(0.57f, parabolaPhiStraight + 0.05f*radiansInterval, false, 0.6f, previousStep, FOOTSTEP_STATE_NORMAL, result);
				}
				else {
					// in this case goal is oriented to outer side of foot, NOT in front.
					// the character's "ideal" choice would be to actually "stop" and accelerate in the correct direction on the next step.
					_addFootstepIfValid(0.71f, parabolaPhiStraight + 0.05f*radiansInterval, false, 0.4f, previousStep, FOOTSTEP_STATE_NORMAL, result);
				}
			}


			// ALSO ALLOW SOME GENERIC TURNING CHOICES AS WELL...

			/*
			_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.05f*radiansInterval, 0.5f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.075f*radiansInterval, 0.5f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			*/
			// _addFootstepIfValid(0.6f, parabolaPhiStraight - 0.1f*radiansInterval, false, 0.5f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.1f*radiansInterval, false, 0.5f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.15f*radiansInterval, false, 0.5f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.7f, parabolaPhiStraight + 0.23f*radiansInterval, false, 0.4f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.7f, parabolaPhiStraight + 0.48f*radiansInterval, false, 0.4f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.7f, parabolaPhiStraight + 0.96f*radiansInterval, false, 0.4f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			
		}

		// ALSO add a stopping option.
		_addFootstepIfValid(0.4f, parabolaPhiStraight + 0.15f*radiansInterval, false, 0.0f, previousStep, FOOTSTEP_STATE_STOPPING, result);

		//_addFootstepIfValid(0.4f, parabolaPhiStraight, false, 0.0f, previousStep, FOOTSTEP_STATE_BACKSTEP, result);

	}
	else if (previousStep.state == FOOTSTEP_STATE_STOPPING)
	{
		// valid state transitions are:
		// STOPPING --> STARTING      (whichFoot DOES NOT alternate)
		// STOPPING --> STATIONARY      (whichFoot does not matter when stationary (except for knowing which foot location is actually described in the data structure))

		// options to generate:
		// 1. (STARTING) precision starting towards goal
		// 2. (STARTING) precision starting in other directions?
		// 3. (STATIONARY) stationary special step

		_addFootstepIfValid(0.4f, idealParabolaPhi, true, 0.5f, previousStep, FOOTSTEP_STATE_STARTING, result);
		_addFootstepIfValid(0.4f, parabolaPhiStraight + 0.23f*radiansInterval, false, 0.5f, previousStep, FOOTSTEP_STATE_STARTING, result);
		_addFootstepIfValid(0.5f, parabolaPhiStraight + 0.48f*radiansInterval, false, 0.4f, previousStep, FOOTSTEP_STATE_STARTING, result);
		_addFootstepIfValid(0.5f, parabolaPhiStraight + 0.96f*radiansInterval, false, 0.4f, previousStep, FOOTSTEP_STATE_STARTING, result);

		_addFootstepIfValid(0.45f, parabolaPhiStraight + 0.05f*radiansInterval, false, 0.0f, previousStep, FOOTSTEP_STATE_STATIONARY, result);



		// add backstepping choices
		/*_addFootstepIfValid(0.4f, idealParabolaPhi, true, -0.5f, previousStep, FOOTSTEP_STATE_STARTING, result);
		_addFootstepIfValid(0.4f, parabolaPhiStraight + 0.23f*radiansInterval, false, -0.5f, previousStep, FOOTSTEP_STATE_STARTING, result);
		_addFootstepIfValid(0.5f, parabolaPhiStraight + 0.48f*radiansInterval, false, -0.4f, previousStep, FOOTSTEP_STATE_STARTING, result);
		_addFootstepIfValid(0.5f, parabolaPhiStraight + 0.96f*radiansInterval, false, -0.4f, previousStep, FOOTSTEP_STATE_STARTING, result);*/
	}
	/*else if (previousStep.state == FOOTSTEP_STATE_STARTING) {
		// valid state transitions are:
		// STARTING --> NORMAL          (whichFoot alternates)
		// STARTING --> STOPPING        (whichFoot alternates)

		// options to generate:
		// 1. (NORMAL) same choices as normal choices above.
		// 2. (STOPPING) single stop option that steps forward.

		_addFootstepIfValid(0.4f, idealParabolaPhi, 0.5f, previousStep, FOOTSTEP_STATE_NORMAL, result);
		_addFootstepIfValid(0.4f, parabolaPhiStraight + 0.23f*radiansInterval, 0.5f, previousStep, FOOTSTEP_STATE_NORMAL, result);
		_addFootstepIfValid(0.5f, parabolaPhiStraight + 0.48f*radiansInterval, 0.4f, previousStep, FOOTSTEP_STATE_NORMAL, result);
		_addFootstepIfValid(0.5f, parabolaPhiStraight + 0.96f*radiansInterval, 0.4f, previousStep, FOOTSTEP_STATE_STARTING, result);
		_addFootstepIfValid(0.4f, parabolaPhiStraight + 0.05f*radiansInterval, 0.5f, previousStep, FOOTSTEP_STATE_STATIONARY, result);
	}*/
	else if (previousStep.state == FOOTSTEP_STATE_STATIONARY || previousStep.state == FOOTSTEP_STATE_INPLACETURNING)
	{
		// valid state transitions are:
		// STATIONARY --> STATIONARY      (whichFoot does not matter when stationary)
		// STATIONARY --> STARTING      (whichFoot can be left or right depending on the chosen trajectory direction)

		// options to generate:
		// 1. (STATIONARY) character stays stationary for longer
		// 2. (STARTING) in any direction
		_addFootstepIfValid(0.4f, idealParabolaPhi, true, 0.5f, previousStep, FOOTSTEP_STATE_STARTING, result);
		_addFootstepIfValid(0.4f, parabolaPhiStraight + 0.23f*radiansInterval, false, 0.5f, previousStep, FOOTSTEP_STATE_STARTING, result);
		_addFootstepIfValid(0.5f, parabolaPhiStraight + 0.48f*radiansInterval, false, 0.4f, previousStep, FOOTSTEP_STATE_STARTING, result);
		_addFootstepIfValid(0.5f, parabolaPhiStraight + 0.96f*radiansInterval, false, 0.4f, previousStep, FOOTSTEP_STATE_STARTING, result);

		_addFootstepIfValid(0.4f, parabolaPhiStraight + 0.05f*radiansInterval, false, 0.5f, previousStep, FOOTSTEP_STATE_STATIONARY, result);

#ifdef ENABLE_INPLACE_TURNING
		// add in-place turning choices
		_addFootstepIfValid(0.4f, parabolaPhiStraight + 0.5f*radiansInterval, false, 0.5f, previousStep, FOOTSTEP_STATE_INPLACETURNING, result);
		_addFootstepIfValid(0.4f, parabolaPhiStraight - 0.5f*radiansInterval, false, 0.5f, previousStep, FOOTSTEP_STATE_INPLACETURNING, result);
#endif


		// add backstepping choices
		/*_addFootstepIfValid(0.4f, idealParabolaPhi, true, -0.5f, previousStep, FOOTSTEP_STATE_STARTING, result);
		_addFootstepIfValid(0.4f, parabolaPhiStraight + 0.23f*radiansInterval, false, -0.5f, previousStep, FOOTSTEP_STATE_STARTING, result);
		_addFootstepIfValid(0.5f, parabolaPhiStraight + 0.48f*radiansInterval, false, -0.4f, previousStep, FOOTSTEP_STATE_STARTING, result);
		_addFootstepIfValid(0.5f, parabolaPhiStraight + 0.96f*radiansInterval, false, -0.4f, previousStep, FOOTSTEP_STATE_STARTING, result);

		_addFootstepIfValid(0.4f, parabolaPhiStraight + 0.05f*radiansInterval, false, -0.5f, previousStep, FOOTSTEP_STATE_STATIONARY, result);*/
	}
	else if (previousStep.state == FOOTSTEP_STATE_HOPPING)
	{
		//_addFootstepIfValid(0.4f, parabolaPhiStraight + 0.15f*radiansInterval, false, 0.0f, previousStep, FOOTSTEP_STATE_STOPPING, result);
		//_addFootstepIfValid(0.4f, parabolaPhiStraight + 0.05f*radiansInterval, false, 0.5f, previousStep, FOOTSTEP_STATE_STATIONARY, result);
		//_addFootstepIfValid(0.45f, parabolaPhiStraight + 0.05f*radiansInterval, false, 0.0f, previousStep, FOOTSTEP_STATE_STATIONARY, result);

		/*_addFootstepIfValid(0.4f, idealParabolaPhi, true, 0.5f, previousStep, FOOTSTEP_STATE_STARTING, result);
		_addFootstepIfValid(0.4f, parabolaPhiStraight + 0.23f*radiansInterval, false, 0.5f, previousStep, FOOTSTEP_STATE_STARTING, result);
		_addFootstepIfValid(0.5f, parabolaPhiStraight + 0.48f*radiansInterval, false, 0.4f, previousStep, FOOTSTEP_STATE_STARTING, result);
		_addFootstepIfValid(0.5f, parabolaPhiStraight + 0.96f*radiansInterval, false, 0.4f, previousStep, FOOTSTEP_STATE_STARTING, result);
		_addFootstepIfValid(0.4f, parabolaPhiStraight - 0.23f*radiansInterval, false, 0.5f, previousStep, FOOTSTEP_STATE_STARTING, result);
		_addFootstepIfValid(0.5f, parabolaPhiStraight - 0.48f*radiansInterval, false, 0.4f, previousStep, FOOTSTEP_STATE_STARTING, result);
		_addFootstepIfValid(0.5f, parabolaPhiStraight - 0.96f*radiansInterval, false, 0.4f, previousStep, FOOTSTEP_STATE_STARTING, result);*/
		_addFootstepIfValid(0.4f, idealParabolaPhi, true, 0.5f, previousStep, FOOTSTEP_STATE_NORMAL, result);
		_addFootstepIfValid(0.4f, parabolaPhiStraight + 0.23f*radiansInterval, false, 0.5f, previousStep, FOOTSTEP_STATE_NORMAL, result);
		_addFootstepIfValid(0.5f, parabolaPhiStraight + 0.48f*radiansInterval, false, 0.4f, previousStep, FOOTSTEP_STATE_NORMAL, result);
		_addFootstepIfValid(0.5f, parabolaPhiStraight + 0.96f*radiansInterval, false, 0.4f, previousStep, FOOTSTEP_STATE_NORMAL, result);
		_addFootstepIfValid(0.4f, parabolaPhiStraight - 0.23f*radiansInterval, false, 0.5f, previousStep, FOOTSTEP_STATE_NORMAL, result);
		_addFootstepIfValid(0.5f, parabolaPhiStraight - 0.48f*radiansInterval, false, 0.4f, previousStep, FOOTSTEP_STATE_NORMAL, result);
		_addFootstepIfValid(0.5f, parabolaPhiStraight - 0.96f*radiansInterval, false, 0.4f, previousStep, FOOTSTEP_STATE_NORMAL, result);
	}
	else if (previousStep.state == FOOTSTEP_STATE_BACKSTEP)
	{
		// valid state transitions are:
		// NORMAL --> NORMAL      (whichFoot alternates)	
		// NORMAL --> STOPPING      (whichFoot alternates)

		// options to generate:
		// 1. (NORMAL) precision towards goal
		// 2. (NORMAL) precision slightly displaced away from goal
		// 3. (NORMAL) large turns
		// 4. (STOPPING) single stop option that steps forward.


		// COMPUTE GOAL-DEPENDENT CHOICES.
		// 
		// goal-dependent choices depend on where the goal is w.r.t. the character.
		// there are 4 main choices:
		//   - goal oriented to outer side of foot, in front
		//   - goal oriented to outer side of foot, NOT in front (a slightly larger region than simply "from behind")
		//   - goal oriented to inner side of foot, in front
		//   - goal oriented to inner side of foot, NOT in front.

		if (previousStep.simulationA < 0.9f) { // this is uninitiallized sometimes. specifically simulationA

			if ( ((rightDotProduct < 0.0f) && (previousStep.whichFoot == RIGHT_FOOT)) || ((rightDotProduct > 0.0f) && (previousStep.whichFoot == LEFT_FOOT)) ) {
				if (  frontDotProduct > 0.2f) {
					// in this case goal is oriented to inner side of foot, in front
					_addFootstepIfValid(0.6f, idealParabolaPhi, true, 1.1f, previousStep, FOOTSTEP_STATE_NORMAL, result);
				}
				else {
					// in this case goal is oriented to inner side of foot, NOT in front
					_addFootstepIfValid(0.6f, idealParabolaPhi, true, 1.1f, previousStep, FOOTSTEP_STATE_NORMAL, result);
				}
			}
			else {
				if (  frontDotProduct > 0.2f) {
					// in this case goal is oriented to outer side of foot, in front
					// the primary reasonable choice in this case is to simply walk straight.
					_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.05f*radiansInterval, false, 1.0f, previousStep, FOOTSTEP_STATE_NORMAL, result);
				}
				else {
					// in this case goal is oriented to outer side of foot, NOT in front.
					// the character's "ideal" choice would be to actually "stop" and accelerate in the correct direction on the next step.
					_addFootstepIfValid(0.5f, parabolaPhiStraight + 0.05f*radiansInterval, false, 0.6f, previousStep, FOOTSTEP_STATE_NORMAL, result);
				}
			}


			// ALSO ALLOW SOME GENERIC TURNING CHOICES AS WELL...

			/*
			_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.05f*radiansInterval, 1.3f, previousStep, FOOTSTEP_STATE_NORMAL, result)
			_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.075f*radiansInterval, 1.3f, previousStep, FOOTSTEP_STATE_NORMAL, result)
			_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.1f*radiansInterval, 1.3f, previousStep, FOOTSTEP_STATE_NORMAL, result)
			*/
			/*
			_addFootstepIfValid(0.6f, parabolaPhiStraight - 0.04f*radiansInterval, false, 1.0f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.6f, parabolaPhiStraight - 0.15f*radiansInterval, false, 1.0f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.6f, parabolaPhiStraight - 0.23f*radiansInterval, false, 1.0f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.7f, parabolaPhiStraight - 0.48f*radiansInterval, false, 0.9f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.8f, parabolaPhiStraight - 0.96f*radiansInterval, false, 0.8f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			*/
			_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.04f*radiansInterval, false, 1.0f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.15f*radiansInterval, false, 1.0f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.23f*radiansInterval, false, 1.0f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.7f, parabolaPhiStraight + 0.48f*radiansInterval, false, 0.9f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.8f, parabolaPhiStraight + 0.96f*radiansInterval, false, 0.8f, previousStep, FOOTSTEP_STATE_NORMAL, result);
		}
		else {
			// AFTER A LARGE CURVE, the choices are actually slower.

			if ( ((rightDotProduct < 0.0f) && (previousStep.whichFoot == RIGHT_FOOT)) || ((rightDotProduct > 0.0f) && (previousStep.whichFoot == LEFT_FOOT)) ) {
				if (  frontDotProduct > 0.2f) {
					// in this case goal is oriented to inner side of foot, in front
					_addFootstepIfValid(0.57f, idealParabolaPhi, true, 0.6f, previousStep, FOOTSTEP_STATE_NORMAL, result);				}
				else {
					// in this case goal is oriented to inner side of foot, NOT in front
					// this means we can use a corrective step, but it needs to be wary of the foot orientation limits.
					// foot orientation limits are accounted for in _createFootstepAction.
					// turn step?
					_addFootstepIfValid(0.7f, idealParabolaPhi, true, 0.4f, previousStep, FOOTSTEP_STATE_NORMAL, result);
				}
			}
			else {
				if (  frontDotProduct > 0.2f) {
					// in this case goal is oriented to outer side of foot, in front
					// the primary reasonable choice in this case is to simply walk straight.
					_addFootstepIfValid(0.57f, parabolaPhiStraight + 0.05f*radiansInterval, false, 0.6f, previousStep, FOOTSTEP_STATE_NORMAL, result);
				}
				else {
					// in this case goal is oriented to outer side of foot, NOT in front.
					// the character's "ideal" choice would be to actually "stop" and accelerate in the correct direction on the next step.
					_addFootstepIfValid(0.71f, parabolaPhiStraight + 0.05f*radiansInterval, false, 0.4f, previousStep, FOOTSTEP_STATE_NORMAL, result);
				}
			}


			// ALSO ALLOW SOME GENERIC TURNING CHOICES AS WELL...

			/*
			_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.05f*radiansInterval, 0.5f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.075f*radiansInterval, 0.5f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			*/
			// _addFootstepIfValid(0.6f, parabolaPhiStraight - 0.1f*radiansInterval, false, 0.5f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.1f*radiansInterval, false, 0.5f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.6f, parabolaPhiStraight + 0.15f*radiansInterval, false, 0.5f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.7f, parabolaPhiStraight + 0.23f*radiansInterval, false, 0.4f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.7f, parabolaPhiStraight + 0.48f*radiansInterval, false, 0.4f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			_addFootstepIfValid(0.7f, parabolaPhiStraight + 0.96f*radiansInterval, false, 0.4f, previousStep, FOOTSTEP_STATE_NORMAL, result);
			
		}

		// ALSO add a stopping option.
		_addFootstepIfValid(0.4f, parabolaPhiStraight + 0.15f*radiansInterval, false, 0.0f, previousStep, FOOTSTEP_STATE_STOPPING, result);
	}
	else
	{
		std::cout << "Footstep Agent lost state" << std::endl;
	}

	/*if (previousStep.state == FOOTSTEP_STATE_STATIONARY)
	{
		
		if (_addFootstepIfValid(1.2f, idealParabolaPhi, true, 2.2f, previousStep, FOOTSTEP_STATE_HOPPING, result))
		{
			int asd = 2;

		}
	}*/

#ifdef ENABLE_RANDOM_CHOICES
	// Here we add a random choice
	srand(_numNodesExpanded + nodeId + lastNodeId + agent->_footstepCreationAttempts); // make things somewhat predictable for debugging
	const double randomAngle = parabolaPhiStraight + ((((double) rand() / (RAND_MAX)))) * radiansInterval * 0.3; // not totally random, but probably good enough
	// std::cout << "trying random footstep angle " << randomAngle << std::endl;
	srand(_numNodesExpanded + nodeId + lastNodeId + agent->_footstepCreationAttempts); // make things somewhat predictable for debugging
	const double randomTime = fabs((((double) rand() / (RAND_MAX)))) + 0.3; // not totally random, but probably good enough
	if (previousStep.state == FOOTSTEP_STATE_STATIONARY)
	{
		_addFootstepIfValid(randomTime, randomAngle, false, 0.3f, previousStep, FOOTSTEP_STATE_STARTING, result);
#ifdef ENABLE_INPLACE_TURNING
		// add in-place turning choices
		// std::cout << "Trying inplace turning random: angle " << randomAngle << std::endl;
		_addFootstepIfValid(randomTime, randomAngle, false, 0.5f, previousStep, FOOTSTEP_STATE_INPLACETURNING, result);
		_addFootstepIfValid(randomTime, randomAngle, false, 0.5f, previousStep, FOOTSTEP_STATE_INPLACETURNING, result);
#endif
	}
	else if (previousStep.state == FOOTSTEP_STATE_NORMAL || previousStep.state == FOOTSTEP_STATE_STARTING)	{
		_addFootstepIfValid(randomTime, randomAngle, false, 0.3f, previousStep, FOOTSTEP_STATE_NORMAL, result);
	}
	else if (previousStep.state == FOOTSTEP_STATE_STOPPING)	{
		_addFootstepIfValid(randomTime, randomAngle, false, 0.3f, previousStep, FOOTSTEP_STATE_STARTING, result);
	}
	else if (previousStep.state == FOOTSTEP_STATE_HOPPING) {
		_addFootstepIfValid(randomTime, randomAngle, false, 0.3f, previousStep, FOOTSTEP_STATE_NORMAL, result);
	}
		
#endif

	// take a hop in any state if it's in front of us
	/*if (frontDotProduct > 0.9)
		_addFootstepIfValid(1.2f, idealParabolaPhi, true, 2.2f, previousStep, FOOTSTEP_STATE_HOPPING, result);
	*/

	//if (frontDotProduct > 0.9)
	//	_addFootstepIfValid(1.0f, idealParabolaPhi, true, 2.0f, previousStep, FOOTSTEP_STATE_HOPPING, result);

}

bool FootstepEnvironment::isValidNodeId(int nodeId) const
{
	// TODO for testing code, do a real verification.
	
	// for performance code, return true
	return true;
}

