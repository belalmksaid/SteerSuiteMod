//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#include "SteerLib.h"
#include "EgocentricAgent.h"
#include "EgocentricAIModule.h"
#include "EgocentricMap.h"
#include "GridAStar.h"

/// @file EgocentricAgent.cpp
/// @brief Implements the EgocentricAgent class.

#define MAX_FORCE_MAGNITUDE 3.0f
#define MAX_SPEED 1.33f
#define AGENT_MASS 1.0f

#ifndef INFINITY
#define INFINITY FLT_MAX
#endif


using namespace EgocentricGlobals;

EgocentricAgent::EgocentricAgent()
{
	//std::cout << " non parametrized constructor \n";
	_midTermPath = new int[EgocentricGlobals::ego_next_waypoint_distance];
	// _midTermPath = int[202];
	// _midTermPath = malloc(EgocentricGlobals::ego_next_waypoint_distance * sizeof(int));
	_enabled = false;
}

EgocentricAgent::~EgocentricAgent()
{
	if (_enabled) {
		Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
		gSpatialDatabase->removeObject( dynamic_cast<SpatialDatabaseItemPtr>(this), bounds);
		_enabled=false;
	}

	free (t); 
	free ( _esm);
	// free(_midTermPath);
}

SteerLib::EngineInterface * EgocentricAgent::getSimulationEngine()
{
	return gEngine;
}

void EgocentricAgent::init (const int & numberOfNodesPerLevel, const int & numberOfLevels)
{
	assert(_forward.length()!=0.0f);

	assert(_radius != 0.0f);

	//
	// init the vars that don't get initialized from the test case.
	// here they are grouped the same way that they are grouped in the class declaration.
	//

	//==============================================
	_waypoints.clear();
	_currentWaypointIndex = 0;

	//==============================================
	//int _midTermPath[ego_next_waypoint_distance]; // TODO: initialize
	_midTermPathSize = 0;

	//==============================================
	_currentGoal = _goalQueue.front();
	//_localTargetLocation = _currentGoal;
	//==============================================

	runLongTermPlanningPhase();
	runMidTermPlanningPhase();
	runShortTermPlanningPhase();


	_finalSteeringCommand.clear();
	//==============================================
	_rightSide = rightSideInXZPlane(_forward);
	//==============================================
	//_velocity = normalize(_forward) * _currentSpeed;
	_mass = 1.0f;
	_maxSpeed = ped_max_speed;
	_maxForce = ped_max_force;
	//==============================================
	_currentTimeStamp = 0.0f;
	_dt = 0.0f;
	_currentFrameNumber = 0;
	//==============================================

	// MUBBASIR TODO -- Is there a better place and value to initialize this ?? 
	_desiredSpeed = 1.3f;

	if (_desiredSpeed > _maxSpeed) {
		std::cerr << "Warning, initial desired speed (" << _desiredSpeed << " m/s) is larger than the max speed (" << _maxSpeed << " m/s) of our agent." << std::endl;
	}

	// MUBBASIR - Instantiating the map and t here. 

	t = (float *) malloc(numberOfLevels * sizeof(float));

	_esm = new EgocentricMap ( numberOfNodesPerLevel, numberOfLevels, 
		position(), currentGoal().targetLocation );

	_esm->updateMapAtEveryMovement ( _position, 0.0);

	// Initializing _esm->nodeIdToMoveTo to the node for "_forward"
	//_esm->nodeIdToMoveTo = _esm->getNodeIdFromPositionVector ( _position + _forward, false );

	//std::cout << "Initial nodeIdToMoveTo : " << _esm->nodeIdToMoveTo << std::endl;


	// for determineSpeed()
	maintainSpeed = false;
}
void EgocentricAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	// compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
	// because the value is not used in that case.
	Util::AxisAlignedBox oldBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);

	// initialize the agent based on the initial conditions
	_position = initialConditions.position;
	_forward = initialConditions.direction;
	_radius = initialConditions.radius;
	_velocity = initialConditions.speed * Util::normalize(initialConditions.direction);
	_currentSpeed = _velocity.length();

	// compute the "new" bounding box of the agent
	Util::AxisAlignedBox newBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);

	if (!_enabled) {
		// if the agent was not enabled, then it does not already exist in the database, so add it.
		gSpatialDatabase->addObject( dynamic_cast<SpatialDatabaseItemPtr>(this), newBounds);
	}
	else {
		// if the agent was enabled, then the agent already existed in the database, so update it instead of adding it.
		gSpatialDatabase->updateObject( dynamic_cast<SpatialDatabaseItemPtr>(this), oldBounds, newBounds);
	}

	_enabled = true;

	if (initialConditions.goals.size() == 0) {
		throw Util::GenericException("No goals were specified!\n");
	}

	// iterate over the sequence of goals specified by the initial conditions.
	for (unsigned int i=0; i<initialConditions.goals.size(); i++) {
		if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
			_goalQueue.push(initialConditions.goals[i]);
			if (initialConditions.goals[i].targetIsRandom) {
				// if the goal is random, we must randomly generate the goal.
				SteerLib::AgentGoalInfo _goal;
				_goal.targetLocation = gSpatialDatabase->randomPositionWithoutCollisions(1.0f, true);
				_goalQueue.push(_goal);
			}
		}
		else {
			throw Util::GenericException("Unsupported goal type; EgocentricAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET.");
		}
	}

	assert(_forward.length()!=0.0f);
	assert(_goalQueue.size() != 0);
	assert(_radius != 0.0f);

	if ( (_currentSpeed != _currentSpeed) || (_velocity != _velocity) )
	{
		throw GenericException("Something happend to the current speed");
	}

	// MUBBASIR TODO -- Adding init here.
	init (16,8);
}

void EgocentricAgent::runLongTermPlanningPhase()
{

	AStarLite longTermAStar;

	if (!_enabled) return;

	// if we're at our destination, then chose a new landmark target.
	if (reachedCurrentGoal()) {
		runCognitivePhase();
		if (!_enabled) return;
	}

	//==========================================================================
	int myIndexPosition = gSpatialDatabase->getCellIndexFromLocation(_position);
	int goalIndex = gSpatialDatabase->getCellIndexFromLocation(_currentGoal.targetLocation);

	if (myIndexPosition != -1 && goalIndex != -1) {

		//GridEnvironment gridEnvironment;

		// run the main a-star search here
		longTermAStar.findPath( (*gEnvironmentForAStar), myIndexPosition, goalIndex);
		//longTermAStar.findPath( *(dynamic_cast<Environment*>(gEnvironmentForAStar)), myIndexPosition, goalIndex);

		// set up the waypoints along this path.
		// if there was no path, then just make one waypoint that is the landmark target.
		_waypoints.clear();
		int longTermAStarLength = (int) longTermAStar.getPath().size();
		//std::cout << "long term length " << longTermAStarLength << "\n";
		if (longTermAStarLength > 2) {
			// note the >2 condition: if the astar path is not at least this large, then there will be a behavior bug in the AI
			// when it tries to create waypoints.  in this case, the right thing to do is create only one waypoint that is at the landmark target.
			// remember the astar lib produces "backwards" paths that start at [pathLengh-1] and end at [0].
			int nextWaypointIndex = ((int)longTermAStar.getPath().size())-1 - waypoint_distance;
			while (nextWaypointIndex > 0) {
				Point result; 
				gSpatialDatabase->getLocationFromIndex(longTermAStar.getPath()[nextWaypointIndex], result);
				_waypoints.push_back(result);
				nextWaypointIndex -= waypoint_distance;
			}
			_waypoints.push_back(_currentGoal.targetLocation);
		}
		else {
			_waypoints.push_back(_currentGoal.targetLocation);
		}

		// since we just computed a new long-term path, set the character to steer towards the first waypoint.
		_currentWaypointIndex = 0; 
	}
	else {
		// can't do A-star if we are outside the database.
		// this happens rarely, if ever, but still needs to be robustly handled... this seems like a reasonable decision to make in the extreme case.
		_waypoints.push_back(_currentGoal.targetLocation);
	}

}

void EgocentricAgent::runMidTermPlanningPhase()
{

	AStarLite midTermAStar;

	if (!_enabled) return;

	// if we reached the final goal, then schedule long-term planning to run
	// long-term planning will call cognitive
	if (reachedCurrentGoal()) {
		runLongTermPlanningPhase();
		if (!_enabled) return;
	}

	// if we reached the current waypoint, then increment to the next waypoint
	if (reachedWaypoint() && ( (_currentWaypointIndex-1) < _waypoints.size())) {
		_currentWaypointIndex++;
	}

	// compute a local a-star from your current location to the waypoint.
	int myIndexPosition = gSpatialDatabase->getCellIndexFromLocation(_position);
	int waypointIndexPosition = gSpatialDatabase->getCellIndexFromLocation(_waypoints[_currentWaypointIndex]);
	
	// MUBBASIR ADDED THIS 
	if ( myIndexPosition == -1 || waypointIndexPosition == -1)
	{
		std::cerr << "egocentric mid term PROBLEM \n";
		_midTermPathSize =0;
		return;
	}

	midTermAStar.findPath((*gEnvironmentForAStar), myIndexPosition, waypointIndexPosition);

	// copy the local AStar path to your array
	// keeping the convention that the beginning of the path starts at the end of the array
	_midTermPathSize = (int)(midTermAStar.getPath().size());

	// sanity checks
	if (_midTermPathSize != (int)(midTermAStar.getPath().size()) ) {
		printf("ERROR!!!  _midTermPathSize does not equal (midTermAStar.getPath()).size(). exiting ungracefully.\n");
		exit(1);
	}
	if (_midTermPathSize > EgocentricGlobals::ego_next_waypoint_distance) {
		// the plus 1 is because a-star counts the agent's immediate location, but we do not.
		printf("ERROR!!!  _midTermPathSize is larger than expected: should be less than or equal to %d, but it actually is %d.\n", ego_next_waypoint_distance+1, _midTermPathSize);
		printf("exiting ungracefully.\n");
		exit(1);
	}

	for (int i=0; i<_midTermPathSize-1; i++) {
		_midTermPath[i] = (int) (midTermAStar.getPath())[i];
		// _midTermPath[i] = midTermAStar.top();
		// midTermAStar.pop();
	}

	// TODO: should we reset the localTarget here??
}

void EgocentricAgent::runShortTermPlanningPhase()
{
	int closestPathNode;
	if (!_enabled) return;


	// 0. if you're at your current waypoint
	if (reachedWaypoint()) {
		// then schedule midTermPlanning phase
		runMidTermPlanningPhase();
		if (!_enabled) return;
	}

	int myIndexPosition = gSpatialDatabase->getCellIndexFromLocation(_position.x, _position.z);
	closestPathNode = _midTermPathSize; // the last index in the path array  *** PLUS ONE ***
	if ((myIndexPosition!=-1)&&(closestPathNode!=0)) {

		// 1. find the node that you're nearest to in your current path
		// NOTE that we MUST search ALL nodes of the path here.
		// reason: the path may unintuitively snake around so that some nodes are closer than others even if they are all very far from you.
		float minDistSquared = INFINITY;
		for (int i=_midTermPathSize-1; i >= 0; i--) {
			Point tempTargetLocation;
			gSpatialDatabase->getLocationFromIndex( _midTermPath[i], tempTargetLocation );
			Vector temp = tempTargetLocation-_position;
			float distSquared = temp.lengthSquared();
			if (minDistSquared > distSquared) {
				minDistSquared = distSquared;
				closestPathNode = i;
			}
		}

		// at this point in code, closestPathNode is the node that is closest to your ped's position.

		// 2. iterate over nodes tracing rays to the find the local target
		if (closestPathNode > 2)
		{
			float dummyt;
			SpatialDatabaseItem * dummyObject;
			int localTargetIndex = closestPathNode;
			int furthestTargetIndex = max(0, closestPathNode - furthest_local_target_distance);
			int localTarget = _midTermPath[localTargetIndex];
			gSpatialDatabase->getLocationFromIndex( localTarget, _localTargetLocation );
			Ray lineOfSightTest1, lineOfSightTest2;
			lineOfSightTest1.initWithUnitInterval(_position + _radius*_rightSide, _localTargetLocation - (_position + _radius*_rightSide));
			lineOfSightTest2.initWithUnitInterval(_position - _radius*_rightSide, _localTargetLocation - (_position - _radius*_rightSide));
			while ((!gSpatialDatabase->trace(lineOfSightTest1,dummyt, dummyObject, dynamic_cast<SpatialDatabaseItem*>(this),true))
				&& (!gSpatialDatabase->trace(lineOfSightTest2,dummyt, dummyObject, dynamic_cast<SpatialDatabaseItem*>(this),true))
				&& (localTargetIndex >= furthestTargetIndex))
			{
				localTargetIndex--;
				localTarget = _midTermPath[localTargetIndex];
				gSpatialDatabase->getLocationFromIndex( localTarget, _localTargetLocation  );
				lineOfSightTest1.initWithUnitInterval(_position + _radius*_rightSide, _localTargetLocation - (_position + _radius*_rightSide));
				lineOfSightTest2.initWithUnitInterval(_position - _radius*_rightSide, _localTargetLocation - (_position - _radius*_rightSide));
			}
			localTargetIndex++; // the last node we found was actually NOT visible, so backtrack by one.
			if (localTargetIndex <= closestPathNode) {
				// if localTargetIndex is valid
				localTarget = _midTermPath[localTargetIndex];
				gSpatialDatabase->getLocationFromIndex( localTarget, _localTargetLocation );
				if ((_localTargetLocation - _waypoints[_currentWaypointIndex]).length() < 2.0f * ego_ped_reached_target_distance_threshold) {
					_localTargetLocation = _waypoints[_currentWaypointIndex];
				}
			}
			else {
				// if localTargetIndex is pointing backwards, then just aim for 2 nodes ahead of the current closestPathNode.
				localTarget = _midTermPath[closestPathNode-2];
				gSpatialDatabase->getLocationFromIndex( localTarget, _localTargetLocation  );
			}
		}
		else {
			_localTargetLocation = _waypoints[_currentWaypointIndex];
		}
	}
	else {
		if (myIndexPosition==-1) {
			// this case is reached when you're outside the database
			// in this case, just point towards your waypoint and hope for the best.
			_localTargetLocation = _waypoints[_currentWaypointIndex];
		}
		else if (closestPathNode==0) {
			// this case is reached when you're very close to your goal, and the planned path is very short.
			// in this case, just point towards the closest node.
			gSpatialDatabase->getLocationFromIndex( closestPathNode, _localTargetLocation );
		}
		else {
			// this case should never be reached
			printf("ERROR: unexpected case! closestPathNode==%d,  myIndexPosition==%d.\nexiting ungracefully.\n", closestPathNode, myIndexPosition);
			exit(1);
		}
	}



}


void EgocentricAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
	Util::AutomaticFunctionProfiler profileThisFunction( &gPhaseProfilers->aiProfiler );

	// for this function, we assume that all goals are of type GOAL_TYPE_SEEK_STATIC_TARGET.
	// the error check for this was performed in reset().
	//Util::Vector vectorToGoal = _goalQueue.front().targetLocation - _position;

	//// it is up to the agent to decide what it means to have "accomplished" or "completed" a goal.
	//// for the simple AI, if the agent's distance to its goal is less than its radius, then the agent has reached the goal.
	//if (vectorToGoal.lengthSquared() < ego_ped_reached_target_distance_threshold * ego_ped_reached_target_distance_threshold) {
	//	_goalQueue.pop();
	//	if (_goalQueue.size() != 0) {
	//		// in this case, there are still more goals, so start steering to the next goal.
	//		vectorToGoal = _goalQueue.front().targetLocation - _position;
	//	}
	//	else {
	//		// in this case, there are no more goals, so disable the agent and remove it from the spatial database.
	//		Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
	//		gSpatialDatabase->removeObject( this, bounds);
	//		_enabled = false;
	//		return;
	//	}
	//}

	// -------------------------------- * * * ---------------------------------- //

	//// initialize some vars for this update step
	_currentTimeStamp = timeStamp;
	_currentFrameNumber = frameNumber;
	_dt = dt;


	_esm->goalPosition = goal();

	if (!enabled ())
		return;

	// SETTING THE TARGETDIRECTION AND NEURON ID'S FOR THE PREVIOUS STEP.
	_esm->prevTargetDirection = this->forward();

	_esm->prevNeuronId = _esm->getNeuronIdFromPositionVector(this->position() + forward(), false);
	_esm->prevNeuronId = (_esm->prevNeuronId) % _esm->numberOfNeuronsPerLevel;

	this->queryDatabase(_esm->numberOfLevels , 0, 360);

	// initializing steering command.
	_finalSteeringCommand.targetDirection = normalize(_esm->goalPosition - _position);
	_finalSteeringCommand.aimForTargetDirection = true;
	_finalSteeringCommand.aimForTargetSpeed = true;
	_finalSteeringCommand.turningAmount = 1.0f;
	_finalSteeringCommand.targetSpeed = _desiredSpeed;

	int goalId = _esm->getNeuronIdFromPositionVector(_esm->goalPosition,false );

	float targetS = _desiredSpeed;

	if (neighbors.size () == 0 )
	{
		// there is no one in front of me -- straight to goal  
		goto end;
	}

	int neuronIdLowRange,neuronIdHighRange;

	if(goalId != -1)
	{
		// Clearing activation.
		_esm->clearActivation();
		// Setting activation of goal to 1.0
		_esm->neuron[goalId]->activation = 1.0f;

#ifdef SPACE_TIME_PLANNING

		// GETTING BOUNDARY CONDITIONS FOR THE (180- ..)  DEGREES WHICH WE WANT TO CONSIDER 
		neuronIdLowRange = _esm->neuronId - 4;
		if(neuronIdLowRange < 0) 
		neuronIdLowRange = _esm->numberOfNeuronsPerLevel + neuronIdLowRange;

		neuronIdHighRange = (_esm->neuronId + 4) % _esm->numberOfNeuronsPerLevel;

		calculateTime(speed());
		queryDynamicObjectsinTime(neuronIdLowRange, neuronIdHighRange,8);
		targetS = determineTargetSpeed();
		calculateTime(targetS);
		//OpenSteerDemo::clock.setPausedState(true);
		queryDynamicObjectsinTime(neuronIdLowRange, neuronIdHighRange,8);

		// Spreading activation
		_esm->goalId = goalId;
		_esm->spreadActivation(goalId);
#else 

		// Spreading activation
		_esm->goalId = goalId;
		_esm->spreadActivation(goalId);

		// After activation has been set, CONSIDER DYNAMIC OBJECTS!!
		// Note -> this will work with the activation of the last cycle for the other peds. 
		evaluateDynamicObjects(5,0,360);
#endif


		//int tryAgainFlag = 0;
		//float cosineOfAngleBetweenForwardAndPositiveX = targetDirection.dot(Vec3(1.0,0.0,0.0));
		//float angleBetweenForwardAndPositiveX = acos(cosineOfAngleBetweenForwardAndPositiveX) * M_180_OVER_PI;
		
		//printf("%f %f\n",cosineOfAngleBetweenForwardAndPositiveX,angleBetweenForwardAndPositiveX);
		//printf("x : %f z : %f\n", targetDirection.x, targetDirection.z);
		
		neuronIdLowRange = _esm->neuronId - 3;
		if(neuronIdLowRange < 0) 
			neuronIdLowRange = _esm->numberOfNeuronsPerLevel + neuronIdLowRange;

		neuronIdHighRange = (_esm->neuronId + 2) % _esm->numberOfNeuronsPerLevel;
		
		int neuronId;
		//neuronId = e->determineNeuronIdToMoveTo(0, 15);
		//CHANGE -- TRYING TO INTRODUCE LOWRANGE AND HIGH RANGE
		//printf("ID'S BEFORE DETERMINE NEURONID : %d %d %d ",e->neuronId,neuronIdLowRange,neuronIdHighRange);
		neuronId = _esm->determineNeuronIdToMoveTo(neuronIdLowRange, neuronIdHighRange, true);

		int extremeFlag = 0;
		if(_esm->neuronId != -1 && abs(_esm->neuronId - neuronId) >= 4 )
		{
			/*printf("THIS IS AN EXTREME CASE !! nl:%d nh:%d gN:%d n:%d\n",
			neuronIdLowRange,neuronIdHighRange,_esm->neuronId,neuronId);*/
			extremeFlag = 1;
			//_finalSteeringCommand.aimForTargetDirection = false;
			//_finalSteeringCommand.aimForTargetSpeed = true;
			//_finalSteeringCommand.targetSpeed = 0.0f;
			//_finalSteeringCommand.turningAmount = 0.0f;
			//goto end;	


		}

		_esm->neuronId = neuronId;

		if (neuronId != -1)
		{
			if(neuronId < 100)
			{
				if ( _esm->neuron[neuronId]->activation < 0.2f )
				{
					//std::cout << " no activation .. may have divided by 0 \n";
					_finalSteeringCommand.aimForTargetDirection = false;
					_finalSteeringCommand.aimForTargetSpeed = true;
					_finalSteeringCommand.targetSpeed = 0.0f;
					_finalSteeringCommand.turningAmount = 0.0f;
					goto end;
				}
				_finalSteeringCommand.targetDirection = normalize(_esm->determineTargetDirection(neuronId,true, _forward) );
			}
			else
			{
				neuronId -= 100;
				if ( _esm->neuron[neuronId]->traversabilityRatio < 0.2f )
				{
					std::cout << " no traversability .. may have divided by 0 \n";
					_finalSteeringCommand.aimForTargetDirection = false;
					_finalSteeringCommand.aimForTargetSpeed = true;
					_finalSteeringCommand.targetSpeed = 0.0f;
					_finalSteeringCommand.turningAmount = 0.0f;
					goto end;
				}

				_esm->neuronId = neuronId;
				_finalSteeringCommand.targetDirection = normalize (_esm->determineTargetDirection(neuronId,false, _forward) );
			}

			if (extremeFlag == 1 )
			{
				//std::cout << " since extreme, interpolating with forward. \n";
				//_finalSteeringCommand.targetDirection = _finalSteeringCommand.targetDirection * 0.1f + _forward * 0.9f;
			}

			

			// CHANGE
			// PUTTING CONDITION FOR VALID TARGET DIRECTION
			if(_finalSteeringCommand.targetDirection.x >= -1.0 && _finalSteeringCommand.targetDirection.x <= 1.0 &&
				_finalSteeringCommand.targetDirection.z >= -1.0 && _finalSteeringCommand.targetDirection.z <= 1.0 )
			{
				_finalSteeringCommand.aimForTargetDirection = true;
			}
			else
			{
				printf("Invalid Target Direction. Setting aimForTargetDirection to false.\n\n");
				_finalSteeringCommand.aimForTargetSpeed = true;
				_finalSteeringCommand.targetSpeed = 0.0;
				_finalSteeringCommand.aimForTargetDirection = false;
				_finalSteeringCommand.turningAmount = 0.0;
				goto end;
			}
			

			// SPEED MANIPULATION STARTS HERE // 

			_finalSteeringCommand.aimForTargetSpeed = true;

			// Speed is dependant on the activationPed around the agent. 

			int prevNeuronId,nextNeuronId;
			
			if(neuronId == 0) prevNeuronId = _esm->numberOfNeuronsPerLevel-1;
			else prevNeuronId = neuronId -1 ;

			if(neuronId == _esm->numberOfNeuronsPerLevel -1 ) nextNeuronId = 0;
			else nextNeuronId = neuronId + 1;
			
			float m1 = _esm->neuron[neuronId]->activationPed;
			float m2 = _esm->neuron[nextNeuronId]->activationPed;
			float m3 = _esm->neuron[prevNeuronId]->activationPed;
			float m4 = _esm->neuron[neuronId + _esm->numberOfNeuronsPerLevel]->activationPed;
			float m5 = _esm->neuron[nextNeuronId + _esm->numberOfNeuronsPerLevel]->activationPed;
			float m6 = _esm->neuron[prevNeuronId + _esm->numberOfNeuronsPerLevel]->activationPed;

			float m7,m8,m9;


			float minActivationPed = min(min(min(m1,m2),m3),min(min(m4,m5),m6));
			
			// CHANGE 3 
			if(minActivationPed > 0.1f)
			//if(minActivationPed > 0.5)
			{	
				// if going fast, look farther ahead. 
				m7 = _esm->neuron[neuronId + _esm->numberOfNeuronsPerLevel*2]->activationPed;
				m8 = _esm->neuron[nextNeuronId + _esm->numberOfNeuronsPerLevel*2]->activationPed;
				m9 = _esm->neuron[prevNeuronId + _esm->numberOfNeuronsPerLevel*2]->activationPed;

				minActivationPed = (minActivationPed + m7 + m8 + m9)/4.0f;

			}

			_finalSteeringCommand.targetSpeed = minActivationPed;	
			

	

			#ifdef USE_GOAL_APPROXIMATE
			printf("GOAL APPROXIMATE");
			//INTRODUCING GOAL APPROXIMATE.
			// IF PRETTY CLOSE TO THE GOAL, PUT AGENT AT GOAL.

			if(abs(e->root->centerPosition.x - e->goalPosition.x) < 2.0 &&
				abs(e->root->centerPosition.z - e->goalPosition.z) < 2.0 )
			{

			if ((this==OpenSteerDemo::selectedVehicle) && (OpenSteerDemo::camera.mode != Camera::cmFixed)) 
			{
				OpenSteerDemo::camera.mode = Camera::cmFixed;
				Vec3 target = position();
				Vec3 pos = OpenSteerDemo::camera.position();
				OpenSteerDemo::camera.fixedTarget.set (target.x, target.y, target.z);
				OpenSteerDemo::camera.fixedPosition.set (pos.x, pos.y, pos.z);
			}
			setPosition( 10000.0f, 10000.0f, 10000.0f);
			//gSpatialDatabase->removeObject(&databaseInfo);
			//enabled = false;
			}
			#endif

		}
		//TODO ::: WHEN IT FAILS FOR THIS CONDITION, IT MERELY GOES TOWARDS THE GOAL.
		// ADD SOMETHIN TO MAKE IT GO TOWARDS GOAL AND CHECK FOR COLLISION AVOIDANCE.
		// I.E CHOOSE NEXT CELL ON BASIS OF TRAVERSABILITY.

		/*
		scanning the first layer of neurons and picking the one with least traversability.
		we should give priority to whats in front of us.
		*/
		else
		{
			// it sometimes comes here and JUST STOPS!!
			//aimForTargetSpeed = true;
			//targetSpeed = 0.0;

			//int maxTraversabilityRatio = 0.0;
			//int a;
			//for( a = 0 ; a < e->numberOfNeuronsPerLevel/2; a ++)
			//{
			//	if(e->neuron[a]->traversabilityRatio > maxTraversabilityRatio)
			//	{
			//		maxTraversabilityRatio = e->neuron[a]->traversabilityRatio;
			//		neuronId = a;
			//	}
			//}

			//targetDirection = e->determineTargetDirection(neuronId).normalize();
			//aimForTargetDirection = true;
			//turningAmount = 0.5;
			//aimForTargetSpeed = true;
			//targetSpeed = ped_max_speed ;

			//std::cout << " Neuron Id is -1 \n";
			_finalSteeringCommand.targetSpeed = 0.0f;
			_finalSteeringCommand.aimForTargetSpeed = true;
			_finalSteeringCommand.aimForTargetDirection = false;
			std::cout << " am i coming here ?? \n";
			
		}



	}




	// CHANGE -- NORMALIZING TARGETSPEED
	//targetSpeed *= PED_TYPICALSPEED;
	//calculate distance between you and leader.
	_finalSteeringCommand.targetSpeed *= _desiredSpeed;
	
	//if(!leader)
	//{
	//	float distance = abs(e->root->centerPosition.z - leaderPointer->e->root->centerPosition.z);
	//
	//	if(distance <= 2.0f && targetSpeed > leaderPointer->targetSpeed) 
	//		targetSpeed = leaderPointer->targetSpeed;
	//}



#ifdef SPACE_TIME_PLANNING

	_finalSteeringCommand.targetSpeed = targetS;
#endif

end:  _finalSteeringCommand.scoot = 0.0f;

	Vector oldForward = _forward;


	// MUBBASIR -- If we use this, we will get constant speed of movement
	//_doEulerStep ( _finalSteeringCommand.targetDirection, dt );
	// MUBBASIR TODO -- There is some problem with this function
	//doCommandBasedSteering(dt);

	// MUBBASIR -- Trying own steering mechanism
		doSimpleSteering (dt);
	//float theta = acos( dot ( forward(), oldForward) );
	// MUBBASIR --- CHANGING CALCULATION FOR THETA 
	float theta = atan2( (forward().z - oldForward.z), (forward().x - oldForward.x) );
	if ( theta < 0.0f )
		theta = 360.0f + theta; 


	// MUBBASIR TODO -- PLEASE REMOVE THIS IF BLOCK
	//float initialDot = dot ( _finalSteeringCommand
	if ( theta >= 0.0f && theta <= 360.f)
	{
		// no problem
	}
	else
	{
		// MUBBASIR TODO -- WE SHOULD NEVER COME HERE 
		std::cout << " theta problem " << theta << "\n";
		theta = 360.0f;
	}

	_esm->updateMapAtEveryMovement(_position, theta);

	return;


	// -------------------------------- * * * ---------------------------------- //

}


void EgocentricAgent::draw()
{
#ifdef ENABLE_GUI
	// if the agent is selected, do some annotations just for demonstration
	if (gEngine->isAgentSelected(this)) {
		Util::Ray ray;
		ray.initWithUnitInterval(_position, _forward);
		float t = 0.0f;
		SteerLib::SpatialDatabaseItem * objectFound;

		//DrawLib::glColor(Util::gRed);
		Util::DrawLib::drawLine(ray.pos, ray.eval(1.0f));

		if (gSpatialDatabase->trace(ray, t, objectFound, this, false)) {
			Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gBlue);
		}
		else {
			Util::DrawLib::drawAgentDisc(_position, _forward, _radius);
		}

		// drawing map when selected
		_esm->drawMap (MapDisplayChoice_NodeNumber);

	}
	else {

		Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gDarkBlue);

		// MUBBASIR 
		// drawing direction of motion.
		
		Util::DrawLib::drawLine ( _position, _position + _forward * 2.0f);
		//_esm->drawMap (MapDisplayChoice::MapDisplayChoice_Activation);

	}
	if (_goalQueue.front().goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
		Util::DrawLib::drawFlag(_goalQueue.front().targetLocation);
	}


	std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
	gSpatialDatabase->getItemsInRange(_neighbors, _position.x-(this->_radius * 3), _position.x+(this->_radius * 3),
			_position.z-(this->_radius * 3), _position.z+(this->_radius * 3), dynamic_cast<SpatialDatabaseItemPtr>(this));

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin();  neighbor != _neighbors.end();  neighbor++)
	{
		if ( (*neighbor)->computePenetration(this->position(), this->_radius) > 0.0f)
		{
			Util::DrawLib::drawStar(this->position(), Util::Vector(1,0,0), 1.34f, gRed);
		}
	}
#endif
}





//
// doSimpleSteering()
//
void EgocentricAgent::doSimpleSteering (float dt)
{
	Vector steeringForce;
	Vector newForward; 

	// turn to face "targetDirection" - magnitude of targetDirection doesn't matter
	float initialDot = dot(_finalSteeringCommand.targetDirection,_rightSide);
	float turningRate = (initialDot > 0.0f) ? ped_max_turning_rate : -ped_max_turning_rate;  // positive rate is right-turn
	newForward = _forward + turningRate * fabsf(_finalSteeringCommand.turningAmount) * _rightSide;
	float newDot = dot(_finalSteeringCommand.targetDirection, rightSideInXZPlane(newForward)); // dot with the new side vector
	if (initialDot*newDot <= 0.0f) {
		// if the two dot products are different signs, that means we turned too much, so just set the new forward to the goal vector.
		// NOTE that above condition is less than **OR EQUALS TO** - that is because initialDot will be zero when the agent is 
		// pointing already in the exact correct direction.  If we used strictly less than, the pedestrian oscillates between a 
		// small offset direction and the actual target direction.
		newForward = _finalSteeringCommand.targetDirection;
	}

	_forward = newForward;
	_rightSide = rightSideInXZPlane(newForward);

	// This next line is specific to command-based steering, but is not physically based.
	// everything else in command-based steering, however, is physcially based.
	_velocity = newForward * _currentSpeed;
	if ( _velocity != _velocity)
	{
		throw GenericException("Something happend to the _velocity");
	}

	float scalarForce = (_finalSteeringCommand.targetSpeed - _currentSpeed) * 8.0f; // crudely trying to make accelerations quicker...
	if (scalarForce > _maxForce) scalarForce = _maxForce;
	//if (force < maxBackwardsForce) force = maxBackwardsForce;
	steeringForce = scalarForce * _forward; // forward is a unit vector, normalized during turning just above.

	_doEulerStep(steeringForce, dt);

}
//
// doCommandBasedSteering()
//
void EgocentricAgent::doCommandBasedSteering(float dt)
{

	// TODO: define/initialize these vars properly:
	Vector totalSteeringForce;
	Vector newForward;

	//
	// choose the new orientation of the agent
	//
	if (!_finalSteeringCommand.aimForTargetDirection) {
		// simple turning case "turn left" or "turn right"
		newForward = _forward + ped_max_turning_rate * _finalSteeringCommand.turningAmount * _rightSide;
	}
	else {
		// turn to face "targetDirection" - magnitude of targetDirection doesn't matter
		float initialDot = dot(_finalSteeringCommand.targetDirection,_rightSide);
		float turningRate = (initialDot > 0.0f) ? ped_max_turning_rate : -ped_max_turning_rate;  // positive rate is right-turn
		newForward = _forward + turningRate * fabsf(_finalSteeringCommand.turningAmount) * _rightSide;
		float newDot = dot(_finalSteeringCommand.targetDirection, rightSideInXZPlane(newForward)); // dot with the new side vector
		if (initialDot*newDot <= 0.0f) {
			// if the two dot products are different signs, that means we turned too much, so just set the new forward to the goal vector.
			// NOTE that above condition is less than **OR EQUALS TO** - that is because initialDot will be zero when the agent is 
			// pointing already in the exact correct direction.  If we used strictly less than, the pedestrian oscillates between a 
			// small offset direction and the actual target direction.
			newForward = _finalSteeringCommand.targetDirection;
		}
	}

	//
	// set the orientation
	//
	newForward = normalize(newForward);
	_forward = newForward;
	_rightSide = rightSideInXZPlane(newForward);

	// This next line is specific to command-based steering, but is not physically based.
	// everything else in command-based steering, however, is physcially based.
	_velocity = newForward * _currentSpeed;


	//
	// choose the force of the agent.  In command-based mode, the force is always aligned 
	// with the agent's forward facing direction, so we can use scalars until we add 
	// side-to-side scoot at the end.
	//
	assert(fabsf(_finalSteeringCommand.acceleration) <= 1.0f); // -1.0f <= acceleration <= 1.0f;
	if (!_finalSteeringCommand.aimForTargetSpeed) {
		// simple "speed up" or "slow down"
		totalSteeringForce = _maxForce * _finalSteeringCommand.acceleration * _forward;
	}
	else {
		// accelerate towards a target speed
		// do it the naive greedy way;
		//
		// the most force you can apply without making velocity direction flip:
		// (force / mass) * time-step = delta-speed
		// if delta-speed == -speed
		// force * mass * time-step = -speed
		//
		//float maxBackwardsForce = (-PED_BRAKING_RATE * fabsf(_currentSpeed) * _mass / _dt);
		float scalarForce = (_finalSteeringCommand.targetSpeed - _currentSpeed) * 8.0f; // crudely trying to make accelerations quicker...
		if (scalarForce > _maxForce) scalarForce = _maxForce;
		//if (force < maxBackwardsForce) force = maxBackwardsForce;
		totalSteeringForce = scalarForce * _forward; // forward is a unit vector, normalized during turning just above.
	}

	// TODO: should we clamp scoot?
	// add the side-to-side motion to the planned steering force.
	totalSteeringForce = totalSteeringForce + ped_scoot_rate * _finalSteeringCommand.scoot * _rightSide;

	//doEulerStepWithForce(totalSteeringForce);
	_doEulerStep(totalSteeringForce, dt);

}


// MUBBASIR -- THIS IS STEERSUITE CODE 
void EgocentricAgent::_doEulerStep(const Util::Vector & steeringDecisionForce, float dt)
{

	// compute acceleration, _velocity, and newPosition by a simple Euler step
	const Util::Vector clippedForce = Util::clamp(steeringDecisionForce, MAX_FORCE_MAGNITUDE);
	Util::Vector acceleration = (clippedForce / AGENT_MASS);
	_velocity = _velocity + (dt*acceleration);
	_velocity = clamp(_velocity, MAX_SPEED);  // clamp _velocity to the max speed

	// MUBBASIR --- ADDING SETTING OF CURRENT SPEED
	_currentSpeed = _velocity.length ();
	if ( _currentSpeed != _currentSpeed)
	{
		throw GenericException("Something happended to the current speed");
	}

	const Util::Point newPosition = _position + (dt*_velocity);

	// For this simple agent, we just make the orientation point along the agent's current velocity.
	if (_velocity.lengthSquared() != 0.0f) {
		_forward = normalize(_velocity);
	}

	// update the database with the new agent's setup
	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	Util::AxisAlignedBox newBounds(newPosition.x - _radius, newPosition.x + _radius, 0.0f, 0.0f, newPosition.z - _radius, newPosition.z + _radius);
	if ( ( newBounds.xmin != newBounds.xmin ) || (newBounds.xmax != newBounds.xmax) || (newBounds.zmin != newBounds.zmin) ||
				(newBounds.zmax != newBounds.zmax))
	{
		throw GenericException("Invalid agent bounds. Bounds are NaN");
	}
	gSpatialDatabase->updateObject( dynamic_cast<SpatialDatabaseItemPtr>(this), oldBounds, newBounds);

	_position = newPosition;
}





//
// reachedWaypoint()
//
bool EgocentricAgent::reachedWaypoint ()
{
	//return false;
	return ( (_waypoints[_currentWaypointIndex]-_position).lengthSquared() < (ego_ped_reached_target_distance_threshold * ego_ped_reached_target_distance_threshold) );
}

//
// reachedCurrentGoal()
//
bool EgocentricAgent::reachedCurrentGoal()
{
	return ( (_currentGoal.targetLocation - _position ).lengthSquared() < (ego_ped_reached_target_distance_threshold * ego_ped_reached_target_distance_threshold) );
}


bool EgocentricAgent::reachedCurrentWaypoint()
{
	return this->reachedWaypoint();
}

//
// reachedLocalTarget()
//
bool EgocentricAgent::reachedLocalTarget()
{
	return ( (_localTargetLocation - _position ).lengthSquared() < (ego_ped_reached_target_distance_threshold * ego_ped_reached_target_distance_threshold) );
}

void EgocentricAgent::runCognitivePhase()
{
	//std::cout << " goal queue size " << _goalQueue.size () << "\n";

	//std::cout << "Egocentric agent currently only supports point goals \n";
	assert(_goalQueue.front().targetLocation == _currentGoal.targetLocation);

	// pop off the previous goal
	_goalQueue.pop();

	if (_goalQueue.empty()) {
		// nowhere left to steer, this pedestrian is done.
		disable();
		return;
	}
	else {
		_currentGoal = _goalQueue.front();
	}

	//runLongTermPlanningPhase ();
}

void EgocentricAgent::disable()
{
	//std::cout << "Disabling .. \n";
	// if we tried to disable a second time, most likely we accidentally ignored that it was disabled, and should catch that error.
	assert(_enabled==true);  

	//  1. remove from database
	Util::AxisAlignedBox b = Util::AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	//gSpatialDatabase->removeObject(dynamic_cast<SpatialDatabaseItem*>(this), b);
	gSpatialDatabase->removeObject( dynamic_cast<SpatialDatabaseItemPtr>(this), b);

	//  2. set enabled = false
	_enabled = false;
}

Point 
EgocentricAgent::goal ()
{

	// no planning
	//if ( reachedCurrentGoal() )
	//	_enabled = false;
	//return _currentGoal.targetLocation;

	// previous
	//if ( !reachedWaypoint() )
	//	return _waypoints[_currentWaypointIndex];
	//else
	//{
	//	if ( _currentWaypointIndex == _waypoints.size() -1 )
	//	{
	//		//std::cerr << "EgocentricAgent::goal : Reached end of waypoint list without runCognitive phase being called \n";
	//		//runCognitivePhase();
	//		//return goal();
	//		_enabled = false;
	//		return Point ( -1.0, -1.0, -1.0);
	//	}
	//	else
	//	{
	//		return _waypoints[ ++ _currentWaypointIndex ];
	//	}
	//}


	 //if you are using all the phases.
	if ( reachedLocalTarget() )
	{
		runShortTermPlanningPhase();
		/* this guy will in turn call 
		runMid -> runLong -> runCog as it deems necessary
		*/
	}
	return _localTargetLocation;
	
}



/*
void queryDatabase(int level, float startAngle, float endAngle)
This function is used to query the "entire area" of the egocentric map.

a) the level limits the amount of area to be queried && the number of neurons to be checked. 
b) the angle limits the number of neurons to be checked. 
   270
	
180	r	0

	90
*/

/*TODO:: 
Specify level limit upto which area is limited.
Specify angle
Specify TOP HALF  --> angle of 180 degrees
*/

void 
EgocentricAgent::queryDatabase(int level, float startAngle, float endAngle)
{

	/*
	Step a)
	Estimate the area = square with radius = R[numberOfLevels - 1] + r[numberOfLevels - 1]
	*/

	//Place cap on level
	if (level > _esm->numberOfLevels) level = _esm->numberOfLevels;

	float a = this->_esm->R[level - 1] + this->_esm->r[level - 1];
	float xCenter =this->_esm->root->centerPosition.x;
	float zCenter = this->_esm->root->centerPosition.z;

	/*
	Step b)
	Estimate the arguments to be given to :
	void collectEntriesInRange(std::vector<DBEntryPtr> & neighborList, float xLowBound, float xHiBound, float zLowBound, float zHiBound, DBEntry * exclude);
	*/

	float xLowBound = xCenter - a;
	float xHiBound = xCenter + a;
	float zLowBound = zCenter - a;
	//float zLowBound = zCenter; when you want it just to look in the front!!
	float zHiBound = zCenter + a ;

	//printf("xl : %f xhi : %f zlo : %f zhi : %f\n", xLowBound,xHiBound,zLowBound,zHiBound);
	
	/*
	Step c)
	Perform Query
	*/

	neighbors.clear();
	//gSpatialDatabase->getItemsInRange( neighbors, xLowBound, xHiBound, zLowBound, zHiBound, this);
	gSpatialDatabase->getItemsInVisualField ( neighbors, xLowBound, xHiBound, zLowBound, zHiBound, this,
		_position,_forward, a*a );


	/*
	Step d)
	Find the neuron range that is to be checked.
	*/

	float stepAngle = 360.0f / _esm->numberOfNeuronsPerLevel;
	int startNeuronId = int ( startAngle / stepAngle);
	int endNeuronId = int ( endAngle / stepAngle);
	

	/*
	Step e)
	Clear all relevant traversabilityRatios 
	*/

	int j = startNeuronId ;
			
	//RESETTING RELEVANT traversabilityRatios first!! 
	//TODO:: SHOULD WE ADD NEW VARIABLE FOR PEDS?
	//TODO:: WE SHOUD RESET THEM FIRST.
	//TODO:: WHAT HAPPENS TO THE ONES WE DONT REFRESH? 
	do
	{
		for(int k = 0; k < level ; k++) 
		{
			_esm->neuron[k*_esm->numberOfNeuronsPerLevel + j]->traversabilityRatio = 1.0;
		}
		j = (j+1)%_esm->numberOfNeuronsPerLevel;

	}
	while(j !=endNeuronId%_esm->numberOfNeuronsPerLevel);

	/*
	Step f)
	Scan through all neighbors and place the information in appropriate neuron.
	*/

	std::set<SteerLib::SpatialDatabaseItem *>::iterator it;

	Point myPosition = this->position();
	Vector myForward = this->forward();

	for (it=neighbors.begin() ; it != neighbors.end(); it++) 
	{
		if ((*it)->isAgent() == true ) 
		{
			EgocentricAgent * agent = dynamic_cast<EgocentricAgent *> ((*it));

			Point positionOfObject = agent->position() ;
			Vector forwardOfObject = agent->forward() ;

			float dotProduct = dot(myForward,forwardOfObject);
			if (dotProduct < 0.0 || (dotProduct < 0.2 && dotProduct > -0.2) ) // the second condn is the perpendicular condn 
			//else if (dot < 0.0 || (dot < 0.4 && dot > -0.4) )
			{
				// OPPOSITE [ AND APPARENTLY PERPENDICULAR]
				//printf("< 0 %f\n",dot);
				///printf("my pos : %f %f \n", myPosition.x, myPosition.z);
				//printf("pos of obj: %f %f \n", positionOfObject.x, positionOfObject.z);

				
			int neuronId = _esm->getNeuronIdFromPositionVector(positionOfObject, false);
			// not sure if i should use numberOfPedestrains?? 

			//first introducing areaOfPedestrian into the traversability Ratio.
			do
			{
				for(int k = 0; k < level ; k++)
				{	

					//Assuming a ped is a square with center: positionOfObject and radius : 0.5


					
					
					float areaOfClippedPedestrian = 
						_esm->neuron[k*_esm->numberOfNeuronsPerLevel + j]->getIntersectedArea
						(positionOfObject.x - pedestrian_radius,positionOfObject.x + pedestrian_radius,
						 positionOfObject.z - pedestrian_radius,positionOfObject.z + pedestrian_radius, _esm->r[k]);

					//printf("area ; %f\n",areaOfClippedBuilding);

					_esm->neuron[k*_esm->numberOfNeuronsPerLevel + j]->traversabilityRatio = 
						_esm->neuron[k*_esm->numberOfNeuronsPerLevel + j]->traversabilityRatio -
																  areaOfClippedPedestrian / pow(2 * _esm->r[k],2);
					
					// you  inhibit the posn of the ped and TO ITS RIGHT AS WELL.
					// to make you go left!!
					// reducing the traversability ratio of the adjacent neuron as well :: '+1' 
					_esm->neuron[(k*_esm->numberOfNeuronsPerLevel + j + 1) % _esm->numberOfNeuronsPerLevel]->traversabilityRatio = 
						_esm->neuron[(k*_esm->numberOfNeuronsPerLevel + j+ 1) % _esm->numberOfNeuronsPerLevel]->traversabilityRatio -
																  areaOfClippedPedestrian / pow(2 * _esm->r[k],2);

				

					//printf("tR ; %f\n",e->neuron[k*e->numberOfNeuronsPerLevel + j]->traversabilityRatio);

				}
				j = (j+1)%_esm->numberOfNeuronsPerLevel;

			}
			while(j !=endNeuronId%_esm->numberOfNeuronsPerLevel);

			}



		}
		else 
		{
			SteerLib::ObstacleInterface *building = (SteerLib::ObstacleInterface *) (*it);
			Util::AxisAlignedBox bounds = building->getBounds();
			
			// TODO - Confirm that this is correct.
			float xlow = bounds.xmin - pedestrian_static_object_comfort_zone;
			float xhigh = bounds.xmax + pedestrian_static_object_comfort_zone;
			float zlow = bounds.zmin - pedestrian_static_object_comfort_zone;
			float zhigh = bounds.zmax + pedestrian_static_object_comfort_zone;

			j = startNeuronId;


			do
			{
				for(int k = 0; k < level ; k++) // try using forward pointer 'level' times instead. 
				{	


					float areaOfClippedBuilding = 
						_esm->neuron[k*_esm->numberOfNeuronsPerLevel + j]->getIntersectedArea
						(xlow,xhigh,zlow,zhigh, _esm->r[k]);

					//printf("area ; %f\n",areaOfClippedBuilding);

					_esm->neuron[k*_esm->numberOfNeuronsPerLevel + j]->traversabilityRatio = 
						_esm->neuron[k*_esm->numberOfNeuronsPerLevel + j]->traversabilityRatio -
																  areaOfClippedBuilding / pow(2 * _esm->r[k],2);


				}
				j = (j+1)%_esm->numberOfNeuronsPerLevel;

			}
			while(j !=endNeuronId%_esm->numberOfNeuronsPerLevel);


		}
		

	}
	

}

void 
EgocentricAgent::evaluateDynamicObjects(int level, float startAngle, float endAngle)
{
	float a = this->_esm->R[level - 1] + this->_esm->r[level - 1];
	float xCenter =this->_esm->root->centerPosition.x;
	float zCenter = this->_esm->root->centerPosition.z;

	float xLowBound = xCenter - a;
	float xHiBound = xCenter + a;
	float zLowBound = zCenter - a;
	//float zLowBound = zCenter;
	float zHiBound = zCenter + a ;

	neighbors.clear();
	gSpatialDatabase->getItemsInRange( neighbors, xLowBound, xHiBound, zLowBound, zHiBound,  this );


	// TODO: COMMENTED OUT THESE VARIABLES BECAUSE THEY WERE CURRENTLY NOT USED
	//float stepAngle = 360.0f / _esm->numberOfNeuronsPerLevel;
	//int startNeuronId = int ( startAngle / stepAngle);
	//int endNeuronId = int ( endAngle / stepAngle);
	//int j = startNeuronId ;
	std::set<SteerLib::SpatialDatabaseItem *>::iterator it;

	Point myPosition = this->position();
	Vector myForward = this->forward();

	for (it=neighbors.begin() ; it != neighbors.end(); it++) 
	{
		
		if ((*it)->isAgent() == true) 
		{

			// first the area of the ped itself!  -- RIGHT NOW IN QUERY DATABASE.
			EgocentricAgent * agent = dynamic_cast<EgocentricAgent *> (*(it));


			
			// this is the first level of my neighbor.
			for(int l = 0 ; l < agent->_esm->numberOfNeuronsPerLevel ; l++)
			{
				// the first level of considerably activated neurons must serve as inhibition for ME.

				float xlow = agent->_esm->neuron[l]->centerPosition.x -
							agent->_esm->r[l / agent->_esm->numberOfNeuronsPerLevel];

				float xhigh = agent->_esm->neuron[l]->centerPosition.x +
							agent->_esm->r[l / agent->_esm->numberOfNeuronsPerLevel];

				float zlow = agent->_esm->neuron[l]->centerPosition.z -
							agent->_esm->r[l / agent->_esm->numberOfNeuronsPerLevel];

				float zhigh = agent->_esm->neuron[l]->centerPosition.z +
								 agent->_esm->r[l /agent->_esm->numberOfNeuronsPerLevel];

				float radius = agent->_esm->r[l / agent->_esm->numberOfNeuronsPerLevel];

				for(int a = 0 ; a <  _esm->numberOfNeuronsPerLevel * 2 ; a++)
				{
						//printf("I am here in evaluateDynamicObjects \n");
					float areaOfClippedNeuron = 
					_esm->neuron[a]->getIntersectedArea(xlow,xhigh,zlow,zhigh, _esm->r[a/_esm->numberOfNeuronsPerLevel]);

					_esm->neuron[a]->activationPed -= /*e->neuron[a]->activationPed **/ areaOfClippedNeuron 
												/ pow(2*radius,2);

					//e->neuron[a]->activation -= PEDESTRIAN_PTR(neighbors[i])->e->neuron[l]->activation * areaOfClippedNeuron 
					//							/ pow(2*radius,2);
					
					if(_esm->neuron[a]->activationPed < 0.0 ) _esm->neuron[a]->activationPed = 0.0;


					//CHECK THIS -> REVERT BACK 
					/*e->neuron[a]->activation -= e->neuron[a]->activation * pow(2*radius,2) 
												/ pow(2*e->r[a/e->numberOfNeuronsPerLevel],2);*/

				}



			}
			// these are the remaining levels. 
			for(int l = 1 ; l < level ; l ++)
			{

				int neuronId = agent->_esm->determineNeuronIdWithMaximumActivationAtLevel(l);
				// we found the neuron id with max activation at level 'l'.
				if(neuronId != -1)
				{
					// now we need to make neuronId's activation serve as inhibition on our map.
					float xlow = agent->_esm->neuron[neuronId]->centerPosition.x -
								 agent->_esm->r[l];

					float xhigh = agent->_esm->neuron[neuronId]->centerPosition.x +
								 agent->_esm->r[l];

					float zlow = agent->_esm->neuron[neuronId]->centerPosition.z -
								 agent->_esm->r[l];

					float zhigh = agent->_esm->neuron[neuronId]->centerPosition.z +
								 agent->_esm->r[l];

					float radius = agent->_esm->r[l];


					for(int a = 0 ; a < level*_esm->numberOfNeuronsPerLevel ; a++)
					{
						//printf("I am here in evaluateDynamicObjects \n");
						float areaOfClippedNeuron = 
						_esm->neuron[a]->getIntersectedArea(xlow,xhigh,zlow,zhigh, _esm->r[a/_esm->numberOfNeuronsPerLevel]);

						//e->neuron[a]->activation -= e->neuron[a]->activation * areaOfClippedNeuron 
						//							/ pow(2*radius,2);

						_esm->neuron[a]->activationPed -=  /*e->neuron[a]->activationPed 
							**/areaOfClippedNeuron / pow(2*radius,2);
					}

				}

			}

		}
	}



}



