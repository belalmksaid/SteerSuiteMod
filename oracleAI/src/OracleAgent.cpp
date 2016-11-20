//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#include "astar/AStarLite.h"
#include "oracleAI/GridAStar.h"
#include "oracleAI/OracleAIModule.h"
#include "oracleAI/OracleAgent.h"
#include "oracleAI/OracleEnvironment.h"
#include "SteerLib.h"
#include <math.h>

using namespace Util;
using namespace SteerLib;
using namespace OracleGlobals;



#define IN_INTERVAL_STRICT(X,A,B) (((X)>(A)) && (X)<(B))
#define IN_INTERVAL_INCLUSIVE(X,A,B) (((X)>=(A)) && (X)<=(B))
#define IGNORE_PLANNING


float gTempCurrentTime;

inline void evaluateParabolaAtTime(const Footstep & step, float currentTime, Point & position, Vector & velocity)
{
	// TODO, even if this is "technically" a better thing to do, it made the AI work worse because everyone gets more stuck
	// behind each other.  (instead of the parabola eventually be evaluated outrageously away from everything else)
	// the RIGHT solution is to keep this and fix other issues.
	if (currentTime > step.endTime) currentTime = step.endTime;

	float currentLocalStepTime;
	switch (step.state) {
		case FOOTSTEP_STATE_NORMAL:
			currentLocalStepTime = (currentTime - step.startTime) - (0.5f*(step.endTime - step.startTime));
			break;
		case FOOTSTEP_STATE_STOPPING:
			currentLocalStepTime = (currentTime - step.startTime) - (step.endTime - step.startTime);
			break;
		case FOOTSTEP_STATE_STARTING:
			currentLocalStepTime = (currentTime - step.startTime);
			break;
		case FOOTSTEP_STATE_STATIONARY:
			position = Point(step.outputCOMState.x, 0.0f, step.outputCOMState.z);
			//velocity = Vector(1.0f, 0.0f, 0.0f);
			velocity = Vector(step.outputCOMState.dx, 0.0f, step.outputCOMState.dz);
			return;
			break;
		default:
			std::cerr << "ERROR: trying to evaluate parabola of a step that has an invalid state!\n";
			assert(false);
	}

	float localX = step.simulationDx * currentLocalStepTime;
	float localZ = step.simulationA * currentLocalStepTime * currentLocalStepTime;
	float localDx = step.simulationDx;
	float localDz = 2.0f * step.simulationA * currentLocalStepTime;
	float worldX = step.parabolaX + step.simulationIx * localX  +  step.simulationJx * localZ;
	float worldZ = step.parabolaZ + step.simulationIz * localX  +  step.simulationJz * localZ;
	float worldDx = step.simulationIx * localDx  +  step.simulationJx * localDz;
	float worldDz = step.simulationIz * localDx  +  step.simulationJz * localDz;
	position.x = worldX;
	position.y = 0.0f;
	position.z = worldZ;
	velocity.x = worldDx;
	velocity.y = 0.0f;
	velocity.z = worldDz;
}


inline void drawFoot(float x, float z, float orientation, bool whichFoot) {
	glPushAttrib(GL_CURRENT_BIT);
	glPushMatrix();

	if (whichFoot==LEFT_FOOT) {
		DrawLib::glColor(gYellow);
	}
	else {
		DrawLib::glColor(gCyan);
	}

	glTranslatef( x, 0.0f, z );
	glRotatef( -orientation * M_180_OVER_PI, 0.0f, 1.0f, 0.0f );
	//glScalef(0.05f, 0.05f, 0.05f);
	//DrawLib::drawCube();
	glScalef(0.2f, 0.05f, 0.07f);
	DrawLib::drawCube();
	glTranslatef(0.2f, 0.0f, (whichFoot==LEFT_FOOT) ? -0.1f : 0.1f);
	DrawLib::drawSphere();
	glPopMatrix();
	glPopAttrib();
}



inline void drawFootstepTrajectory(const Footstep & step, const Util::Color & color) {

	float increment = 0.01f * (step.endTime - step.startTime);

	if (increment < 0.00001f) {
		std::cerr << "WARNING - footstep had invalid time interval!\n";
		std::cerr << "problematic step was:" << step;
		return;
	}

	glPushAttrib(GL_CURRENT_BIT);
	glPushMatrix();

	DrawLib::glColor(color);

	glBegin(GL_LINE_STRIP);
	for (float t = step.startTime;  t <= step.endTime; t += increment) {
		Point COMPos;
		Vector COMVel;
		evaluateParabolaAtTime(step, t, COMPos, COMVel);
		glVertex3f( COMPos.x, 0.0f, COMPos.z);
	}
	glEnd();

	glPopMatrix();
	glPopAttrib();
}

inline float determineFootstepOrientation(const Footstep & currentStep, const Footstep & previousStep) 
{
	float currentPhi;
	if (currentStep.whichFoot==LEFT_FOOT) {
		// outer > inner
		if (previousStep.simulationA > 1.7f) {
			currentPhi = currentStep.outerFootOrientationPhi;
		}
		else if ( IN_INTERVAL_INCLUSIVE(currentStep.parabolaOrientationPhi, currentStep.innerFootOrientationPhi, currentStep.outerFootOrientationPhi)) {
			currentPhi = currentStep.parabolaOrientationPhi;
		} else {
			currentPhi = currentStep.innerFootOrientationPhi;
		}
	}
	else {
		// inner > outer
		if (previousStep.simulationA > 1.7f) {
			currentPhi = currentStep.outerFootOrientationPhi;
		}
		else if ( IN_INTERVAL_INCLUSIVE(currentStep.parabolaOrientationPhi, currentStep.outerFootOrientationPhi, currentStep.innerFootOrientationPhi)) {
			currentPhi = currentStep.parabolaOrientationPhi;
		} else {
			currentPhi = currentStep.innerFootOrientationPhi;
		}
	}
	return currentPhi;
}

OracleAgent::OracleAgent()
{
	_enabled = false;
	cout << "Size of a single Footstep structure: " << sizeof(Footstep) << endl;
	cout.flush();
}

OracleAgent::~OracleAgent()
{
}

void OracleAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	//
	//
	//  first, initialize  _walkParameters
	//
	//

	_walkParameters.centerOfMassHeight = DEFAULT_COM_HEIGHT;
	_walkParameters.centerOfMassHeightInverse = 1.0f / DEFAULT_COM_HEIGHT;
	_walkParameters.mass = DEFAULT_MASS;
	_walkParameters.massInverse = 1.0f/DEFAULT_MASS;
	_walkParameters.minStepLength = DEFAULT_MIN_STEP_LENGTH;
	_walkParameters.maxStepLength = DEFAULT_MAX_STEP_LENGTH;
	_walkParameters.minStepTime = DEFAULT_MIN_STEP_TIME;
	_walkParameters.maxStepTime = DEFAULT_MAX_STEP_TIME;
	_walkParameters.maxSpeed = DEFAULT_MAX_SPEED;
	_walkParameters.baseRadius = DEFAULT_BASE_RADIUS;
	_walkParameters.preferredStepLength = 2.0f * _walkParameters.centerOfMassHeight * sinf(PREFERRED_STEP_ANGLE);

	_walkParameters.timeCostWeight = DEFAULT_TIME_COST_WEIGHT;
	_walkParameters.trajectoryCostWeight = DEFAULT_TRAJECTORY_COST_WEGHT;

	// check if the stride interval includes the "optimal" step angle, otherwise print a warning
	if ( (_walkParameters.preferredStepLength < _walkParameters.minStepLength) || (_walkParameters.preferredStepLength > _walkParameters.maxStepLength)) {
		std::cerr << "###############################################\n";
		std::cerr << "# WARNING: \"optimal\" stride length is outside\n";
		std::cerr << "# of the user-defined min/max step interval:\n";
		std::cerr << "# preferred: " << _walkParameters.preferredStepLength << ",  interval: (" << _walkParameters.minStepLength  << ", " << _walkParameters.maxStepLength << ")\n";
		std::cerr << "###############################################\n";
	}

	// compute the energy cost of an "ideal step" that maintains max velocity with max stride.
	// at every step velocity is scaled by cos( 2 theta ), where theta is the angle associated with the inverted pendulum
	// then, energy is essentially 0.5 * m * (vnew^2 - vold^2).
	//float speedAfterLoss = _walkParameters.maxSpeed * cosf( 2.0f * asinf(0.5f * _walkParameters.maxStepLength/_walkParameters.centerOfMassHeight));

	float speedAfterLoss = _walkParameters.maxSpeed * cosf( 2.0f * PREFERRED_STEP_ANGLE);
	_walkParameters.heuristicEnergyCostPerStep = (_walkParameters.timeCostWeight * 0.6f) + (0.5f * _walkParameters.mass * fabsf((_walkParameters.maxSpeed*_walkParameters.maxSpeed) - (speedAfterLoss*speedAfterLoss)));


	// Furthermore, the cost of a step is the total work done by the center of mass.
	// 0.17 is an estimated "minimum" if the quadratic parameter "a". when going in approximately a straight line
	_walkParameters.heuristicEnergyCostPerStep += _walkParameters.trajectoryCostWeight * 2.0f * _walkParameters.mass * 0.17f * _walkParameters.preferredStepLength;



	//
	//
	//  then, initialize steersuite agent stuff
	//
	//

	AxisAlignedBox oldBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
	_position = initialConditions.position;
	_forward  = initialConditions.direction;
	_radius   = PED_TORSO_RADIUS;  //initialConditions.radius;
	AxisAlignedBox newBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);

	if (!_enabled) {
		gSpatialDatabase->addObject( dynamic_cast<SpatialDatabaseItemPtr>(this), newBounds);
	}
	else {
		gSpatialDatabase->updateObject( dynamic_cast<SpatialDatabaseItemPtr>(this), oldBounds, newBounds);
	}
	_enabled = true;


	clearGoals();
	for (unsigned int i=0; i<initialConditions.goals.size(); i++) {
		if (initialConditions.goals[0].goalType != GOAL_TYPE_SEEK_STATIC_TARGET) {
			throw Util::GenericException("Currently the phase decimation agent does not support goal types other than GOAL_TYPE_SEEK_STATIC_TARGET.");
		}
		addGoal(initialConditions.goals[i]);
	}


	assert(_forward.length() >= 0.00001f);
	assert(_goalQueue.size() != 0);
	// this assertion does not work with the new AgentGoalInfo struct; probably beacuse there is no == operator?
	// assert(_goalQueue.front() == _currentGoal);
	assert(_radius >= 0.00001f);

	_waypoints.clear();
	_currentWaypointIndex = 0;
	_midTermPathSize = 0;
	_localTargetLocation = _currentGoal.targetLocation;
	//_neighbors.clear();
	//_numAgentsInVisualField = 0;

	//======================================

	//
	//
	//  initialize the _currentStep (and _previousStep)
	//
	//

	_currentStep.targetX = _localTargetLocation.x;
	_currentStep.targetZ = _localTargetLocation.z;
		
	_currentStep.whichFoot = LEFT_FOOT;
	_currentStep.startTime = 0.0f;
	MTRand mtr((unsigned int)(initialConditions.position.x*initialConditions.position.z));
	_currentStep.endTime = (float)mtr.rand() * PED_INITIAL_STEP_VARIATION + 0.1f;
	//_currentStep.parabolaOrientationPhi = 0.0f * M_PI_OVER_180;
	//_currentStep.innerFootOrientationPhi = 0.0f * M_PI_OVER_180;
	//_currentStep.outerFootOrientationPhi = 0.0f * M_PI_OVER_180;
	_currentStep.parabolaOrientationPhi = atan2( normalize(initialConditions.direction).z, normalize(initialConditions.direction).x );
	_currentStep.innerFootOrientationPhi = atan2( normalize(initialConditions.direction).z, normalize(initialConditions.direction).x );
	_currentStep.outerFootOrientationPhi = atan2( normalize(initialConditions.direction).z, normalize(initialConditions.direction).x );
	_currentStep.parabolaX = initialConditions.position.x;
	_currentStep.parabolaZ = initialConditions.position.z;
	_currentStep.footX = initialConditions.position.x;
	_currentStep.footZ = initialConditions.position.z;
	_currentStep.energyCost = 0.0f;
	_currentStep.outputCOMState.x = initialConditions.position.x;
	_currentStep.outputCOMState.z = initialConditions.position.z;
	_currentStep.outputCOMState.dx = (1.0f) * cos(_currentStep.parabolaOrientationPhi) + (.1f) * sin(_currentStep.parabolaOrientationPhi); 
	_currentStep.outputCOMState.dz = (-.1f) * cos(_currentStep.parabolaOrientationPhi) + (1.0f) * sin(_currentStep.parabolaOrientationPhi); 
	_currentStep.isAGoalState = false;
	_currentStep.state = FOOTSTEP_STATE_NORMAL;
	_currentStep.phiIsIdeal = false;
	// TODO: should eventually ideally start stationary.
	//_currentStep.state = FOOTSTEP_STATE_STATIONARY;
	float timeDurationToOrigin, a, localDz, localDx, jx, jz, stepDuration, ix, iz;

	stepDuration = _currentStep.endTime - _currentStep.startTime;
	
	ix = cos(_currentStep.parabolaOrientationPhi);
	iz = sin(_currentStep.parabolaOrientationPhi);

	jx = iz;
	jz = -ix;
	
	localDz = jx * _currentStep.outputCOMState.dx + jz * _currentStep.outputCOMState.dz;
	localDx = ix * _currentStep.outputCOMState.dx + iz * _currentStep.outputCOMState.dz;
	
	timeDurationToOrigin = stepDuration * 0.5f;
	a = localDz / (2.0f * -timeDurationToOrigin);

	_currentStep.simulationA  = a;
	_currentStep.simulationDx = localDx;
	_currentStep.simulationIx = ix;
	_currentStep.simulationIz = iz;
	_currentStep.simulationJx = jx;
	_currentStep.simulationJz = jz;
	
	_previousStep = _currentStep;

	_currentSimulatedPosition = Point(_currentStep.outputCOMState.x, 0.0f, _currentStep.outputCOMState.z);
	_currentSimulatedVelocity = Vector(_currentStep.outputCOMState.dx, 0.0f, _currentStep.outputCOMState.dz);

	_currentLocationInPath = 0;
	_footstepPlan.clear();
	_needToRunFootstepPlanner = true;
	planAge = 0;


	/*
	// THE FOLLOWING OVERRIDES TEST CASE INPUTS
	// JUST FOR TESTING

	clearGoals();

	SteerLib::AgentGoalInfo nextGoal;
	//nextGoal.targetLocation.x = 3.0f;
	//nextGoal.targetLocation.y = 0.0f;
	//nextGoal.targetLocation.z = -5.0f;
	//_goalQueue.push(nextGoal);
	//nextGoal.targetLocation.x = 6.0f;
	//nextGoal.targetLocation.y = 0.0f;
	//nextGoal.targetLocation.z = -5.0f;
	//_goalQueue.push(nextGoal);
	//nextGoal.targetLocation.x = 6.0f;
	//nextGoal.targetLocation.y = 0.0f;
	//nextGoal.targetLocation.z = 0.0f;
	//_goalQueue.push(nextGoal);
	//nextGoal.targetLocation.x = 9.0f;
	//nextGoal.targetLocation.y = 0.0f;
	//nextGoal.targetLocation.z = 0.0f;
	//_goalQueue.push(nextGoal);

	nextGoal.targetLocation.x = 3.0f;
	nextGoal.targetLocation.y = 0.0f;
	nextGoal.targetLocation.z = 0.0f;
	_goalQueue.push(nextGoal);
	nextGoal.targetLocation.x = -3.0f;
	nextGoal.targetLocation.y = 0.0f;
	nextGoal.targetLocation.z = -5.0f;
	_goalQueue.push(nextGoal);
	nextGoal.targetLocation.x = 3.0f;
	nextGoal.targetLocation.y = 0.0f;
	nextGoal.targetLocation.z = -5.0f;
	_goalQueue.push(nextGoal);
	nextGoal.targetLocation.x = 3.0f;
	nextGoal.targetLocation.y = 0.0f;
	nextGoal.targetLocation.z = 5.0f;
	_goalQueue.push(nextGoal);
	nextGoal.targetLocation.x = -3.0f;
	nextGoal.targetLocation.y = 0.0f;
	nextGoal.targetLocation.z = 5.0f;
	_goalQueue.push(nextGoal);

	_currentGoal = _goalQueue.front();
	_localTargetLocation = _currentGoal.targetLocation;
	*/

}



bool OracleAgent::_createFootstepAction(float stepDuration, float parabolaOrientationPhi, bool phiIsIdeal, float desiredVelocity, const Footstep & previousStep, Footstep & nextStep, FootStateEnum nextState)
{
	nextStep.phiIsIdeal = phiIsIdeal;
	nextStep.desiredSpeed = desiredVelocity;
	//float startTime;
	//float endTime;
	//float parabolaX, parabolaZ;
	//float parabolaOrientationPhi;
	//bool whichFoot;
	//float energyCost;
	//DynamicState outputCOMState;
	//float simulationA, simulationDx, simulationIx, simulationIz, simulationJx, simulationJz;
	//float footX, footZ;
	//bool isAGoalState;
	//FootStateEnum state;
	//
	//float innerFootOrientationPhi;
	//float outerFootOrientationPhi;
	//
	//
	// to create a starting step:
	//   - start time is prev step start time
	//   - end time is   start + stepDuration, BUT ONLY LASTS FOR HALF THE USUAL PARABOLA
	//   - parabolaX/Z is the location of the foot now
	//   - orientationPhi can be anything that it usually is - probably more options
	//   - inner/outer would be same as before
	//
	// to create a stopping step:
	// to create a stationary step:
	//    start time is previous
	//    end time is previous + duration
	//    parabola X Y does not change
	//    foot X Y can change, but not necessary
	//    parabolaphi is meaningless
	//    
	//    
	//

	//std::cerr << "=========================================\n";
	//std::cerr << "CREATING A FOOTSTEP:\n";
	//std::cerr << "=========================================\n";
	//std::cerr << "PREVIOUS STEP:\n    " << previousStep << "\n";
	//std::cerr << "INPUT OPTIONS:\n" << "  stepDuration = " << stepDuration << "\n  parabolaOrientationPhi = " << parabolaOrientationPhi << "\n  desiredVelocity = " << desiredVelocity << "\n\n";


	if (nextState == FOOTSTEP_STATE_STATIONARY) {
		assert((previousStep.state == FOOTSTEP_STATE_STATIONARY)||(previousStep.state == FOOTSTEP_STATE_STOPPING));
		nextStep = previousStep;

		nextStep.startTime = previousStep.endTime;
		nextStep.endTime = nextStep.startTime + stepDuration;
		nextStep.state = FOOTSTEP_STATE_STATIONARY;
		nextStep.energyCost = _walkParameters.timeCostWeight * stepDuration;
		
		//nobody ever checked stationary agents for collisions!
		bool collisionWithAgentFound = false;
		bool collisionWithObstacleFound = false;
		//for (float interp = 0.5f; interp <= 1.0f; interp += 0.49f) {
		//for (float interp = 0.33f; interp <= 1.0f; interp += 0.32f) {
		//for (float interp = 0.20f; interp <= 1.0f; interp += 0.19f) {
		float overlapPenalty;
		for (float interp = 0.01f; interp <= 1.0f; interp += 0.005f) {
			Vector collisionForward;
			Point collisionPosition;
			float collisionTime = nextStep.startTime + stepDuration * interp;
			evaluateParabolaAtTime(nextStep, collisionTime, collisionPosition, collisionForward);
			Vector collisionRightSide = normalize(Vector(collisionForward.z, 0.0f, -collisionForward.x));
			// for every object in the character's visual field...
			for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin();  neighbor != _neighbors.end();  ++neighbor) {
				if ((*neighbor)->isAgent()) {
					// collision procedure for other agents
					OracleAgent * agent;
					agent = dynamic_cast<OracleAgent*>(*neighbor);
				
					// check if the obstacle collides with the character's main body
					bool collisionCheck = agent->collidesAtTimeWith(collisionPosition, collisionRightSide, _radius, collisionTime, nextStep.footX, nextStep.footZ);
					if (collisionCheck) {
						collisionWithAgentFound = true;
						break;
					}

				}
				else {
					// collision procedure for obstacles
					SteerLib::ObstacleInterface * obstacle;
					obstacle = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbor);

					// check if the obstacle collides with the character's main body
					if (obstacle->overlaps(collisionPosition, _radius)) {
						collisionWithObstacleFound = true;
						break;
					}

					// check if the obstacle collides with the character's shoulders.
					// for now, if mustRotateShoulders is already true, we can skip this check.
					if (( obstacle->overlaps(collisionPosition + _radius*collisionRightSide, _radius) || obstacle->overlaps(collisionPosition - _radius*collisionRightSide, _radius))) {
						collisionWithObstacleFound = true;
						break;
					}

					// OLD VERSION - NOTE CAREFULLY THE "!mustRotateShoulders" portion...
					//if ((!mustRotateShoulders)  && ( obstacle->overlaps(collisionPosition + _radius*collisionRightSide, _radius) || obstacle->overlaps(collisionPosition - _radius*collisionRightSide, _radius))) {
					//	mustRotateShoulders = true;
					//}

				}

			}
		}

		if (collisionWithAgentFound) {
			//nextStep.energyCost += 100.0f * _walkParameters.mass * overlapPenalty;
			return false;
		}

		if (collisionWithObstacleFound) {
			//nextStep.energyCost += 1000.0f * _walkParameters.mass;
			return false;
		}
		
		return true;
	}

	if (nextState == FOOTSTEP_STATE_STARTING) {
		nextStep.whichFoot = previousStep.whichFoot;
	}
	else {
		nextStep.whichFoot = !previousStep.whichFoot;
	}

	// 0. compute the energy cost of accelerating to the target speed.
	//    note that at every step, some velocity (i.e. energy) dissipates into the ground.
	//    some of these values are re-used a bit later, and so are made explicit local variables.
	float prevStepDistanceX = previousStep.outputCOMState.x-previousStep.parabolaX;
	float prevStepDistanceZ = previousStep.outputCOMState.z-previousStep.parabolaZ;
	float prevSpeed = sqrtf(previousStep.outputCOMState.dx*previousStep.outputCOMState.dx + previousStep.outputCOMState.dz*previousStep.outputCOMState.dz);
	float stepLength = sqrtf( prevStepDistanceX*prevStepDistanceX + prevStepDistanceZ*prevStepDistanceZ);
	float twoTheta = 2.0f * asinf(stepLength/_walkParameters.centerOfMassHeight);
	float speedAfterLoss = prevSpeed * cosf(twoTheta);
	nextStep.energyCost = (_walkParameters.timeCostWeight*stepDuration) + (0.5f * _walkParameters.mass * fabsf(desiredVelocity*desiredVelocity - speedAfterLoss*speedAfterLoss));

	//std::cerr << "speed after dissipation = (" << speedAfterLoss << ")\n";
	//std::cerr << "energy cost for a step of (previous) length " << stepLength << " is " << nextStep.energyCost << "\n";

	// 1. compute components of the new velocity
	//
	//  FOR A NORMAL STEP:  velocity is the new desired velocity immediately.
	//  FOR A STOPPING STEP:  velocity is the speed after dissipation, and the character does not push.
	//  FOR A STARTING STEP:  velocity technically starts at 0, but to compute the desired parabola 
	//                        properly, this is the "virtual velocity" of the parabola where it is symmetric
	//                        to the output velocity.
	//
	float prevSpeedInverse = 1.0f/prevSpeed;
	float velocityX, velocityZ;
	if (nextState == FOOTSTEP_STATE_NORMAL) {
		velocityX = previousStep.outputCOMState.dx * desiredVelocity * prevSpeedInverse;
		velocityZ = previousStep.outputCOMState.dz * desiredVelocity * prevSpeedInverse;
		//std::cerr << "Normal step, after dissipation and push, before trajectory, the velocity is = (" << velocityX  << ", " << velocityZ << ")\n";
	}
	else if (nextState == FOOTSTEP_STATE_STOPPING) {
		velocityX = previousStep.outputCOMState.dx * speedAfterLoss * prevSpeedInverse;
		velocityZ = previousStep.outputCOMState.dz * speedAfterLoss * prevSpeedInverse;
		//std::cerr << "Stopping step after dissipation (no push), before trajectory, the velocity is = (" << velocityX  << ", " << velocityZ << ")\n";
	}
	else if (nextState == FOOTSTEP_STATE_STARTING) {
		velocityX = previousStep.outputCOMState.dx * desiredVelocity * prevSpeedInverse;
		velocityZ = previousStep.outputCOMState.dz * desiredVelocity * prevSpeedInverse;
		//std::cerr << "Starting step, the VIRTUAL input velocity is = (" << velocityX  << ", " << velocityZ << ")\n";
	}




	// 2. generate bases vectors for orientation based on orientationPhi

	// horizontal basis vector
	float ix = cos(parabolaOrientationPhi);
	float iz = sin(parabolaOrientationPhi);
	
	// vertical basis vector
	float jx, jz;  
	if (nextStep.whichFoot == RIGHT_FOOT) {
		// THIS NEXT STEP WILL BE A RIGHT FOOT
		// normal 2D coordinate system y-axis points up
		jx = -iz;
		jz = ix;
	}
	else {
		// THIS NEXT STEP WILL BE A LEFT FOOT
		// inverted coordinate system, y-axis points down
		jx = iz;
		jz = -ix;
	}


	//std::cerr << "i = (" << ix << ", " << iz << ")\n";
	//std::cerr << "j = (" << jx << ", " << jz << ")\n";

	// 3. transform the velocity into local space
	float localDx = ix * velocityX + iz * velocityZ;
	float localDz = jx * velocityX + jz * velocityZ;


	//std::cerr << "local Velocity at beginning of trajectory (virtual for a starting step) = (" << localDx << ", " << localDz << ")\n";

	if (localDx < 0.0f) {
		// if this condition happens, it may not be a bug or problem, but should be noted for testing purposes.
		//std::cerr << "####  negative local-space localDx = " << localDx << " - IS CHARACTER STEPPING BACKWARDS??\n";
	}


	//if (localDz > 0.0001f) {
	if (localDz > 0.0f) {
		//std::cerr << "####  FootstepAgent::_createFootstepAction() - pendulum swinging in wrong direction (away from foot)!  localDz = " << localDz << "\n";
		return false;
	}



	// 4. solve for the location of the origin of the parabola
	//    this is actually done implicitly by figuring out the localX and localZ of the initial center of mass location.

	float localX, localZ, a, timeDurationToOrigin;
	switch (nextState) {
		case FOOTSTEP_STATE_NORMAL:
			timeDurationToOrigin = stepDuration * 0.5f;
			localX =  - timeDurationToOrigin * localDx;  // x starts on the negative side
			a = localDz / (2.0f * -timeDurationToOrigin);  // NOTE CAREFULLY: the real formula is y = a t^2 -->  dy = 2 a t --> a = dy/2t --> so the half and 2 cancel out.
			localZ = a*timeDurationToOrigin*timeDurationToOrigin;  // technically its (-halfStepDuration) * (-halfStepDuration)
			nextStep.parabolaX = previousStep.outputCOMState.x -  ix * localX  -  jx * localZ;
			nextStep.parabolaZ = previousStep.outputCOMState.z -  iz * localX  -  jz * localZ;
			break;
		case FOOTSTEP_STATE_STOPPING:
			timeDurationToOrigin = stepDuration;
			localX =  - timeDurationToOrigin * localDx;  // x starts on the negative side
			a = localDz / (2.0f * -timeDurationToOrigin);  // NOTE CAREFULLY: the formula is y = a t^2 -->  dy = 2 a t --> a = dy/2t --> so the half and 2 cancel out.
			localZ = a*timeDurationToOrigin*timeDurationToOrigin;  // technically its (-halfStepDuration) * (-halfStepDuration)
			nextStep.parabolaX = previousStep.outputCOMState.x -  ix * localX  -  jx * localZ;
			nextStep.parabolaZ = previousStep.outputCOMState.z -  iz * localX  -  jz * localZ;
			break;
		case FOOTSTEP_STATE_STARTING:
			timeDurationToOrigin = stepDuration;
			localX =  - timeDurationToOrigin * localDx;  // x starts on the negative side
			a = localDz / (2.0f * -timeDurationToOrigin);  // NOTE CAREFULLY: the formula is y = a t^2 -->  dy = 2 a t --> a = dy/2t --> so the half and 2 cancel out.
			localZ = a*timeDurationToOrigin*timeDurationToOrigin;  // technically its (-halfStepDuration) * (-halfStepDuration)
			nextStep.parabolaX = previousStep.parabolaX;
			nextStep.parabolaZ = previousStep.parabolaZ;
			break;
		case FOOTSTEP_STATE_STATIONARY:
			std::cerr << "ERROR: should not get here, should have handled stationary case above?!\n";
			assert(false);
			break;
		default:
			std::cerr << "ERROR: trying to create footstep for a step that has an invalid state!\n";
			assert(false);
	}


	// check if the center-of-mass location is valid
	if (localZ < -0.0001f) {
	//if (localZ < 0.0f) {
		// if this condition happens, it is probably an internal bug.
		std::cerr << "####  OracleAgent::_createFootstepAction() - invalid position condition for this desired step!\n";
		assert(false);
	}


	// add the trajectory's change-in-momentum to the energy cost:
	//   computed as force*distance, where force = mass * acceleration, and both distance and "A" have a 2.0 in front of them.
	// note that "localX * localX + localZ * localZ" is "square of the half distance" and so there is an extra 2.0f factor outside.
	nextStep.energyCost += _walkParameters.trajectoryCostWeight * 4.0f * _walkParameters.mass * a * sqrtf(localX * localX + localZ * localZ);

	//std::cerr << "old local Position = (" << localX << ", " << localZ << ")\n";

	// initialize the parameters of the step that are needed by evaluateParabolaAtTime()
	nextStep.simulationA  = a;
	nextStep.simulationDx = localDx;
	nextStep.simulationIx = ix;
	nextStep.simulationIz = iz;
	nextStep.simulationJx = jx;
	nextStep.simulationJz = jz;
	nextStep.startTime = previousStep.endTime;
	nextStep.endTime = previousStep.endTime + stepDuration;
	nextStep.state = nextState;


	// 5. evaluate the parabola and get the global-space COM state.
	Point COMStateLocation;
	Vector COMStateVelocity;
	evaluateParabolaAtTime(nextStep, nextStep.endTime, COMStateLocation, COMStateVelocity);
	nextStep.outputCOMState.x  = COMStateLocation.x;
	nextStep.outputCOMState.z  = COMStateLocation.z;
	nextStep.outputCOMState.dx = COMStateVelocity.x;
	nextStep.outputCOMState.dz = COMStateVelocity.z;
	
	//std::cerr << "POSITION OF OUTPUT COM IS " << COMStateLocation << "\n";
	//std::cerr << "DIRECTION OF OUTPUT COM IS " << COMStateVelocity << "\n";
	/*
	// OLD CODE:

	//    symmetry of the quadratic makes this beautifully simple
	localX = -localX;
	localDz = -localDz;

	//std::cerr << "new local Velocity = (" << localDx << ", " << localDz << ")\n";
	//std::cerr << "new local Position = (" << localX << ", " << localZ << ")\n";


	// 6. transform back to world space
	nextStep.outputCOMState.x = nextStep.parabolaX + ix * localX  +  jx * localZ;
	nextStep.outputCOMState.z = nextStep.parabolaZ + iz * localX  +  jz * localZ;
	nextStep.outputCOMState.dx = ix * localDx  +  jx * localDz;
	nextStep.outputCOMState.dz = iz * localDx  +  jz * localDz;
	*/


	//std::cerr << "new world parabola origin = (" << nextStep.parabolaX << ", " << nextStep.parabolaZ << ")\n";
	//std::cerr << "new world mass Position = (" << nextStep.outputCOMState.x << ", " << nextStep.outputCOMState.z << ")\n";
	//std::cerr << "new world Velocity = (" << nextStep.outputCOMState.dx << ", " << nextStep.outputCOMState.dz << ")\n";


	// 7. fill in the rest of the step data.
	nextStep.parabolaOrientationPhi = parabolaOrientationPhi;
	nextStep.isAGoalState = false; // the FootstepEnvironment may override this.
	nextStep.footX = nextStep.parabolaX - nextStep.simulationJx * _walkParameters.baseRadius;
	nextStep.footZ = nextStep.parabolaZ - nextStep.simulationJz * _walkParameters.baseRadius;


	if ((nextState == FOOTSTEP_STATE_NORMAL) || (nextState == FOOTSTEP_STATE_STOPPING)) {
		// check if the distance from COM to foot position (origin) is within the proper step length walk parameters this character is allowed
		float stepDistanceSquared = (nextStep.footX-previousStep.footX)*(nextStep.footX-previousStep.footX) + (nextStep.footZ-previousStep.footZ)*(nextStep.footZ-previousStep.footZ);

		//std::cerr << "step distance is " << sqrtf(stepDistanceSquared) << "\n\n";
		if (stepDistanceSquared < _walkParameters.minStepLength*_walkParameters.minStepLength) {
			//std::cerr << "####  FootstepAgent::_createFootstepAction() - cannot create this step, it is too small!  " << sqrtf(stepDistanceSquared) << " < " << _walkParameters.minStepLength << "\n";
			return false;
		}

		if (stepDistanceSquared > _walkParameters.maxStepLength*_walkParameters.maxStepLength) {
			//std::cerr << "####  FootstepAgent::_createFootstepAction() - cannot create this step, it is too large!  " << sqrtf(stepDistanceSquared) << " > " << _walkParameters.maxStepLength << "\n";
			return false;
		}
	}




	// 8. figure out the valid interval of foot orientations based on current parabola and previous footstep.
	if (nextState == FOOTSTEP_STATE_STARTING) {
			nextStep.innerFootOrientationPhi = previousStep.innerFootOrientationPhi;
			nextStep.outerFootOrientationPhi = previousStep.outerFootOrientationPhi;
	}
	else {
		float outerMomentumOrientationBeforeStep = atan2f(previousStep.outputCOMState.dz, previousStep.outputCOMState.dx);
		float innerMomentumOrientationAfterStep = atan2f(nextStep.outputCOMState.dz, nextStep.outputCOMState.dx);
		if (nextStep.whichFoot == LEFT_FOOT) {
			//
			// then THIS IS A LEFT FOOT:
			//
			// the "inner" side will be smaller angles
			// the "outer" side will be larger angles
			//
			//std::cerr << "PREVIOUS min/max foot orientation:  " << previousStep.minFootOrientationPhi << " to " << previousStep.maxFootOrientationPhi << "\n";
			//std::cerr << "parabola orientation is " << parabolaOrientationPhi << "\n";
			nextStep.innerFootOrientationPhi = previousStep.outerFootOrientationPhi - 0.1f * M_PI_OVER_2;
			nextStep.outerFootOrientationPhi = previousStep.innerFootOrientationPhi + M_PI_OVER_2;

			//
			// the next several computations are required because of the periodic nature of angles on a 
			// circle, which causes problems when trying to track an interval.
			// there is enough information, however, to "correct" the interval before intersecting with the inner/outer interval.
			//

			// for a left foot, the initial angle is larger, and then the exit trajectory angle is smaller.  
			// correct the computed angles to make sure they are correct relative to each other.
			while (outerMomentumOrientationBeforeStep < innerMomentumOrientationAfterStep) {
				outerMomentumOrientationBeforeStep += M_2_PI;
			}

			// then initialize the interval so it is completely larger than the foot interval
			while (innerMomentumOrientationAfterStep < nextStep.outerFootOrientationPhi) {
				outerMomentumOrientationBeforeStep += M_2_PI;
				innerMomentumOrientationAfterStep += M_2_PI;
			}

			// and finally find the interval where the lower bound of momentum orientations JUST passes the upper bound of foot orientations
			while (innerMomentumOrientationAfterStep > nextStep.outerFootOrientationPhi) {
				outerMomentumOrientationBeforeStep -= M_2_PI;
				innerMomentumOrientationAfterStep -= M_2_PI;
			}

			// at this point we have found the same "relative period" where it is valid to compute the intersection
			nextStep.innerFootOrientationPhi = max(nextStep.innerFootOrientationPhi, innerMomentumOrientationAfterStep);
			nextStep.outerFootOrientationPhi = min(nextStep.outerFootOrientationPhi, outerMomentumOrientationBeforeStep);

			// if the interval is over-constrained, then this is not a valid footstep choice
			if (nextStep.innerFootOrientationPhi > nextStep.outerFootOrientationPhi) {
				//std::cerr << "TODO!!! this left foot is improperly constrained!! (fixing for now)\n";
				//nextStep.innerFootOrientationPhi = parabolaOrientationPhi;
				//nextStep.outerFootOrientationPhi = parabolaOrientationPhi;
				return false;
			}
		}
		else {
			//
			// then THIS IS A RIGHT FOOT:
			//
			// the "inner" side will be larger angles
			// the "outer" side will be smaller angles
			//
			//std::cerr << "PREVIOUS min/max foot orientation:  " << previousStep.minFootOrientationPhi << " to " << previousStep.maxFootOrientationPhi << "\n";
			//std::cerr << "parabola orientation is " << parabolaOrientationPhi << "\n";
			nextStep.innerFootOrientationPhi = previousStep.outerFootOrientationPhi + 0.1f * M_PI_OVER_2;
			nextStep.outerFootOrientationPhi = previousStep.innerFootOrientationPhi - M_PI_OVER_2;

			//
			// the next several computations are required because of the periodic nature of angles on a 
			// circle, which causes problems when trying to track an interval.
			// there is enough information, however, to "correct" the interval before intersecting with the inner/outer interval.
			//

			// for a left foot, the initial angle is larger, and then the exit trajectory angle is smaller.  
			// correct the computed angles to make sure they are correct relative to each other.
			while (outerMomentumOrientationBeforeStep > innerMomentumOrientationAfterStep) {
				outerMomentumOrientationBeforeStep -= M_2_PI;
			}

			// then initialize the interval so it is completely larger than the foot interval
			while (innerMomentumOrientationAfterStep > nextStep.outerFootOrientationPhi) {
				outerMomentumOrientationBeforeStep -= M_2_PI;
				innerMomentumOrientationAfterStep -= M_2_PI;
			}

			// and finally find the interval where the lower bound of momentum orientations JUST passes the upper bound of foot orientations
			while (innerMomentumOrientationAfterStep < nextStep.outerFootOrientationPhi) {
				outerMomentumOrientationBeforeStep += M_2_PI;
				innerMomentumOrientationAfterStep += M_2_PI;
			}

			// at this point we have found the same "relative period" where it is valid to compute the intersection
			nextStep.innerFootOrientationPhi = min(nextStep.innerFootOrientationPhi, innerMomentumOrientationAfterStep);
			nextStep.outerFootOrientationPhi = max(nextStep.outerFootOrientationPhi, outerMomentumOrientationBeforeStep);

			// if the interval is over-constrained, then this is not a valid footstep choice.
			if (nextStep.innerFootOrientationPhi < nextStep.outerFootOrientationPhi) {
				//std::cerr << "TODO!!! this right foot is improperly constrained!! (fixing for now)\n";
				//nextStep.innerFootOrientationPhi = parabolaOrientationPhi;
				//nextStep.outerFootOrientationPhi = parabolaOrientationPhi;
				return false;
			}

		}
	}



	// 9. iterate over several points in time of the trajectory, and check if the character will collide with anything.
	// there are 3 possible scenarios:
	//   - step is valid normally with no collisions
	//   - step is invalid because there is a collision with the character's main body
	//   - step should have an increased cost, but is still valid, because there was no collision with the main body,
	//     but there was with the shoulders (or other extremities).
	//

	//bool mustRotateShoulders = false;
	bool collisionWithAgentFound = false;
	bool collisionWithObstacleFound = false;
	//for (float interp = 0.5f; interp <= 1.0f; interp += 0.49f) {
	//for (float interp = 0.33f; interp <= 1.0f; interp += 0.32f) {
	//for (float interp = 0.20f; interp <= 1.0f; interp += 0.19f) {
	for (float interp = 0.05f; interp <= 1.0f; interp += 0.09f) {
		Vector collisionForward;
		Point collisionPosition;
		float collisionTime = nextStep.startTime + stepDuration * interp;
		evaluateParabolaAtTime(nextStep, collisionTime, collisionPosition, collisionForward);
		Vector collisionRightSide = normalize(Vector(collisionForward.z, 0.0f, -collisionForward.x));

		// for every object in the character's visual field...
		for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin();  neighbor != _neighbors.end();  ++neighbor) {
			if ((*neighbor)->isAgent()) {
				// collision procedure for other agents
				OracleAgent * agent;
				agent = dynamic_cast<OracleAgent*>(*neighbor);
				
				// check if the obstacle collides with the character's main body
				if (agent->collidesAtTimeWith(collisionPosition, collisionRightSide, _radius, collisionTime, nextStep.footX, nextStep.footZ )) {
					collisionWithAgentFound = true;
					break;
				}

			}
			else {
				// collision procedure for obstacles
				SteerLib::ObstacleInterface * obstacle;
				obstacle = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbor);

				// check if the obstacle collides with the character's main body
				if (obstacle->overlaps(collisionPosition, _radius)) {
					collisionWithObstacleFound = true;
					break;
				}

				// check if the obstacle collides with the character's shoulders.
				// for now, if mustRotateShoulders is already true, we can skip this check.
				if (( obstacle->overlaps(collisionPosition + _radius*collisionRightSide, _radius) || obstacle->overlaps(collisionPosition - _radius*collisionRightSide, _radius))) {
					collisionWithObstacleFound = true;
					break;
				}

				// OLD VERSION - NOTE CAREFULLY THE "!mustRotateShoulders" portion...
				//if ((!mustRotateShoulders)  && ( obstacle->overlaps(collisionPosition + _radius*collisionRightSide, _radius) || obstacle->overlaps(collisionPosition - _radius*collisionRightSide, _radius))) {
				//	mustRotateShoulders = true;
				//}

			}

		}
	}

	if (collisionWithAgentFound) {
		nextStep.energyCost += 100.0f * _walkParameters.mass;
		return false;
	}

	if (collisionWithObstacleFound) {
		nextStep.energyCost += 100000000.0f * _walkParameters.mass;
		return false;
	}

	if ((!collisionWithAgentFound) && (!collisionWithObstacleFound)) {
		if ( (nextStep.outputCOMState.x - _cachedFootstepOptions[1].outputCOMState.x)*(nextStep.outputCOMState.x - _cachedFootstepOptions[1].outputCOMState.x) + (nextStep.outputCOMState.z - _cachedFootstepOptions[1].outputCOMState.z)*(nextStep.outputCOMState.z - _cachedFootstepOptions[1].outputCOMState.z) < PED_REACHED_FOOTSTEP_GOAL_THRESHOLD*PED_REACHED_FOOTSTEP_GOAL_THRESHOLD) {
			nextStep.isAGoalState = true;
			nextStep.energyCost = 0.0f; // so that this node is expanded next, an important speedup (near-optimal solution)
		}
	}

	//std::cerr << "\nCREATED STEP:\n    " << nextStep << "\n";

	return true;

}

void OracleAgent::_planFootstepsToShortTermGoal(bool useHardCodedPlan)
{
	#ifdef _DEBUG
	cout << "Footstep Planning from " << _currentStep.state << endl;
	cout.flush();
	#endif
	AutomaticFunctionProfiler profileThisFunction( &OracleGlobals::gPhaseProfilers->footstepPlanningPhaseProfiler );
	AStarLite searchAlgorithm;

	Footstep goalState;
	goalState.outputCOMState.x = _localTargetLocation.x;
	goalState.outputCOMState.z = _localTargetLocation.z;
		
	_currentLocationInPath = 0;
	_footstepPlan.clear();
	_cachedFootstepOptions.clear();
	_currentStep.isAGoalState = false;
	_cachedFootstepOptions.push_back(_currentStep);
	_cachedFootstepOptions.push_back(goalState);

	Util::PerformanceProfiler footstepPlanProfiler;
	footstepPlanProfiler.reset();
	footstepPlanProfiler.start();

	OracleEnvironment footstepPlanningEnvironment(this);
	searchAlgorithm.findPath(footstepPlanningEnvironment, 0, 1, false, this);

	footstepPlanProfiler.stop();

	_needToRunFootstepPlanner = false;

	if (searchAlgorithm.getPath().size() == 0) {
		std::cerr << "================\n" << "NO PATH FOUND TO " << _localTargetLocation << ".\n";
		std::cerr << "current step was:\n" << _currentStep;
		std::cerr << "  - expanded  " << footstepPlanningEnvironment._numNodesExpanded << " nodes\n";
		std::cerr << "  - generated " << _cachedFootstepOptions.size() << " nodes\n";
		footstepPlanProfiler.displayStatistics(std::cerr);
		std::cerr << "EXITING UNGRACEFULLY.\n";
		exit(1);
	}
	else {
		for (int i=searchAlgorithm.getPath().size()-1; i >= 0; i--) {
			_footstepPlan.push_back( searchAlgorithm.getPath()[i] );
			_cachedFootstepOptions[ searchAlgorithm.getPath()[i] ].targetX = _localTargetLocation.x;
			_cachedFootstepOptions[ searchAlgorithm.getPath()[i] ].targetZ = _localTargetLocation.z;
			if (_cachedFootstepOptions[ searchAlgorithm.getPath()[i] ].isAGoalState)
				break;
		}
	}
	//_currentStep = _cachedFootstepOptions[_footstepPlan[_currentLocationInPath]];

	#ifdef _DEBUG
	cout << "Before culling: " << _cachedFootstepOptions.size() << endl;
	cout.flush();
	#endif

	// Need to reduce the cached steps to those necessary for the path
	vector<Footstep> tempCache;
	tempCache.reserve(_footstepPlan.size() + 1);
	tempCache.push_back(_cachedFootstepOptions[0]);
	tempCache.push_back(_cachedFootstepOptions[1]);

	for(unsigned int i = 1; i < _footstepPlan.size(); i++) {
		tempCache.push_back(_cachedFootstepOptions[_footstepPlan[i]]);
	}

	_cachedFootstepOptions.clear();

	for(auto iter = tempCache.begin(); iter != tempCache.end(); ++iter)
	{
		_cachedFootstepOptions.push_back(*iter);
	}

	_cachedFootstepOptions.shrink_to_fit();
	// footstepPlan can become just counting-numbers with the exception of skipping element 1
	_footstepPlan.clear();
	_footstepPlan.push_back(0);
	_footstepPlan.push_back(2);
	for(unsigned int i = 3; i < _cachedFootstepOptions.size(); i++) {
		_footstepPlan.push_back(i);
	}

	#ifdef _DEBUG
	cout << "After culling: " << _cachedFootstepOptions.size() << endl;
	cout.flush();
	#endif

	_needToRunFootstepPlanner = false;
}

void OracleAgent::_updateCharacterFollowingPath(float currentTime)
{
	AutomaticFunctionProfiler profileThisFunction( &OracleGlobals::gPhaseProfilers->locomotionPhaseProfiler );

	// check if we should increment to the next step.
	if (_cachedFootstepOptions[_footstepPlan[_currentLocationInPath]].endTime < currentTime ) {
		if ((_currentLocationInPath == _footstepPlan.size()-1) ) // || (_currentLocationInPath > PED_NUM_STEPS_BEFORE_FORCED_PLAN))
			{
			// then we need to re-plan to the next goal (if there is another goal)
			_runShortTermPlanningPhase();
			if (!_enabled) return;
			_needToRunFootstepPlanner = true;
		}
		else {
			_stepHistory.push_back(_currentStep);
			_currentLocationInPath++;
			_previousStep = _currentStep;
			_currentStep = _cachedFootstepOptions[_footstepPlan[_currentLocationInPath]];
			//std::cerr << "taking next step:\n     " << _currentStep;
		}
	}


	// follow the planned path
	evaluateParabolaAtTime(_currentStep, currentTime, _currentSimulatedPosition, _currentSimulatedVelocity);


	// update the character's location in the spatial database
	AxisAlignedBox oldBounds = AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	_position = _currentSimulatedPosition;
	_forward = normalize(_currentSimulatedVelocity);
	AxisAlignedBox newBounds = AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	gSpatialDatabase->updateObject( dynamic_cast<SpatialDatabaseItemPtr>(this), oldBounds, newBounds);
}


void OracleAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{

	gTempCurrentTime = timeStamp;

	AutomaticFunctionProfiler profileThisFunction( &OracleGlobals::gPhaseProfilers->aiProfiler );

	if (frameNumber == 1) {
		_runLongTermPlanningPhase();
		_runMidTermPlanningPhase();
		_runShortTermPlanningPhase();
	}


	if (_needToRunFootstepPlanner) {
		_runPerceptivePhase();
		_planFootstepsToShortTermGoal(false);
	}

	// updates the character's situation along the current step's parabola
	// increments to the next step as appropriate
	// schedules another footstep plan for the next frame if appropriate.
	if ( _enabled )
	{ // This should not be called if the Character is disabled.
		_updateCharacterFollowingPath(timeStamp);
	}

}

void OracleAgent::draw()
{
	AutomaticFunctionProfiler profileThisFunction( &OracleGlobals::gPhaseProfilers->drawProfiler );

	// draw positive axes (red is x, blue is z)
	//DrawLib::drawLine(  Point( 0.0f, 0.0f, 0.0f ), Point(100.0f, 0.0f, 0.0f) , gRed );
	//DrawLib::drawLine(  Point( 0.0f, 0.0f, 0.0f ), Point(0.0f, 0.0f, 100.0f) , gBlue );

	// draw center of mass
	DrawLib::drawAgentDisc( _currentSimulatedPosition,  _currentSimulatedVelocity, 0.15f, gRed);

	float phi = determineFootstepOrientation(_currentStep, _previousStep);
	drawFoot( _currentStep.footX, _currentStep.footZ, phi, _currentStep.whichFoot);
	drawFootstepTrajectory(_currentStep, gGreen);

	// determine the other "active" foot based on what time it occurs
	float halfwayTime = (_currentStep.endTime + _currentStep.startTime) * 0.5f;
	if ((gTempCurrentTime >= halfwayTime) && (_currentLocationInPath < _footstepPlan.size()-1)) {
		float phi = determineFootstepOrientation(_cachedFootstepOptions[_footstepPlan[_currentLocationInPath+1]],_cachedFootstepOptions[_footstepPlan[_currentLocationInPath]]);
		drawFoot( _cachedFootstepOptions[_footstepPlan[_currentLocationInPath+1]].footX, _cachedFootstepOptions[_footstepPlan[_currentLocationInPath+1]].footZ, phi, _cachedFootstepOptions[_footstepPlan[_currentLocationInPath+1]].whichFoot);
		drawFootstepTrajectory(_cachedFootstepOptions[_footstepPlan[_currentLocationInPath+1]], gBlack);
	}
	else if ((gTempCurrentTime < halfwayTime) && (_currentLocationInPath > 0)) {
		float phi;
		if (_currentLocationInPath > 1)
			phi = determineFootstepOrientation(_cachedFootstepOptions[_footstepPlan[_currentLocationInPath-1]],_cachedFootstepOptions[_footstepPlan[_currentLocationInPath-2]]);
		else
			phi = _cachedFootstepOptions[_footstepPlan[_currentLocationInPath-1]].innerFootOrientationPhi;
		drawFoot( _cachedFootstepOptions[_footstepPlan[_currentLocationInPath-1]].footX, _cachedFootstepOptions[_footstepPlan[_currentLocationInPath-1]].footZ, phi, _cachedFootstepOptions[_footstepPlan[_currentLocationInPath-1]].whichFoot);
		drawFootstepTrajectory(_cachedFootstepOptions[_footstepPlan[_currentLocationInPath-1]], gBlack);
	}


	if (gEngineInfo->isAgentSelected(this)) {

		// draw goal
		//DrawLib::drawFlag( _currentGoal.targetLocation );
		DrawLib::drawFlag(_localTargetLocation);
		if (_cachedFootstepOptions.size() > 2) DrawLib::drawFlag( Point(_cachedFootstepOptions[1].outputCOMState.x, 0.0f, _cachedFootstepOptions[1].outputCOMState.z), gBlue);


		// draw collision bounds
		// approx 30 cm from back to front
		// approx 60 cm side-to-side ??
		DrawLib::drawAgentDisc( _currentSimulatedPosition,  _currentSimulatedVelocity, _radius, gRed);
		DrawLib::drawAgentDisc( _currentSimulatedPosition + _radius * rightSideInXZPlane(normalize(_currentSimulatedVelocity)),  _currentSimulatedVelocity, _radius, gRed);
		DrawLib::drawAgentDisc( _currentSimulatedPosition - _radius * rightSideInXZPlane(normalize(_currentSimulatedVelocity)),  _currentSimulatedVelocity, _radius, gRed);

		// draw a blue line to any objects the character sees
		for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin();  neighbor != _neighbors.end();  ++neighbor) {
			if ( (*neighbor)->isAgent()) {
				OracleAgent * agent;
				agent = dynamic_cast<OracleAgent*>(*neighbor);
				DrawLib::drawLine(_position, agent->position(), gBlue);
			}
		}

		// DRAW	long term plan
		for (int nn = 0; nn < (_waypoints.size() - 1) && _waypoints.size() > 1; nn++)
		{

			DrawLib::drawLine(_waypoints.at(nn), _waypoints.at(nn+1), gBlack);
		}

		// Draw midtermPath
		for (int nn = 0; nn < _midTermPathSize - 1 && _midTermPathSize > 1; nn++)
		{
			Point waypointStart;
			Point waypointEnd;
			gSpatialDatabase->getLocationFromIndex(_midTermPath[nn],waypointStart);
			gSpatialDatabase->getLocationFromIndex(_midTermPath[nn+1],waypointEnd);
			DrawLib::drawLine(waypointStart, waypointEnd, gRed);
		}
	}

	// highlights the character if there are collisions
	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin();  neighbor != _neighbors.end();  ++neighbor) {
		if ( (*neighbor)->isAgent()) {
			OracleAgent * agent;
			agent = dynamic_cast<OracleAgent*>(*neighbor);
			// TODO: annotate collisions with other agents here
			//Vector rightSide = normalize(Vector(_forward.z, _forward.y, -_forward.x));
			Vector rightSide = normalize(Vector(_currentSimulatedVelocity.z, _forward.y, -_currentSimulatedVelocity.x));
			if (agent->collidesAtTimeWith(_currentSimulatedPosition, rightSide, _radius, gTempCurrentTime, _currentStep.footX, _currentStep.footZ)) {
				DrawLib::drawHighlight(_currentSimulatedPosition,_currentSimulatedVelocity, 0.16f, gBlue);
				#ifdef _DEBUG
				cout << "Collision! " << endl;
				#endif
			}
		}
		else {
			SteerLib::ObstacleInterface * obstacle;
			obstacle = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbor);
			Point pos = _currentSimulatedPosition;
			Vector rightSide = normalize(Vector(_currentSimulatedVelocity.z, 0.0f, -_currentSimulatedVelocity.x));
			if (obstacle->overlaps(pos, _radius) || obstacle->overlaps(pos + _radius*rightSide, _radius) || obstacle->overlaps(pos - _radius*rightSide, _radius))
			{
				DrawLib::drawHighlight(_currentSimulatedPosition,_currentSimulatedVelocity, 0.16f, gGreen);
				//std::cerr << "ERROR!!! COLLISION DURING STEP:\n      " << _currentStep;
				//std::cerr << "TESTED pos1 = " << pos << "\n";
				//std::cerr << "TESTED pos2 = " << pos + _radius*rightSide << "\n";
				//std::cerr << "TESTED pos3 = " << pos - _radius*rightSide << "\n";
				//std::cerr << "right side = " << rightSide << "\n";

			}
		}
	}
}



bool OracleAgent::collidesAtTimeWith(const Point & p1, const Vector & rightSide, float otherAgentRadius, float timeStamp, float otherAgentFootX, float otherAgentFootZ)
{

	// find the step that is at the time of interest
	unsigned int stepIndex = _currentLocationInPath;
	while ( (stepIndex < _footstepPlan.size())  &&  (_cachedFootstepOptions[_footstepPlan[stepIndex]].endTime < timeStamp) ) {
		stepIndex++;
	}
	if (stepIndex == _footstepPlan.size()) {
		return false; // this character has not planned that far in advance.
	}

	// NOTE CAREFULLY this is an alias
	Footstep & myStep = _cachedFootstepOptions[_footstepPlan[stepIndex]];

	Point s1;
	Vector simulatedVel;

	evaluateParabolaAtTime(myStep, timeStamp, s1, simulatedVel);

	float distanceSquaredThreshold = (otherAgentRadius+_radius)*(otherAgentRadius+_radius);

	if (  (p1-s1).lengthSquared() < distanceSquaredThreshold ) {
		return true;
	}



	Point p2, p3, p4, s2, s3, s4;
	Vector myRightSide = normalize(Vector(simulatedVel.z, 0.0f, -simulatedVel.x));

	p2 = p1 + otherAgentRadius*rightSide;
	p3 = p1 - otherAgentRadius*rightSide;
	p4 = Point(otherAgentFootX, 0.0f, otherAgentFootZ);

	s2 = s1 + _radius*myRightSide;
	s3 = s1 - _radius*myRightSide;
	s4 = Point(myStep.footX, 0.0f, myStep.footZ);



	// TODO: rigorously analyze these probabilities to verify which ordering is best.
	// Here, we order the 20 collision tests based on what is most likely.
	//  our heuristic is that shoulder-shoulder collisions are most likey, followed by footstep collisions,
	//  and lastly torso collisions.

	// shoulder-shoulder collisions
	if ((  (p2-s2).lengthSquared() < distanceSquaredThreshold ) ||
		(  (p2-s3).lengthSquared() < distanceSquaredThreshold ) ||
		(  (p3-s2).lengthSquared() < distanceSquaredThreshold ) ||
		(  (p3-s3).lengthSquared() < distanceSquaredThreshold ))
	{
		return true;
	}

	// footstep collision tests
	if ((  (p2-s4).lengthSquared() < distanceSquaredThreshold ) ||
		(  (p3-s4).lengthSquared() < distanceSquaredThreshold ) ||
		(  (p4-s2).lengthSquared() < distanceSquaredThreshold ) ||
		(  (p4-s3).lengthSquared() < distanceSquaredThreshold ) ||
		(  (p4-s4).lengthSquared() < distanceSquaredThreshold ))
	{
		return true;
	}

	// torso collisions
	if ( // (  (p1-s1).lengthSquared() < distanceSquaredThreshold ) || // <-- this one is already computed above.
		(  (p1-s2).lengthSquared() < distanceSquaredThreshold ) ||
		(  (p1-s3).lengthSquared() < distanceSquaredThreshold ) ||
		(  (p1-s4).lengthSquared() < distanceSquaredThreshold ) ||
		(  (p2-s1).lengthSquared() < distanceSquaredThreshold ) ||
		(  (p3-s1).lengthSquared() < distanceSquaredThreshold ) ||
		(  (p4-s1).lengthSquared() < distanceSquaredThreshold ))
	{
		return true;
	}
	

	// if a secondary step exists, check with that one
	float halfwayTime = (myStep.endTime + myStep.startTime) * 0.5f;
	if ((timeStamp >= halfwayTime) && (stepIndex < _footstepPlan.size()-1)) {
		// check with the foot in front
		Point s5 = Point(_cachedFootstepOptions[_footstepPlan[stepIndex+1]].footX, 0.0f, _cachedFootstepOptions[_footstepPlan[stepIndex+1]].footZ);
		if ((  (p1-s5).lengthSquared() < distanceSquaredThreshold ) ||
			(  (p2-s5).lengthSquared() < distanceSquaredThreshold ) ||
			(  (p3-s5).lengthSquared() < distanceSquaredThreshold ) ||
			(  (p4-s5).lengthSquared() < distanceSquaredThreshold )) {
				return true;
		}
	}
	else if ((timeStamp < halfwayTime) && (stepIndex > 0)) {
		// check with the foot behind
		Point s5 = Point(_cachedFootstepOptions[_footstepPlan[stepIndex-1]].footX, 0.0f, _cachedFootstepOptions[_footstepPlan[stepIndex-1]].footZ);
		if ((  (p1-s5).lengthSquared() < distanceSquaredThreshold ) ||
			(  (p2-s5).lengthSquared() < distanceSquaredThreshold ) ||
			(  (p3-s5).lengthSquared() < distanceSquaredThreshold ) ||
			(  (p4-s5).lengthSquared() < distanceSquaredThreshold )) {
				return true;
		}
	}
	else {
		// not common to reach here, it means that there was no "secondary step" at the moment.
	}




	// no collision  (unfortunately, this is the common case, which means we have to go through all 20 checks)
	return false;
}

///===========================================================================
///===========================================================================
///===========================================================================
///===========================================================================
///===========================================================================
///===========================================================================
///===========================================================================
///===========================================================================
///===========================================================================
///===========================================================================
///===========================================================================
///===========================================================================
///===========================================================================
///===========================================================================
///===========================================================================
///===========================================================================
///===========================================================================
///===========================================================================
///===========================================================================
///===========================================================================
///===========================================================================





void OracleAgent::addGoal(const SteerLib::AgentGoalInfo & newGoal) { 
	if (newGoal.goalType != SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
		throw Util::GenericException("Currently the phase decimation agent does not support goal types other than GOAL_TYPE_SEEK_STATIC_TARGET.");
	}
	_goalQueue.push(newGoal);
	if (_goalQueue.size()==1) {
		_currentGoal = newGoal;
		if (_currentGoal.targetIsRandom) {

			Util::AxisAlignedBox aab = Util::AxisAlignedBox(-100.0f, 100.0f, 0.0f, 0.0f, -100.0f, 100.0f);
			_currentGoal.targetLocation = gSpatialDatabase->randomPositionInRegionWithoutCollisions(aab, 1.0f, true);
		}
	}
}



//
// reachedCurrentGoal()
//
bool OracleAgent::_reachedCurrentGoal()
{
	return ( (_currentGoal.targetLocation-_position).lengthSquared() < (PED_REACHED_TARGET_DISTANCE_THRESHOLD * PED_REACHED_TARGET_DISTANCE_THRESHOLD) );
}


//
// reachedCurrentWaypoint()
//
bool OracleAgent::_reachedCurrentWaypoint()
{
	return ( (_waypoints[_currentWaypointIndex]-_position).lengthSquared() < (PED_REACHED_TARGET_DISTANCE_THRESHOLD * PED_REACHED_TARGET_DISTANCE_THRESHOLD) );
}



//
// reachedLocalTarget()
//
bool OracleAgent::_reachedLocalTarget()
{
	return ( (_localTargetLocation-_position).lengthSquared() < (PED_REACHED_TARGET_DISTANCE_THRESHOLD * PED_REACHED_TARGET_DISTANCE_THRESHOLD) );
}


void OracleAgent::disable()
{
	this->_disable();
}

//
// disable()
//
void OracleAgent::_disable()
{
	// if we tried to disable a second time, most likely we accidentally ignored that it was disabled, and should catch that error.
	assert(_enabled==true);  

	//  1. remove from database
	AxisAlignedBox b = AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	gSpatialDatabase->removeObject(dynamic_cast<SpatialDatabaseItemPtr>(this), b);

	//  2. set enabled = false
	_enabled = false;
}



//
// runCognitivePhase()
//
void OracleAgent::_runCognitivePhase()
{
	//assert(_goalQueue.front() == _currentGoal);

	// pop off the previous goal
	_goalQueue.pop();

	if (_goalQueue.empty()) {
		// nowhere left to steer, this pedestrian is done.
		_disable();
		return;
	}
	else {
		_currentGoal = _goalQueue.front();
	}

	// if the goal asks for a random target, then randomly assign the target location
	if (_currentGoal.goalType != GOAL_TYPE_SEEK_STATIC_TARGET) {
		throw GenericException("Currently the phase decimation agent only supports GOAL_TYPE_SEEK_STATIC_TARGET goal types");
	}
	if (_currentGoal.targetIsRandom) {
		AxisAlignedBox aab = AxisAlignedBox(-100.0f, 100.0f, 0.0f, 0.0f, -100.0f, 100.0f);
		_currentGoal.targetLocation = gSpatialDatabase->randomPositionInRegionWithoutCollisions(aab, 1.0f, true);
	}

}


//
// runLongTermPlanningPhase()
//
void OracleAgent::_runLongTermPlanningPhase()
{
	#ifdef _DEBUG
	cout << "Long Term Planning" << endl;
	cout.flush();
	#endif
#ifndef USE_ANNOTATIONS
	// if not using annotations, then declare things local here.
	AStarLite longTermAStar;
#endif

#ifdef IGNORE_PLANNING
	return;
#endif

	if (!_enabled) return;


	// if we're at our destination, then chose a new landmark target.
	if (_reachedCurrentGoal()) {
		_runCognitivePhase();
		if (!_enabled) return;
	}

	AutomaticFunctionProfiler profileThisFunction( &OracleGlobals::gPhaseProfilers->longTermPhaseProfiler );

	//==========================================================================
	int myIndexPosition = gSpatialDatabase->getCellIndexFromLocation(_position);
	int goalIndex = gSpatialDatabase->getCellIndexFromLocation(_currentGoal.targetLocation);

	if (myIndexPosition != -1) {
		// run the main a-star search here
		longTermAStar.findPath((*gEnvironmentForAStar), myIndexPosition, goalIndex, false, this);

		// set up the waypoints along this path.
		// if there was no path, then just make one waypoint that is the landmark target.
		_waypoints.clear();
		int longTermAStarLength = (int) longTermAStar.getPath().size();
		if (longTermAStarLength > 2) {
			// note the >2 condition: if the astar path is not at least this large, then there will be a behavior bug in the AI
			// when it tries to create waypoints.  in this case, the right thing to do is create only one waypoint that is at the landmark target.
			// remember the astar lib produces "backwards" paths that start at [pathLengh-1] and end at [0].
			int nextWaypointIndex = longTermAStarLength - 1 - NEXT_WAYPOINT_DISTANCE;
			while (nextWaypointIndex > 0) {
				Point waypoint;
				gSpatialDatabase->getLocationFromIndex(longTermAStar.getPath()[nextWaypointIndex],waypoint);
				_waypoints.push_back(waypoint);
				nextWaypointIndex -= NEXT_WAYPOINT_DISTANCE;
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


//
// runMidTermPlanningPhase()
//
void OracleAgent::_runMidTermPlanningPhase()
{
	#ifdef _DEBUG
	cout << "MidTerm Planning" << endl;
	cout.flush();
	#endif
#ifndef USE_ANNOTATIONS
	// if not using annotations, then declare things local here.
	AStarLite midTermAStar;
#endif

#ifdef IGNORE_PLANNING
	return;
#endif

	if (!_enabled) return;

	// if we reached the final goal, then schedule long-term planning to run
	// long-term planning will call cognitive
	if (_reachedCurrentGoal()) {
		_runLongTermPlanningPhase();
		if (!_enabled) return;
	}

	AutomaticFunctionProfiler profileThisFunction( &OracleGlobals::gPhaseProfilers->midTermPhaseProfiler );

	// if we reached the current waypoint, then increment to the next waypoint
	if (_reachedCurrentWaypoint()) {
		_currentWaypointIndex++;
	}

	// compute a local a-star from your current location to the waypoint.
	int myIndexPosition = gSpatialDatabase->getCellIndexFromLocation(_position.x, _position.z);
	int waypointIndexPosition = gSpatialDatabase->getCellIndexFromLocation(_waypoints[_currentWaypointIndex].x, _waypoints[_currentWaypointIndex].z);
	midTermAStar.findPath((*gEnvironmentForAStar), myIndexPosition, waypointIndexPosition, false, this);

	// copy the local AStar path to your array
	// keeping the convention that the beginning of the path starts at the end of the array
	_midTermPathSize = (int)(midTermAStar.getPath()).size();

	// sanity checks
	if (_midTermPathSize != (int)(midTermAStar.getPath()).size()) {
		std::cerr << "ERROR!!!  _midTermPathSize does not equal (midTermAStar.getPath()).size(). exiting ungracefully.\n";
		assert(false);
	}

	// WARNING!!!   THIS WAS HACKED TO REMOVE A BUG, BUT BUG WAS NOT SOLVED.
	//  it used to be " + 1",  but we have since increased the size of the _midTermPath array, just
	//  so it allows extra leftovers.
	if (_midTermPathSize > NEXT_WAYPOINT_DISTANCE + 3) {
		// the plus 1 is because a-star counts the agent's immediate location, but we do not.
		std::cerr << "ERROR!!!  _midTermPathSize is larger than expected: should be less than or equal to " << NEXT_WAYPOINT_DISTANCE+1 << ", but it actually is " << _midTermPathSize << "\n";
		std::cerr << "exiting ungracefully.\n";
		exit(1);
	}

	for (int i=0; i<_midTermPathSize-1; i++) {
		_midTermPath[i] = (midTermAStar.getPath())[i];
	}

	// TODO: should we reset the localTarget here??
}

//
// runShortTermPlanningPhase()
//
void OracleAgent::_runShortTermPlanningPhase()
{
	#ifdef _DEBUG
	cout << "Short Term Planning" << endl;
	cout.flush();
	#endif
	int closestPathNode;
	if (!_enabled) return;

#ifdef IGNORE_PLANNING
	// if we want to ignore planning, then just decide to steer towards the final target.
	// possibly update the landmark target if we arrived at it.
	if (_reachedCurrentGoal()) {
		_runCognitivePhase();
	}
	_localTargetLocation = _currentGoal.targetLocation;
	return;
#endif

	// 0. if you're at your current waypoint
	if (_reachedCurrentWaypoint()) {
		// then schedule midTermPlanning phase
		_runMidTermPlanningPhase();
		if (!_enabled) return;
	}

	AutomaticFunctionProfiler profileThisFunction( &OracleGlobals::gPhaseProfilers->shortTermPhaseProfiler );

	Vector rightSide = Vector(_forward.z, _forward.y, -_forward.x);

	int myIndexPosition = gSpatialDatabase->getCellIndexFromLocation(_position.x, _position.z);
	closestPathNode = _midTermPathSize; // the last index in the path array  *** PLUS ONE ***
	if ((myIndexPosition!=-1)&&(closestPathNode!=0)) {

		// 1. find the node that you're nearest to in your current path
		// NOTE that we MUST search ALL nodes of the path here.
		// reason: the path may unintuitively snake around so that some nodes are closer than others even if they are all very far from you.
		float minDistSquared = INFINITY;
		for (int i=_midTermPathSize-1; i >= 0; i--) {
			Point tempTargetLocation;
			gSpatialDatabase->getLocationFromIndex( _midTermPath[i], tempTargetLocation);
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
			SpatialDatabaseItemPtr dummyObject;
			int localTargetIndex = closestPathNode;
			int furthestTargetIndex = max(0, closestPathNode - FURTHEST_LOCAL_TARGET_DISTANCE);
			int localTarget = _midTermPath[localTargetIndex];
			gSpatialDatabase->getLocationFromIndex( localTarget, _localTargetLocation );
			Ray lineOfSightTest1, lineOfSightTest2;
			lineOfSightTest1.initWithUnitInterval(_position + _radius*rightSide, _localTargetLocation - (_position + _radius*rightSide));
			lineOfSightTest2.initWithUnitInterval(_position - _radius*rightSide, _localTargetLocation - (_position - _radius*rightSide));
			while ((!gSpatialDatabase->trace(lineOfSightTest1,dummyt, dummyObject, dynamic_cast<SpatialDatabaseItemPtr>(this),true))
				&& (!gSpatialDatabase->trace(lineOfSightTest2,dummyt, dummyObject, dynamic_cast<SpatialDatabaseItemPtr>(this),true))
				&& (localTargetIndex >= furthestTargetIndex))
			{
				localTargetIndex--;
				localTarget = _midTermPath[localTargetIndex];
				gSpatialDatabase->getLocationFromIndex( localTarget, _localTargetLocation );
				lineOfSightTest1.initWithUnitInterval(_position + _radius*rightSide, _localTargetLocation - (_position + _radius*rightSide));
				lineOfSightTest2.initWithUnitInterval(_position - _radius*rightSide, _localTargetLocation - (_position - _radius*rightSide));
			}
			localTargetIndex++; // the last node we found was actually NOT visible, so backtrack by one.
			if (localTargetIndex <= closestPathNode) {
				// if localTargetIndex is valid
				localTarget = _midTermPath[localTargetIndex];
				gSpatialDatabase->getLocationFromIndex( localTarget, _localTargetLocation );
				if ((_localTargetLocation - _waypoints[_currentWaypointIndex]).length() < 2.0f * PED_REACHED_TARGET_DISTANCE_THRESHOLD) {
					_localTargetLocation = _waypoints[_currentWaypointIndex];
				}
			}
			else {
				// if localTargetIndex is pointing backwards, then just aim for 2 nodes ahead of the current closestPathNode.
				localTarget = _midTermPath[closestPathNode-2];
				gSpatialDatabase->getLocationFromIndex( localTarget, _localTargetLocation );
			}
		}
		else {
			_localTargetLocation = _waypoints[_currentWaypointIndex];
		}
		//_localTargetLocation.x += (static_cast<float>(rand()) / static_cast<float>(RAND_MAX+1)) * (gSpatialDatabase->getCellSizeX() / 2.0f) - (gSpatialDatabase->getCellSizeX() / 4.0f);
		//_localTargetLocation.z += (static_cast<float>(rand()) / static_cast<float>(RAND_MAX+1)) * (gSpatialDatabase->getCellSizeZ() / 2.0f) - (gSpatialDatabase->getCellSizeZ() / 4.0f);
	}
	else {
		if (myIndexPosition==-1) {
			// this case is reached when you're outside the database
			// in this case, just point towards your waypoint and hope for the best.
			_localTargetLocation = _waypoints[_currentWaypointIndex];
			//_localTargetLocation.x += (static_cast<float>(rand()) / static_cast<float>(RAND_MAX+1)) * (gSpatialDatabase->getCellSizeX() / 2.0f) - (gSpatialDatabase->getCellSizeX() / 4.0f);
			//_localTargetLocation.z += (static_cast<float>(rand()) / static_cast<float>(RAND_MAX+1)) * (gSpatialDatabase->getCellSizeZ() / 2.0f) - (gSpatialDatabase->getCellSizeZ() / 4.0f);
		}
		else if (closestPathNode==0) {
			// this case is reached when you're very close to your goal, and the planned path is very short.
			// in this case, just point towards the closest node.

			// TODO WAS THIS A BUG?  SEEMS LIKE THIS CASE IS REACHED WHEN MID TERM PATH IS ZERO??
			// COMMENTED OUT ORIGINAL AND TEMPORARILY PUT WHAT SEEMS RIGHT POINTING TO CURRENT GOAL TARGET
			//gSpatialDatabase->getLocationFromIndex( _midTermPath[closestPathNode], _localTargetLocation);
			_localTargetLocation = _currentGoal.targetLocation;
		}
		else {
			// this case should never be reached
			std::cerr << "ERROR: unexpected case! closestPathNode==" << closestPathNode << ",  myIndexPosition==" << myIndexPosition << ".\nexiting ungracefully.\n";
			assert(false);
		}
	}

/*
	if (gUseDynamicDecimation) {
		// decimating short-term planning
		float distanceHeuristic = (_position - _localTargetLocation).length() - 5.0f;
		if (distanceHeuristic <= 0.0f) {
			_framesToNextShortTermPlanning = 75;
		}
		else {
			// starting at 75 frames
			_framesToNextShortTermPlanning = 75 + (unsigned int)(2*distanceHeuristic*distanceHeuristic);
			if (_framesToNextShortTermPlanning > 170) _framesToNextShortTermPlanning = 150;
		}
	}
*/
#ifdef USE_ANNOTATIONS
	__closestPathNode = closestPathNode;
#endif
}


//
// runPerceptivePhase()
//
void OracleAgent::_runPerceptivePhase()
{
	if (!_enabled) return;

	AutomaticFunctionProfiler profileThisFunction( &OracleGlobals::gPhaseProfilers->perceptivePhaseProfiler );

	_neighbors.clear();

	// gSpatialDatabase->getItemsInVisualField(_neighbors, _position.x-PED_QUERY_RADIUS, _position.x+PED_QUERY_RADIUS, _position.z-PED_QUERY_RADIUS, _position.z+PED_QUERY_RADIUS, dynamic_cast<SpatialDatabaseItemPtr>(this), _position, _forward, (float)(PED_QUERY_RADIUS*PED_QUERY_RADIUS));

	gSpatialDatabase->getItemsInRange(_neighbors, _position.x-PED_QUERY_RADIUS, _position.x+PED_QUERY_RADIUS, _position.z-PED_QUERY_RADIUS, _position.z+PED_QUERY_RADIUS, dynamic_cast<SpatialDatabaseItemPtr>(this));
}
