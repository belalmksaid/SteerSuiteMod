//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//




#include "astar/AStarLite.h"
#include "footstepAI/GridAStar.h"
#include "footstepAI/FootstepAIModule.h"
#include "footstepAI/FootstepAgent.h"
#include "footstepAI/FootstepEnvironment.h"
#include "SteerLib.h"
#include <math.h>

using namespace Util;
using namespace SteerLib;
using namespace FootstepGlobals;



#define IN_INTERVAL_STRICT(X,A,B) (((X)>(A)) && (X)<(B))
#define IN_INTERVAL_INCLUSIVE(X,A,B) (((X)>=(A)) && (X)<=(B))

//#define _DEBUG 1
//#define DRAW_FOOTSTEP_HISTORY
#define DRAW_FOOTSTEP_PLAN
//#define PARABOLIC_HOP
//#define USE_OLD_FOOTSTEP_INIT

float gTempCurrentTime;

inline void evaluateParabolaAtTime(const Footstep & step, float currentTime, Util::Point & position, Vector & velocity)
{
	// TODO, even if this is "technically" a better thing to do, it made the AI work worse because everyone gets more stuck
	// behind each other.  (instead of the parabola eventually be evaluated outrageously away from everything else)
	// the RIGHT solution is to keep this and fix other issues.
	if (currentTime > step.endTime) currentTime = step.endTime;


	float startTime, endTime;
	float lx, lz;
	Util::Point start, end;


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
			position = Util::Point(step.outputCOMState.x, 0.0f, step.outputCOMState.z);
			velocity = Vector(step.outputCOMState.dx, 0.0f, step.outputCOMState.dz);
			//velocity = Vector(1.0f, 0.0f, 0.0f);
			return;
			break;
		case FOOTSTEP_STATE_INPLACETURNING:
			position = Util::Point(step.outputCOMState.x, 0.0f, step.outputCOMState.z);
			velocity = Vector(step.outputCOMState.dx, 0.0f, step.outputCOMState.dz);
			return;
			break;
		case FOOTSTEP_STATE_HOPPING:
#ifdef PARABOLIC_HOP
			currentLocalStepTime = (currentTime - step.startTime) - (0.5f*(step.endTime - step.startTime));
			//currentLocalStepTime = (currentTime - step.startTime) - (1.0f*(step.endTime - step.startTime));

			if (currentLocalStepTime > 0)
				currentLocalStepTime = 0;
			//currentLocalStepTime = (currentTime - step.startTime) - (0.5f*(step.endTime - step.startTime));
#else
			 //startTime = -0.5;
			startTime = 0 - (0.5f*(step.endTime - step.startTime));
			endTime = (step.endTime - step.startTime) - (0.5f*(step.endTime - step.startTime));
			 lx = step.simulationDx * startTime;
			 lz = step.simulationA * startTime * startTime;
			start.x = step.parabolaX + step.simulationIx * lx  +  step.simulationJx * lz;
			start.z = step.parabolaZ + step.simulationIz * lx  +  step.simulationJz * lz;
			 lx = step.simulationDx * endTime;
			 lz = step.simulationA * endTime * endTime;
			end.x = step.parabolaX + step.simulationIx * lx  +  step.simulationJx * lz;
			end.z = step.parabolaZ + step.simulationIz * lx  +  step.simulationJz * lz;
 			//currentLocalStepTime = (currentTime - step.startTime) / (step.endTime - step.startTime);
			currentLocalStepTime = sin((currentTime - step.startTime) / (step.endTime - step.startTime) * M_PI_OVER_2);
			//velocity = Vector(step.simulationIx, 0, step.simulationIz);
			velocity = Vector(step.simulationIx, 0, step.simulationIz) * step.desiredSpeed;
			position = start + (end - start) * (currentLocalStepTime * 0.5f);
			//position = start + (end - start) * currentLocalStepTime;
			return;
#endif
			break;
		case FOOTSTEP_STATE_BACKSTEP:
			currentLocalStepTime = (currentTime - step.startTime) - (0.5f*(step.endTime - step.startTime));
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


inline void drawFoot(float x, float z, float orientation, bool whichFoot, const float alpha=1.0f) {
	GLboolean lightingEnabled = glIsEnabled(GL_LIGHTING);

	glDisable (GL_LIGHTING);

	glPushAttrib(GL_CURRENT_BIT);
	glPushMatrix();

	if (alpha == 1.0f)
	{
		if (whichFoot==LEFT_FOOT)
			DrawLib::glColor(gYellow);
		else
			DrawLib::glColor(gCyan);
	}
	else // drawing a historical footstep
	{
		if (whichFoot==LEFT_FOOT)
			glColor4f (1.0f, 0.8f, 0.2f, alpha); // yellow
		else
			glColor4f (0.4f, 0.6f, 1.0f, alpha); // cyan
	}

	glTranslatef( x, 0.0f, z );
	glRotatef( -orientation * M_180_OVER_PI, 0.0f, 1.0f, 0.0f );
	//glScalef(0.05f, 0.05f, 0.05f);
	//DrawLib::drawCube();
	glScalef(0.2f, 0.0005f, 0.07f);
	//DrawLib::drawCube();
	//glTranslatef(0.2f, 0.0f, (whichFoot==LEFT_FOOT) ? -0.1f : 0.1f);
	DrawLib::drawSphere();

	// draw the arrow
	const float h = 0;
	glScalef(1/0.2f * 0.1f, 1/0.0005f, 1/0.07f * 0.1f); // undo the previous scale and scale the arrow
	glColor4f(0.3f, 0.3f, 0.3f, alpha);
	DrawLib::drawQuad(Util::Point(1.0f, h+.01f, 0.0f),
		Util::Point(-0.6f, h+0.01f, -0.5f),
		Util::Point(-0.62f, h+0.01f, 0.0f),
		Util::Point(-0.6f, h+0.01f, 0.5f));

	glPopMatrix();
	glPopAttrib();

	if (lightingEnabled)
		glEnable (GL_LIGHTING);
}



inline void drawFootstepTrajectory(const Footstep & step, const Util::Color & color, const float alpha=1.0f) {

	float increment = 0.01f * (step.endTime - step.startTime);

	if (increment < 0.00001f) {
		std::cerr << "WARNING - footstep had invalid time interval!\n";
		std::cerr << "problematic step was:" << step;
		return;
	}

	glPushAttrib(GL_CURRENT_BIT);
	glPushMatrix();

	if (alpha == 1.0)
		DrawLib::glColor(color);
	else
		glColor4f (color.r, color.g, color.b, alpha);

	glBegin(GL_LINE_STRIP);
	for (float t = step.startTime;  t <= step.endTime; t += increment) {
		Util::Point COMPos;
		Vector COMVel;
		evaluateParabolaAtTime(step, t, COMPos, COMVel);
		glVertex3f( COMPos.x, 0.0f, COMPos.z);
	}
	glEnd();

	glPopMatrix();
	glPopAttrib();
}


float FootstepAgent::determineFootstepOrientation(const Footstep & currentStep, const Footstep & previousStep)
{
	float currentPhi;

	if (currentStep.whichFoot==RIGHT_FOOT) {
		// inner > outer
		if (previousStep.simulationA > 1.7f) {
			currentPhi = currentStep.outerFootOrientationPhi;
		}
		else if ( !IN_INTERVAL_INCLUSIVE(currentStep.parabolaOrientationPhi, currentStep.outerFootOrientationPhi, currentStep.innerFootOrientationPhi) && currentStep.desiredSpeed > 0) {
			currentPhi = currentStep.parabolaOrientationPhi;

			//if (currentStep.desiredSpeed < 0)
			//	currentPhi += M_PI;
		} else {
			currentPhi = currentStep.innerFootOrientationPhi;
		}
	}
	else {
		// outer > inner
		if (previousStep.simulationA > 1.7f) {
			currentPhi = currentStep.outerFootOrientationPhi;
		}
		else if ( !IN_INTERVAL_INCLUSIVE(currentStep.parabolaOrientationPhi, currentStep.innerFootOrientationPhi, currentStep.outerFootOrientationPhi) && currentStep.desiredSpeed > 0) {
			currentPhi = currentStep.parabolaOrientationPhi;
		} else {
			if (currentStep.desiredSpeed < 0)
				currentPhi = currentStep.outerFootOrientationPhi;
			else
				currentPhi = currentStep.innerFootOrientationPhi;
		}
	}

	return currentPhi;
}

FootstepAgent::FootstepAgent()
{
	_enabled = false;
	_selectedLastDraw = false;
	_footstepCreationAttempts = 0;
}

FootstepAgent::~FootstepAgent()
{
	if (_enabled)
	{
		Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
		gSpatialDatabase->removeObject( dynamic_cast<SpatialDatabaseItemPtr>(this), bounds);
	}
}

SteerLib::EngineInterface * FootstepAgent::getSimulationEngine()
{
	return gEngine;
}

void FootstepAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	//
	//
	//  first, initialize  _walkParameters
	//
	//

	_walkParameters.centerOfMassHeight = default_com_height;
	_walkParameters.centerOfMassHeightInverse = 1.0f / default_com_height;
	_walkParameters.mass = default_mass;
	_walkParameters.massInverse = 1.0f/default_mass;
	_walkParameters.minStepLength = default_min_step_length;
	_walkParameters.maxStepLength = default_max_step_length;
	_walkParameters.minStepTime = default_min_step_time;
	_walkParameters.maxStepTime = default_max_step_time;
	_walkParameters.maxSpeed = default_max_speed;
	_walkParameters.baseRadius = default_base_radius;
	_walkParameters.preferredStepLength = 2.0f * _walkParameters.centerOfMassHeight * sinf(preferred_step_angle);

	_walkParameters.timeCostWeight = default_time_cost_weight;
	_walkParameters.trajectoryCostWeight = default_trajectory_cost_weght;

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

	float speedAfterLoss = _walkParameters.maxSpeed * cosf( 2.0f * preferred_step_angle);
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
	_last_position  = _position;
	_forward  = initialConditions.direction;
	_radius   = ped_torso_radius;  //initialConditions.radius;
	_velocity = initialConditions.speed * _forward;
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
		if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET||
				initialConditions.goals[i].goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL)
		{
			if (initialConditions.goals[i].targetIsRandom)
			{
				// if the goal is random, we must randomly generate the goal.
				// std::cout << "assigning random goal" << std::endl;
				SteerLib::AgentGoalInfo _goal;
				_goal.targetLocation = gSpatialDatabase->randomPositionWithoutCollisions(1.0f, true);
				_goalQueue.push(_goal);
				_currentGoal.targetLocation = _goal.targetLocation;
			}
			else
			{
				_goalQueue.push(initialConditions.goals[i]);
			}
			if (_goalQueue.size()==1) {
				_currentGoal = initialConditions.goals[i];
			}
		}
		else {
			throw Util::GenericException("Unsupported goal type; FootstepAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET and GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL.");
		}
	}


	assert(_forward.length()!=0.0f);
	assert(_goalQueue.size() != 0);
	// this assertion does not work with the new AgentGoalInfo struct; probably beacuse there is no == operator?
	// assert(_goalQueue.front() == _currentGoal);
	assert(_radius != 0.0f);

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

	/*_currentStep.targetX = _localTargetLocation.x;
	_currentStep.targetZ = _localTargetLocation.z;
		
	_currentStep.whichFoot = LEFT_FOOT;
	_currentStep.startTime = 0.0f;
	MTRand mtr((unsigned int)(initialConditions.position.x*initialConditions.position.z));
	_currentStep.endTime = (float)mtr.rand() * ped_initial_step_variation + 0.1f;
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
	_currentStep.outputCOMState.dx = (1.0f) * cos(_currentStep.parabolaOrientationPhi) - (.1f) * sin(_currentStep.parabolaOrientationPhi); 
	_currentStep.outputCOMState.dz = (.1f) * cos(_currentStep.parabolaOrientationPhi) + (1.0f) * sin(_currentStep.parabolaOrientationPhi); 
	_currentStep.isAGoalState = false;
	_currentStep.state = FOOTSTEP_STATE_NORMAL;
	_currentStep.phiIsIdeal = false;*/
	// TODO: should eventually ideally start stationary.
	//_currentStep.state = FOOTSTEP_STATE_STATIONARY;



#ifdef USE_OLD_FOOTSTEP_INIT
	// Old footstep initialization
	_currentStep.targetX = _localTargetLocation.x;
	_currentStep.targetZ = _localTargetLocation.z;

	_currentStep.whichFoot = LEFT_FOOT;
	_currentStep.startTime = 0.0f;
	MTRand mtr((unsigned int)(initialConditions.position.x*initialConditions.position.z));
	_currentStep.endTime = (float)mtr.rand() * ped_initial_step_variation + 0.1f;
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
	_currentStep.outputCOMState.dx = (1.0f) * cos(_currentStep.parabolaOrientationPhi) - (.1f) * sin(_currentStep.parabolaOrientationPhi); 
	_currentStep.outputCOMState.dz = (.1f) * cos(_currentStep.parabolaOrientationPhi) + (1.0f) * sin(_currentStep.parabolaOrientationPhi); 
	_currentStep.isAGoalState = false;
	_currentStep.state = FOOTSTEP_STATE_STARTING;
	_currentStep.phiIsIdeal = false;
#else
	_currentStep.targetX = _localTargetLocation.x;
	_currentStep.targetZ = _localTargetLocation.z;

	_currentStep.whichFoot = LEFT_FOOT;
	_currentStep.startTime = 0.0f;
	MTRand mtr((unsigned int)(initialConditions.position.x*initialConditions.position.z));
	_currentStep.endTime = (float)mtr.rand() * ped_initial_step_variation + 0.1f;
	//_currentStep.parabolaOrientationPhi = 0.0f * M_PI_OVER_180;
	//_currentStep.innerFootOrientationPhi = 0.0f * M_PI_OVER_180;
	//_currentStep.outerFootOrientationPhi = 0.0f * M_PI_OVER_180;
	_currentStep.parabolaOrientationPhi = atan2( normalize(initialConditions.direction).z, normalize(initialConditions.direction).x );
	_currentStep.innerFootOrientationPhi = atan2( normalize(initialConditions.direction).z, normalize(initialConditions.direction).x );
	_currentStep.outerFootOrientationPhi = atan2( normalize(initialConditions.direction).z, normalize(initialConditions.direction).x );
	_currentStep.parabolaX = initialConditions.position.x;
	_currentStep.parabolaZ = initialConditions.position.z;
	_currentStep.footX = initialConditions.position.x + _walkParameters.baseRadius * 1.5 * normalize(initialConditions.direction).z;
	_currentStep.footZ = initialConditions.position.z - _walkParameters.baseRadius * 1.5 * normalize(initialConditions.direction).x;
	_currentStep.energyCost = 0.0f;
	_currentStep.outputCOMState.x = initialConditions.position.x;
	_currentStep.outputCOMState.z = initialConditions.position.z;
	_currentStep.outputCOMState.dx = 0.0001 * normalize(initialConditions.direction).x;
	_currentStep.outputCOMState.dz = 0.0001 * normalize(initialConditions.direction).z;
	//_currentStep.outputCOMState.dx = 0.0001 * ((1.0f) * cos(_currentStep.parabolaOrientationPhi) - (.1f) * sin(_currentStep.parabolaOrientationPhi));
	//_currentStep.outputCOMState.dz = 0.0001 * ((.1f) * cos(_currentStep.parabolaOrientationPhi) + (1.0f) * sin(_currentStep.parabolaOrientationPhi)); 
	_currentStep.isAGoalState = false;
	_currentStep.state = FOOTSTEP_STATE_STATIONARY;
	_currentStep.phiIsIdeal = false;
#endif

	/*_currentStep.footX = _currentStep.outputCOMState.x + cos(_currentStep.parabolaOrientationPhi) * _walkParameters.baseRadius * 3;
	_currentStep.footZ = _currentStep.outputCOMState.z + sin(_currentStep.parabolaOrientationPhi) * _walkParameters.baseRadius * 3;
	_currentStep.innerFootOrientationPhi = _currentStep.outerFootOrientationPhi = _currentStep.parabolaOrientationPhi;
	_currentStep.parabolaOrientationPhi = _currentStep.parabolaOrientationPhi;
	//nextStep.innerFootOrientationPhi += parabolaOrientationPhi - previousStep.parabolaOrientationPhi;
	//nextStep.outerFootOrientationPhi += parabolaOrientationPhi - previousStep.parabolaOrientationPhi;
	_currentStep.outputCOMState.dx = cos(_currentStep.parabolaOrientationPhi) * 0.001;
	_currentStep.outputCOMState.dz = sin(_currentStep.parabolaOrientationPhi) * 0.001;*/



	_previousStep = _currentStep;

	/*_previousStep.footX = initialConditions.position.x - _walkParameters.baseRadius * normalize(initialConditions.direction).z;
	_previousStep.footZ = initialConditions.position.z + _walkParameters.baseRadius * normalize(initialConditions.direction).x;
	_previousStep.whichFoot = RIGHT_FOOT;*/

	_currentSimulatedPosition = Util::Point(_currentStep.outputCOMState.x, 0.0f, _currentStep.outputCOMState.z);
	_currentSimulatedVelocity = clamp(Vector(_currentStep.outputCOMState.dx, 0.0f, _currentStep.outputCOMState.dz),default_max_speed);

	_currentLocationInPath = 0;
	_footstepPlan.clear();
	_needToRunFootstepPlanner = true;
	planAge = 0;

	_stepHistory.clear();
	_stepHistory.push_back(_currentStep);

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



bool FootstepAgent::_createFootstepAction(float stepDuration, float parabolaOrientationPhi, bool phiIsIdeal, float desiredVelocity, const Footstep & previousStep, Footstep & nextStep, FootStateEnum nextState)
{
	_footstepCreationAttempts++;

	// std::cout << "trying angle: " << parabolaOrientationPhi << std::endl;

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

	//std::cerr << "=========================================\n";
	//std::cerr << "CREATING A FOOTSTEP:\n";
	//std::cerr << "=========================================\n";
	//std::cerr << "PREVIOUS STEP:\n    " << previousStep << "\n";
	//std::cerr << "INPUT OPTIONS:\n" << "  stepDuration = " << stepDuration << "\n  parabolaOrientationPhi = " << parabolaOrientationPhi << "\n  desiredVelocity = " << desiredVelocity << "\n\n";

	if (nextState == FOOTSTEP_STATE_STATIONARY) {
		assert((previousStep.state == FOOTSTEP_STATE_STATIONARY)||(previousStep.state == FOOTSTEP_STATE_STOPPING)||(previousStep.state == FOOTSTEP_STATE_HOPPING)||(previousStep.state == FOOTSTEP_STATE_INPLACETURNING));
		nextStep = previousStep;

		nextStep.startTime = previousStep.endTime;
		nextStep.endTime = nextStep.startTime + stepDuration;
		nextStep.state = FOOTSTEP_STATE_STATIONARY;
		nextStep.energyCost = _walkParameters.timeCostWeight * stepDuration;
		return true;
	}

	if (nextState == FOOTSTEP_STATE_INPLACETURNING) {
		//assert(previousStep.state == FOOTSTEP_STATE_STATIONARY);

		nextStep = previousStep;
		nextStep.startTime = previousStep.endTime;
		nextStep.endTime = nextStep.startTime + stepDuration;
		nextStep.state = FOOTSTEP_STATE_INPLACETURNING;

		nextStep.whichFoot = !previousStep.whichFoot;

		nextStep.energyCost = _walkParameters.timeCostWeight * stepDuration; // also add some cost for turning energy

		Vector footVec = Util::Point(nextStep.footX, 0, nextStep.footZ) - Util::Point(nextStep.outputCOMState.x, 0, nextStep.outputCOMState.z);
		float footDist = footVec.length();

		nextStep.footX = nextStep.outputCOMState.x + sin(parabolaOrientationPhi) * _walkParameters.baseRadius * 1.5 * (nextStep.whichFoot == LEFT_FOOT ? 1 : -1);
		nextStep.footZ = nextStep.outputCOMState.z - cos(parabolaOrientationPhi) * _walkParameters.baseRadius * 1.5 * (nextStep.whichFoot == LEFT_FOOT ? 1 : -1);
		nextStep.innerFootOrientationPhi = nextStep.outerFootOrientationPhi = parabolaOrientationPhi;
		nextStep.parabolaOrientationPhi = parabolaOrientationPhi;
		//nextStep.innerFootOrientationPhi += parabolaOrientationPhi - previousStep.parabolaOrientationPhi;
		//nextStep.outerFootOrientationPhi += parabolaOrientationPhi - previousStep.parabolaOrientationPhi;
		nextStep.outputCOMState.dx = cos(parabolaOrientationPhi) * 0.001;
		nextStep.outputCOMState.dz = sin(parabolaOrientationPhi) * 0.001;

		// cout << "turning choice" << endl;

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
	if (nextState == FOOTSTEP_STATE_NORMAL || nextState == FOOTSTEP_STATE_BACKSTEP) {
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
		/*if (previousStep.state == FOOTSTEP_STATE_STATIONARY)
		{
			velocityX = desiredVelocity * cos(parabolaOrientationPhi);
			velocityZ = desiredVelocity * sin(parabolaOrientationPhi);
		}
		else*/
		{
		
		velocityX = previousStep.outputCOMState.dx * desiredVelocity * prevSpeedInverse;
		velocityZ = previousStep.outputCOMState.dz * desiredVelocity * prevSpeedInverse;
		}
		//std::cerr << "Starting step, the VIRTUAL input velocity is = (" << velocityX  << ", " << velocityZ << ")\n";
	}
	else if (nextState == FOOTSTEP_STATE_HOPPING) {
		velocityX = previousStep.outputCOMState.dx * desiredVelocity * prevSpeedInverse;
		velocityZ = previousStep.outputCOMState.dz * desiredVelocity * prevSpeedInverse;
		nextStep.energyCost *= 1;
	}




	// 2. generate bases vectors for orientation based on orientationPhi

	// horizontal basis vector
	float ix = cos(parabolaOrientationPhi);
	float iz = sin(parabolaOrientationPhi);

	// vertical basis vector
	float jx, jz;  
	if (nextStep.whichFoot == LEFT_FOOT) {
		// THIS NEXT STEP WILL BE A LEFT FOOT
		// normal 2D coordinate system y-axis points down
		jx = -iz;
		jz = ix;
	}
	else {
		// THIS NEXT STEP WILL BE A RIGHT FOOT
		// inverted coordinate system, y-axis points up
		jx = iz;
		jz = -ix;
	}

	// negate horizontal basis if we're backstepping
	if (nextState == FOOTSTEP_STATE_BACKSTEP)
	{
		ix = -ix;
		iz = -iz;
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
			// break;
		case FOOTSTEP_STATE_BACKSTEP:
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
		case FOOTSTEP_STATE_HOPPING:
			timeDurationToOrigin = stepDuration * 0.5f;
			localX =  - timeDurationToOrigin * localDx;  // x starts on the negative side
			a = localDz / (2.0f * -timeDurationToOrigin);  // NOTE CAREFULLY: the real formula is y = a t^2 -->  dy = 2 a t --> a = dy/2t --> so the half and 2 cancel out.
			localZ = a*timeDurationToOrigin*timeDurationToOrigin;  // technically its (-halfStepDuration) * (-halfStepDuration)
			nextStep.parabolaX = previousStep.outputCOMState.x -  ix * localX  -  jx * localZ;
			nextStep.parabolaZ = previousStep.outputCOMState.z -  iz * localX  -  jz * localZ;
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
		//std::cerr << "####  FootstepAgent::_createFootstepAction() - invalid position condition for this desired step!\n";
		//assert(false);
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
	Util::Point COMStateLocation;
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


	if ((nextState == FOOTSTEP_STATE_NORMAL) || (nextState == FOOTSTEP_STATE_STOPPING) || (nextState == FOOTSTEP_STATE_BACKSTEP)) {
		// check if the distance from COM to foot position (origin) is within the proper step length walk parameters this character is allowed
		float stepDistanceSquared = (nextStep.footX-previousStep.footX)*(nextStep.footX-previousStep.footX) + (nextStep.footZ-previousStep.footZ)*(nextStep.footZ-previousStep.footZ);

		//std::cerr << "step distance is " << sqrtf(stepDistanceSquared) << "\n\n";
		if (stepDistanceSquared < _walkParameters.minStepLength*_walkParameters.minStepLength) {
			//std::cerr << "####  FootstepAgent::_createFootstepAction() - cannot create this step, it is too small!  " << sqrtf(stepDistanceSquared) << " < " << _walkParameters.minStepLength << "\n";
			return false;
		}

		if (stepDistanceSquared > _walkParameters.maxStepLength*_walkParameters.maxStepLength) {
			std::cerr << "####  FootstepAgent::_createFootstepAction() - cannot create this step, it is too large!  " << sqrtf(stepDistanceSquared) << " > " << _walkParameters.maxStepLength << "\n";
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
		if (nextStep.whichFoot == RIGHT_FOOT) {
			//
			// then THIS IS A RIGHT FOOT:
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
				// std::cerr << "TODO!!! this left foot is improperly constrained!! (fixing for now)\n";
				//nextStep.innerFootOrientationPhi = parabolaOrientationPhi;
				//nextStep.outerFootOrientationPhi = parabolaOrientationPhi;
				return false;
			}
		}
		else {
			//
			// then THIS IS A LEFT FOOT:
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

/*
	int enabled_count = 0;
	for (int ii = 0; ii < gEngine->getAgents().size(); ii++)
	{
		if (gEngine->getAgents().at(ii)->enabled() )
		{
			enabled_count++;
		}
	}

	if (enabled_count == 2)
	{
		std::cout << "next foot is: " << nextStep.whichFoot << std::endl;
	}
	*/
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
	bool willFaceCloseObstacle = false;

	double obstacleCollisionPercentage = 1.0;

	//for (float interp = 0.5f; interp <= 1.0f; interp += 0.49f) {
	//for (float interp = 0.33f; interp <= 1.0f; interp += 0.32f) {
	//for (float interp = 0.20f; interp <= 1.0f; interp += 0.19f) {
	for (float interp = 0.01f; interp <= 1.0f; interp += 0.09f) {
		Vector collisionForward, collisionVelocity;
		Util::Point collisionPosition;
		float collisionTime = nextStep.startTime + (stepDuration * interp);
		evaluateParabolaAtTime(nextStep, collisionTime, collisionPosition, collisionVelocity);
		collisionForward = normalize(collisionVelocity);
		Vector collisionRightSide = normalize(Vector(collisionForward.z, 0.0f, -collisionForward.x));

		// for every object in the character's visual field...
		// ++neighbor was changed to neighbor++ because _neighbors.end() is one past the elements in the set.
#ifdef _DEBUG
		// std::cout << " number of neighbors is " << _neighbors.size() << "\n";
		// _neighbors.
		int qz = 0;
#endif
		for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin();  neighbor != _neighbors.end();  neighbor++)
		{

#ifdef _DEBUG
	// std::cout << " obstacle is at " << *neighbor << " obstacles thus far are " << qz++ << "\n";
#endif
			if ((*neighbor)->isAgent())
			{
#ifdef _DEBUG
// std::cout << " number of agents is **\n";
#endif
				// collision procedure for other agents
				SteerLib::AgentInterface * agent;
				agent = dynamic_cast<SteerLib::AgentInterface*>(*neighbor);
				
				// check if the obstacle collides with the character's main body
				if (agent->collidesAtTimeWith(collisionPosition, collisionRightSide, _radius, collisionTime, nextStep.footX, nextStep.footZ ))
				{
					collisionWithAgentFound = true;
					break;
				}

			}
			else {
				// collision procedure for obstacles
				SteerLib::ObstacleInterface * obstacle;
				obstacle = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbor);

				// check if the obstacle collides with the character's main body
				// if (obstacle->overlaps(collisionPosition, _radius+0.348f)) // manually tested by Glen
				// if (obstacle->overlaps(collisionPosition, _radius+0.118f))
				if (obstacle->overlaps(collisionPosition, _radius))
				{
					collisionWithObstacleFound = true;
					obstacleCollisionPercentage = 1.0;
					// std::cout << "COM disk collision detected." << std::endl;

					const int radiusSegments = 10;

					for(int x = radiusSegments-1; x >= 0; x--)
					{
						if (obstacle->overlaps(collisionPosition, (_radius+0.348f) / radiusSegments * x))
						{
							obstacleCollisionPercentage = (1.0 - (double)x / radiusSegments);

							break;
						}
					}

					break;
				}

				// check if the obstacle collides with the character's shoulders.
				// for now, if mustRotateShoulders is already true, we can skip this check.
				if (( obstacle->overlaps(collisionPosition + _radius*collisionRightSide, _radius+shoulder_comfort_zone_2)
						|| obstacle->overlaps(collisionPosition - _radius*collisionRightSide, _radius+shoulder_comfort_zone_2)))
				{
					collisionWithObstacleFound = true;
					// std::cout << "Shoulder collision object found." << std::endl;
					break;
				}

			}

		}
	}

	if (collisionWithAgentFound) {
		nextStep.energyCost += 10000.0f * _walkParameters.mass;
		// return false;
	}

	if (collisionWithObstacleFound) {
		nextStep.energyCost += 1000000.0f * _walkParameters.mass * obstacleCollisionPercentage;
		// return false;
	}
	// Check that at the end of each step there is geometrically enough room for another step
	/*
	Util::Vector collisionForward, collisionVelocity;
	Util::Util::Point collisionPosition;
	float collisionTime = nextStep.startTime + (stepDuration);
	evaluateParabolaAtTime(nextStep, collisionTime, collisionPosition, collisionVelocity);
	collisionForward = normalize(collisionVelocity);
	if ( this->_willFaceObstacleCheck(collisionPosition , collisionForward )  )
	{
		willFaceCloseObstacle = true;
		nextStep.energyCost += 1000000.0f * _walkParameters.mass * obstacleCollisionPercentage;
		return false;
	}*/

	if ((!collisionWithAgentFound) && (!collisionWithObstacleFound)) {
		if ( (nextStep.outputCOMState.x - _cachedFootstepOptions[1].outputCOMState.x)*(nextStep.outputCOMState.x - _cachedFootstepOptions[1].outputCOMState.x) + (nextStep.outputCOMState.z - _cachedFootstepOptions[1].outputCOMState.z)*(nextStep.outputCOMState.z - _cachedFootstepOptions[1].outputCOMState.z) < ped_reached_footstep_goal_threshold*ped_reached_footstep_goal_threshold) {
			Util::Vector collisionForward, collisionVelocity;
			Util::Point collisionPosition;
			float collisionTime = nextStep.startTime + (stepDuration);
			evaluateParabolaAtTime(nextStep, collisionTime, collisionPosition, collisionVelocity);
			collisionForward = normalize(collisionVelocity);
#ifdef ROBUST_FOOTSTEPS
			if ( this->_willFaceObstacleCheck(collisionPosition , collisionForward, _neighbors)  )
			{
				nextStep.energyCost += 100000.0f * _walkParameters.mass;
				// return false;
			}
			else
#endif
			{
				nextStep.isAGoalState = true;
				nextStep.energyCost = 0.0f; // so that this node is expanded next, an important speedup (near-optimal solution)
				// return false;
			}
		}

	}

	//std::cerr << "\nCREATED STEP:\n    " << nextStep << "\n";

	//while (nextStep.parabolaOrientationPhi < -M_PI)
	//	nextStep.parabolaOrientationPhi+=M_2_PI;
/*
	while (nextStep.outerFootOrientationPhi < -M_PI)
		nextStep.outerFootOrientationPhi+=M_2_PI;

	while (nextStep.innerFootOrientationPhi < -M_PI)
		nextStep.innerFootOrientationPhi+=M_2_PI;*/

	return true;

}

#ifdef ROBUST_FOOTSTEPS
bool FootstepAgent::_willFaceObstacleCheck(Util::Point pos, Util::Vector forward, std::set<SteerLib::SpatialDatabaseItemPtr> neighbours)
{
	Util::Ray right_shoulder_ray;
	Util::Ray left_shoulder_ray;
	Util::Ray front_ray;

	right_shoulder_ray.dir = forward*FORWARD_FOOTSTEP_BOX;// forward();
	left_shoulder_ray.dir = forward*FORWARD_FOOTSTEP_BOX;
	// front_ray.dir = Util::rotateInXZPlane(collisionForward*0.8f, M_PI_2);

	right_shoulder_ray.pos = pos +  Util::rotateInXZPlane(forward*shoulder_comfort_zone, -M_PI_2);
	left_shoulder_ray.pos = pos +  Util::rotateInXZPlane(forward*shoulder_comfort_zone, M_PI_2);
	front_ray.dir = Util::rotateInXZPlane(forward*(shoulder_comfort_zone*2.0f), M_PI_2);
	front_ray.pos = right_shoulder_ray.pos + right_shoulder_ray.dir;

	right_shoulder_ray.mint = 0;
	right_shoulder_ray.maxt = 0.99f;

	left_shoulder_ray.mint = 0;
	left_shoulder_ray.maxt = 0.99f;

	// std::cout << "Testing ray collision:" << std::endl;

	front_ray.mint = 0.0f;
	front_ray.maxt = 0.99f;
	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = neighbours.begin();  neighbor != neighbours.end();  neighbor++)
	{
		if (!(*neighbor)->isAgent())
		{
			SteerLib::ObstacleInterface * obstacle;
			obstacle = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbor);
			float right_t = 0, left_t = 0, front_t=0;

			if ((obstacle->intersects(right_shoulder_ray, right_t) && ((right_t < 1.0f) && (right_t > -1.001f))) ||
					(obstacle->intersects(left_shoulder_ray, left_t) && ((left_t < 1.0f) && (left_t > -1.001f))) ||
					(obstacle->intersects(front_ray, front_t) && ((front_t < 1.0f) && (front_t > -1.001f))))
			{
				// std::cout << "agent# " << this->id() << "Shoulder comfort zone ray intersecting object:" << std::endl;
				// std::cout << "left ray: " << left_shoulder_ray << " , right ray: " << right_shoulder_ray << ", front ray: " << front_ray << std::endl;
				// std::cout << "left ray length: " << left_shoulder_ray.dir.length() << " , right ray length: " << right_shoulder_ray.dir.length() << std::endl;
				// std::cout << "left_t: " << left_t << " right_t: " << right_t << ", front_t: " << front_t <<  std::endl;
				return true;

			}
		}
	}
	return false;
}
#endif



void FootstepAgent::_planFootstepsToShortTermGoal(bool useHardCodedPlan)
{
	if (!useHardCodedPlan) {
#ifdef _DEBUG
		cout << "running short-term planner" << endl;
#endif

		AutomaticFunctionProfiler profileThisFunction( &FootstepGlobals::gPhaseProfilers->footstepPlanningPhaseProfiler );
		AStarLite searchAlgorithm;

		Footstep goalState;
		goalState.outputCOMState.x = _localTargetLocation.x;
		goalState.outputCOMState.z = _localTargetLocation.z;

		_currentLocationInPath = 0;
		_footstepPlan.clear();
		_cachedFootstepOptions.clear();
		_currentStep.isAGoalState = false;
		if ( _currentStep.endTime < gEngine->getClock().getCurrentSimulationTime())
		{
			// std::cout << "Planning using an out of date step" << std::endl;
			// cout << "Current Time " << gEngine->getClock().getCurrentSimulationTime() <<
				// 	std::endl << _currentStep << std::endl;
			_currentStep.startTime = _currentStep.startTime + ( gEngine->getClock().getCurrentSimulationTime() - _currentStep.endTime);
			_currentStep.endTime = gEngine->getClock().getCurrentSimulationTime();

		}
		_cachedFootstepOptions.push_back(_currentStep);
		_cachedFootstepOptions.push_back(goalState);


		Util::PerformanceProfiler footstepPlanProfiler;
		footstepPlanProfiler.reset();
		footstepPlanProfiler.start();

		FootstepEnvironment footstepPlanningEnvironment(this);
		footstepPlanningEnvironment.setMaxNumNodesToExpand(max_nodes_to_expand);
		if ( !searchAlgorithm.findPath( footstepPlanningEnvironment, 0, 1 ) )
		{
			// std::cout << "******* Plan NOT found ********" << std::endl;
			// std::cout << goalState << std::endl;
		}
		else
		{
			// std::cout << "******* Plan found ********" << std::endl;
			// std::cout << goalState << std::endl;

		}

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

			//_needToRunFootstepPlanner = true;

			//float parabolaPhiStraight = atan2f(prevStep.outputCOMState.dz, prevStep.outputCOMState.dx);
			//if (!_createFootstepAction(1.0f, parabolaPhiStraight + 0.05f*radiansInterval, 0.6f, prevStep, nextStep, FOOTSTEP_STATE_STATIONARY) == true) {
			//}

		}
		else {
#ifdef _DEBUG
			std::cerr << "\nCOMPUTED PATH:\n";
			std::cout << searchAlgorithm.getPath().size() << endl;
#endif

			for (int i=searchAlgorithm.getPath().size()-1; i >= 0; i--) {
				// std::cerr << "path node " << i << " --> cachedNode[" << searchAlgorithm.getPath()[i] << "]\n";
				// std::cerr << _cachedFootstepOptions[searchAlgorithm.getPath()[i]] << "\n";
				_footstepPlan.push_back( searchAlgorithm.getPath()[i] );
				_cachedFootstepOptions[ searchAlgorithm.getPath()[i] ].targetX = _localTargetLocation.x;
				_cachedFootstepOptions[ searchAlgorithm.getPath()[i] ].targetZ = _localTargetLocation.z;
				if (_cachedFootstepOptions[ searchAlgorithm.getPath()[i] ].isAGoalState)
				{
					// std::cerr << "path node " << i << " --> cachedNode[" << searchAlgorithm.getPath()[i] << "]\n";
					// std::cerr << _cachedFootstepOptions[searchAlgorithm.getPath()[i]] << "\n";
					break;
				}
			}
#ifdef _DEBUG
			std::cerr << "  - expanded  " << footstepPlanningEnvironment._numNodesExpanded << " nodes\n";
			std::cerr << "  - generated " << _cachedFootstepOptions.size() << " nodes\n\n";
			std::cout << "current time: " << gTempCurrentTime << std::endl;
#endif

		}
		//_currentStep = _cachedFootstepOptions[_footstepPlan[_currentLocationInPath]];
	}
	else {

		std::cerr << " ##### WARNING: using hard-coded plan for testing!!!\n";

		std::cerr << "currentlocation in path = " << _currentLocationInPath << "\n";
		std::cerr << "footstep plan size was: " << _footstepPlan.size() << "\n";

		Footstep goalState;
		goalState.outputCOMState.x = _localTargetLocation.x;
		goalState.outputCOMState.z = _localTargetLocation.z;
		_currentLocationInPath = 0;
		_footstepPlan.clear();
		_cachedFootstepOptions.clear();
		_currentStep.isAGoalState = false;
		_cachedFootstepOptions.push_back(_currentStep);
		_cachedFootstepOptions.push_back(goalState);
		_footstepPlan.push_back(0);

		int successorID;
		Footstep nextStep, prevStep;
		prevStep = _currentStep;
		for (unsigned int step = 1; step < 14; step++) {
	
			float radiansInterval = (prevStep.whichFoot == LEFT_FOOT) ? (-M_PI_OVER_2) : (M_PI_OVER_2);
			float parabolaPhiStraight = atan2f(prevStep.outputCOMState.dz, prevStep.outputCOMState.dx);


			if (step == 1) {
				if (!_createFootstepAction(0.6f, parabolaPhiStraight + 0.05f*radiansInterval, false, 1.3f, prevStep, nextStep, FOOTSTEP_STATE_NORMAL) == true) {
					std::cerr << "footstep 1 not created.\n";
					break;
				}
			}
			else if (step == 2) {
				if (!_createFootstepAction(0.7f, parabolaPhiStraight + 0.05f*radiansInterval, false, 1.3f, prevStep, nextStep, FOOTSTEP_STATE_NORMAL) == true) {
					std::cerr << "footstep 2 not created.\n";
					break;
				}
			}
			else if (step == 3) {
				if (!_createFootstepAction(0.8f, parabolaPhiStraight + 0.05f*radiansInterval, false, 1.3f, prevStep, nextStep, FOOTSTEP_STATE_NORMAL) == true) {
					std::cerr << "footstep 3 not created.\n";
					break;
				}
			}
			else if (step == 10) {
				if (!_createFootstepAction(0.9f, parabolaPhiStraight + 0.05f*radiansInterval, false, 1.3f, prevStep, nextStep, FOOTSTEP_STATE_NORMAL) == true) {
					std::cerr << "footstep 3 not created.\n";
					break;
				}
			}
			else if (step == 4) {
				if (!_createFootstepAction(0.6f, parabolaPhiStraight + 0.05f*radiansInterval, false, 0.0f, prevStep, nextStep, FOOTSTEP_STATE_STOPPING) == true) {
					std::cerr << "footstep 4 not created.\n";
					break;
				}
			}
			else if (step == 5) {
				if (!_createFootstepAction(1.0f, parabolaPhiStraight + 0.05f*radiansInterval, false, 0.6f, prevStep, nextStep, FOOTSTEP_STATE_STATIONARY) == true) {
					std::cerr << "footstep 5 not created.\n";
					break;
				}
			}
			else if (step == 6) {
				if (!_createFootstepAction(1.0f, parabolaPhiStraight + 0.05f*radiansInterval, false, 0.6f, prevStep, nextStep, FOOTSTEP_STATE_STATIONARY) == true) {
					std::cerr << "footstep 6 not created.\n";
					break;
				}
			}
			else if (step == 7) {
				// NOTE this is minus radiansInterval just as a quick test, because whichFoot is the SAME as the previous foot.
				if (!_createFootstepAction(0.6f, parabolaPhiStraight - 0.25f*radiansInterval, false, 0.6f, prevStep, nextStep, FOOTSTEP_STATE_STARTING) == true) {
					std::cerr << "footstep 7 not created.\n";
					break;
				}
			}
			else if (step == 8) {
				if (!_createFootstepAction(0.6f, parabolaPhiStraight + 0.05f*radiansInterval, false, 1.3f, prevStep, nextStep, FOOTSTEP_STATE_NORMAL) == true) {
					std::cerr << "footstep 8 not created.\n";
					break;
				}
			}
			else if (step == 9) {
				if (!_createFootstepAction(0.6f, parabolaPhiStraight + 0.05f*radiansInterval, false, 1.3f, prevStep, nextStep, FOOTSTEP_STATE_NORMAL) == true) {
					std::cerr << "footstep 9 not created.\n";
					break;
				}
			}
			else if (step == 11) {
				if (!_createFootstepAction(0.9f, parabolaPhiStraight + 0.05f*radiansInterval, false, 0.0f, prevStep, nextStep, FOOTSTEP_STATE_STOPPING) == true) {
					std::cerr << "footstep 4 not created.\n";
					break;
				}
			}
			else if (step == 12) {
				if (!_createFootstepAction(0.7f, parabolaPhiStraight + 0.05f*radiansInterval, false, 0.0f, prevStep, nextStep, FOOTSTEP_STATE_STOPPING) == true) {
					std::cerr << "footstep 4 not created.\n";
					break;
				}
			}
			else if (step == 13) {
				if (!_createFootstepAction(0.8f, parabolaPhiStraight + 0.05f*radiansInterval, false, 0.0f, prevStep, nextStep, FOOTSTEP_STATE_STOPPING) == true) {
					std::cerr << "footstep 4 not created.\n";
					break;
				}
			}

			successorID = _cachedFootstepOptions.size();
			_cachedFootstepOptions.push_back(nextStep);
			_footstepPlan.push_back( successorID );

			prevStep = nextStep;
		}

		_needToRunFootstepPlanner = false;
	}


}


void FootstepAgent::_updateCharacterFollowingPath(float currentTime)
{
	AutomaticFunctionProfiler profileThisFunction( &FootstepGlobals::gPhaseProfilers->locomotionPhaseProfiler );

	// check if we should increment to the next step.
	if (_cachedFootstepOptions[_footstepPlan[_currentLocationInPath]].endTime < currentTime )
	{// replan before last step is taken (except when reaching goal). This is forward thinking in the case the last step is a poor situation
		if ( ((_currentLocationInPath == _footstepPlan.size()-1)) ||
#ifdef ROBUST_FOOTSTEPS
				((_currentLocationInPath == _footstepPlan.size()-2) && // (!_LocalTargetIsFinalGoal()) &&
						(!_cachedFootstepOptions[_footstepPlan[_footstepPlan.size()-1]].isAGoalState)) ||
#endif
				(_currentLocationInPath > ped_num_steps_before_forced_plan))
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
#ifdef _DEBUG
			std::cerr << "taking next step:\n     " << _currentStep;
#endif
		}
	}


	// follow the planned path
	evaluateParabolaAtTime(_currentStep, currentTime, _currentSimulatedPosition, _currentSimulatedVelocity);


	// update the character's location in the spatial database
	AxisAlignedBox oldBounds = AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	_position = _currentSimulatedPosition;
	// This normalization was causing issues at the beginning of simulations because _currentSimulatedVelocity.length() = 0 or NaN
	if (_currentSimulatedVelocity.length() > 0.0f)
	{
		_forward = normalize(_currentSimulatedVelocity);
	}
	else
	{
		// _forward = Vector(0, 0, 0);
	}
	AxisAlignedBox newBounds = AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
#ifdef _DEBUG_1
	std::cout << " about to updateObject() in _updateCharacterFollowingPath, radius = " << _radius << "\n";
#endif
	gSpatialDatabase->updateObject( dynamic_cast<SpatialDatabaseItemPtr>(this), oldBounds, newBounds);
}


void FootstepAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
	_last_position = position();
	gTempCurrentTime = timeStamp;

	AutomaticFunctionProfiler profileThisFunction( &FootstepGlobals::gPhaseProfilers->aiProfiler );

	if (frameNumber == 1)
	{
#ifdef _DEBUG_1
	std::cout << "Starting in frameNumber 1\n";
#endif
		_runLongTermPlanningPhase();
		_runMidTermPlanningPhase();
		_runShortTermPlanningPhase();
#ifdef _DEBUG_1
	std::cout << "Done in frameNumber 1\n";
	std::cout << "number of neighbors is " << _neighbors.size() << "\n";
#endif
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
	_velocity = ((position() - _last_position).length() / dt) * forward();
}

void FootstepAgent::draw()
{
#ifdef ENABLE_GUI
	if (!_enabled)
	{
		return;
	}

	AutomaticFunctionProfiler profileThisFunction( &FootstepGlobals::gPhaseProfilers->drawProfiler );

	/*
	if (gEngine->isAgentSelected(this) && _selectedLastDraw == false)
	{
		for (int i = 0; i < _stepHistory.size(); i++)
		{
			cout << "Step " << i << ":\n";
			cout << _stepHistory[i] << endl;
		}

		for (size_t i = 1; i < _footstepPlan.size(); ++i)
		{
			cout << "Planned Step " << i << std::endl;
			cout <<  _cachedFootstepOptions[_footstepPlan[i]] << std::endl;

		}


		cout << "Current step: at " <<  gEngine->getClock().getCurrentSimulationTime() <<"\n";
		cout << _currentStep << endl;

		cout << "waypoints left " << _waypoints.size() << std::endl;
		cout << *this << std::endl;
		//gEngine->unselectAgent(this);

		_selectedLastDraw = true;
	} else if (!gEngine->isAgentSelected(this))
	{
		_selectedLastDraw = false;
	}
	*/

	// draw positive axes (red is x, blue is z)
	//DrawLib::drawLine(  Util::Point( 0.0f, 0.0f, 0.0f ), Util::Point(100.0f, 0.0f, 0.0f) , gRed );
	//DrawLib::drawLine(  Util::Point( 0.0f, 0.0f, 0.0f ), Util::Point(0.0f, 0.0f, 100.0f) , gBlue );
	/*
	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin();  neighbor != _neighbors.end();  neighbor++)
	{
		if ( (*neighbor)->computePenetration(this->position(), this->_radius) > 0.0f)
		{
			//Util::DrawLib::drawStar(this->position(), Util::Vector(1,0,0), 1.34f, gRed);
			Util::DrawLib::drawStar(this->position(), Util::Vector(1,0,0), 0.34f, gRed);
		}
	}*/

#ifdef DRAW_FOOTSTEP_HISTORY
	// draw the footstep history
	for (std::vector<Footstep>::reverse_iterator step = _stepHistory.rbegin(), prevStep; step != _stepHistory.rend() && (step+1 != _stepHistory.rend()); ++step)
	{
		prevStep = step+1;

		float phi = determineFootstepOrientation(*step, *prevStep);
		const bool fade = true;
		const float timeScale = 0.08f;
		const float alpha = fade ? (1 - (gTempCurrentTime - step->startTime) * timeScale) : 1;

		if (alpha < 0)
			break;

		drawFoot( step->footX, step->footZ, phi, step->whichFoot, alpha);
		drawFootstepTrajectory(*step, Color(1,0,1), alpha);

		// draw COM velocity
		//DrawLib::drawLine( Util::Point(step->outputCOMState.x, 0, step->outputCOMState.z), Util::Point(step->outputCOMState.x, 0, step->outputCOMState.z) + Vector(step->outputCOMState.dx, 0, step->outputCOMState.dz) * 0.5f, gRed );
	}
#endif

#ifdef DRAW_FOOTSTEP_PLAN

	if (gEngine->isAgentSelected(this))
	{
		// draw the current footstep path (future footsteps)
		for (int i = 1; i < _footstepPlan.size(); ++i)
		{
			Footstep &step = _cachedFootstepOptions[_footstepPlan[i]];
			Footstep &prevStep = _cachedFootstepOptions[_footstepPlan[i-1]];

			float phi = determineFootstepOrientation(step, prevStep);
			const bool fade = true;
			const float timeScale = 0.08f;
			//const float alpha = fade ? (1 - (gTempCurrentTime - step.startTime) * timeScale) : 1;
			const float alpha = fade ? (1.0f - (float)i / _footstepPlan.size() / 1.4f) : 1;

			if (alpha < 0)
				break;

			drawFoot( step.footX, step.footZ, phi, step.whichFoot, alpha);
			drawFootstepTrajectory(step, Color(1,0,1), alpha);

			// draw COM velocity
			//DrawLib::drawLine( Util::Point(step->outputCOMState.x, 0, step->outputCOMState.z), Util::Point(step->outputCOMState.x, 0, step->outputCOMState.z) + Vector(step->outputCOMState.dx, 0, step->outputCOMState.dz) * 0.5f, gRed );
		}
	}

#endif

	// draw center of mass
	DrawLib::drawAgentDisc( _currentSimulatedPosition,  _currentSimulatedVelocity, 0.05f, gRed);

	float phi = determineFootstepOrientation(_currentStep, _previousStep);
	drawFoot( _currentStep.footX, _currentStep.footZ, phi, _currentStep.whichFoot);
	drawFootstepTrajectory(_currentStep, gGreen);

// #define SHOW_SHOULDER_COMFORT_ZONE 1

	// draw COM velocity
	//DrawLib::drawLine( Util::Point(_currentStep.outputCOMState.x, 0, _currentStep.outputCOMState.z), Util::Point(_currentStep.outputCOMState.x, 0, _currentStep.outputCOMState.z) + Vector(_currentStep.outputCOMState.dx, 0, _currentStep.outputCOMState.dz) * 0.5f, gRed );

	/*if (_cachedFootstepOptions.size() > 0)
	{
		// determine the other "active" foot based on what time it occurs
		float halfwayTime = (_currentStep.endTime + _currentStep.startTime) * 0.5f;
		if ((gTempCurrentTime >= halfwayTime) && (_currentLocationInPath < _footstepPlan.size()-1)) {
			float phi = FootstepAgent::determineFootstepOrientation(_cachedFootstepOptions[_footstepPlan[_currentLocationInPath+1]],_cachedFootstepOptions[_footstepPlan[_currentLocationInPath]]);
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
	}*/

	if (_previousStep.whichFoot != _currentStep.whichFoot)
	{
		drawFoot (_previousStep.footX, _previousStep.footZ, _previousStep.parabolaOrientationPhi, _previousStep.whichFoot);
		drawFootstepTrajectory(_previousStep, gBlack);
	}
	else
	{
		// search for the previous step for the opposite foot (if _previousStep uses the same foot as _currentStep)

		for (int i = _stepHistory.size()-1; i >= 0; i--)
		{
			Footstep &step = _stepHistory[i];

			if (step.whichFoot != _currentStep.whichFoot)
			{
				drawFoot (step.footX, step.footZ, step.parabolaOrientationPhi, step.whichFoot);
				drawFootstepTrajectory(step, gBlack);

				break;
			}
		}
	}

	if (gEngine->isAgentSelected(this))
	{

		// draw goal
		DrawLib::drawFlag( _currentGoal.targetLocation, gGreen, 1.2 );
		DrawLib::drawFlag( _localTargetLocation, gBlue, 1.0 );

		if (_cachedFootstepOptions.size() > 2) DrawLib::drawFlag( Util::Point(_cachedFootstepOptions[1].outputCOMState.x, 0.0f, _cachedFootstepOptions[1].outputCOMState.z), gYellow);


		// draw collision bounds
		// approx 30 cm from back to front
		// approx 60 cm side-to-side ??
		DrawLib::drawAgentDisc( _currentSimulatedPosition,  _currentSimulatedVelocity, _radius, gRed);
		// collisionPosition + _radius*collisionRightSide, _radius+shoulder_comfort_zone_2
		DrawLib::drawAgentDisc( _currentSimulatedPosition + _radius * rightSideInXZPlane(normalize(_currentSimulatedVelocity)),  _currentSimulatedVelocity, _radius+shoulder_comfort_zone_2, gRed);
		DrawLib::drawAgentDisc( _currentSimulatedPosition - _radius * rightSideInXZPlane(normalize(_currentSimulatedVelocity)),  _currentSimulatedVelocity, _radius+shoulder_comfort_zone_2, gRed);

		// draw a blue line to any objects the character sees
		// Corrected pointer arithmetic
		for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin();  neighbor != _neighbors.end();  neighbor++)
		{
			if ( (*neighbor)->isAgent())
			{
				SteerLib::AgentInterface * agent;
				agent = dynamic_cast<SteerLib::AgentInterface*>(*neighbor);
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
			Util::Point waypointStart;
			Util::Point waypointEnd;
			gSpatialDatabase->getLocationFromIndex(_midTermPath[nn],waypointStart);
			gSpatialDatabase->getLocationFromIndex(_midTermPath[nn+1],waypointEnd);
			DrawLib::drawLine(waypointStart, waypointEnd, gRed);
		}

#ifdef SHOW_SHOULDER_COMFORT_ZONE
	Util::Ray right_shoulder_ray;
	Util::Ray left_shoulder_ray;
	Util::Ray front_ray;


	right_shoulder_ray.dir = forward();
	left_shoulder_ray.dir = forward();

	right_shoulder_ray.mint = 0;
	right_shoulder_ray.maxt = FORWARD_FOOTSTEP_BOX;

	left_shoulder_ray.mint = 0;
	left_shoulder_ray.maxt = FORWARD_FOOTSTEP_BOX;

	// std::cout << "Testing ray collision:" << std::endl;

	front_ray.mint = 0.0f;
	front_ray.maxt = shoulder_comfort_zone*2.0f;

	right_shoulder_ray.pos = position() +  Util::rotateInXZPlane(forward()*shoulder_comfort_zone, -M_PI_2);
	left_shoulder_ray.pos = position() +  Util::rotateInXZPlane(forward()*shoulder_comfort_zone, M_PI_2);
	front_ray.dir = Util::rotateInXZPlane(forward(), M_PI_2);
	front_ray.pos = right_shoulder_ray.pos + (right_shoulder_ray.dir*FORWARD_FOOTSTEP_BOX);

	DrawLib::drawLine(right_shoulder_ray.pos, right_shoulder_ray.pos + (right_shoulder_ray.dir*FORWARD_FOOTSTEP_BOX),  gRed);
	DrawLib::drawLine(left_shoulder_ray.pos, left_shoulder_ray.pos + (left_shoulder_ray.dir*FORWARD_FOOTSTEP_BOX), gRed);
	DrawLib::drawLine(front_ray.pos, front_ray.pos + (front_ray.dir*front_ray.maxt), gRed);

	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin();  neighbor != _neighbors.end();  neighbor++)
	{
		if ( !(*neighbor)->isAgent())
		{ // check for obstacle ray intersection
			SteerLib::ObstacleInterface * obstacle;
			obstacle = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbor);
			float t_time = 0 ;
			if ( obstacle->intersects(right_shoulder_ray, t_time) )
			{
				Util::DrawLib::drawStar(right_shoulder_ray.pos + (right_shoulder_ray.dir * t_time ) , Util::Vector(1,0,0), 0.34f, gGreen);
			}
			if ( obstacle->intersects(left_shoulder_ray, t_time) )
			{
				Util::DrawLib::drawStar(left_shoulder_ray.pos + (left_shoulder_ray.dir * t_time ) , Util::Vector(1,0,0), 0.34f, gGreen);
			}
			if ( obstacle->intersects(front_ray, t_time) )
			{
				Util::DrawLib::drawStar(front_ray.pos + (front_ray.dir * t_time ) , Util::Vector(1,0,0), 0.34f, gGreen);
			}
		}
		else {
		}
	}


#endif
	}


	// highlights the character if there are collisions
	// COrrected pointer arithmetic
	/*
	for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin();  neighbor != _neighbors.end();  neighbor++)
	{
		if ( (*neighbor)->isAgent()) {
			SteerLib::AgentInterface * agent;
			agent = dynamic_cast<SteerLib::AgentInterface*>(*neighbor);
			// TODO: annotate collisions with other agents here
			Vector rightSide = normalize(Vector(_forward.z, _forward.y, -_forward.x));
			if (agent->collidesAtTimeWith(_position, rightSide, _radius, gTempCurrentTime, _currentStep.footX, _currentStep.footZ )) {
				DrawLib::drawHighlight(_currentSimulatedPosition,_currentSimulatedVelocity, 0.16f, gBlue);
				//std::cerr << "COLLISION FOUND AT TIME " << gTempCurrentTime << "\n";
			}
		}
		else {
			SteerLib::ObstacleInterface * obstacle;
			obstacle = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbor);
			Util::Point pos = _currentSimulatedPosition;
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
	*/
#endif
}



bool FootstepAgent::collidesAtTimeWith(const Util::Point & p1, const Vector & rightSide, float otherAgentRadius, float timeStamp, float otherAgentFootX, float otherAgentFootZ)
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

	Util::Point s1;
	Vector simulatedVel;

	evaluateParabolaAtTime(myStep, timeStamp, s1, simulatedVel);

	float distanceSquaredThreshold = (otherAgentRadius+_radius)*(otherAgentRadius+_radius);

	if (  (p1-s1).lengthSquared() < distanceSquaredThreshold ) {
		return true;
	}



	Util::Point p2, p3, p4, s2, s3, s4;
	Vector myRightSide = normalize(Vector(simulatedVel.z, 0.0f, -simulatedVel.x));

	p2 = p1 + otherAgentRadius*rightSide;
	p3 = p1 - otherAgentRadius*rightSide;
	p4 = Util::Point(otherAgentFootX, 0.0f, otherAgentFootZ);

	s2 = s1 + _radius*myRightSide;
	s3 = s1 - _radius*myRightSide;
	s4 = Util::Point(myStep.footX, 0.0f, myStep.footZ);



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
	if ((timeStamp >= halfwayTime) && (stepIndex < _footstepPlan.size()-1))
	{
		// check with the foot in front
		Util::Point s5 = Util::Point(_cachedFootstepOptions[_footstepPlan[stepIndex+1]].footX, 0.0f, _cachedFootstepOptions[_footstepPlan[stepIndex+1]].footZ);
		if ((  (p1-s5).lengthSquared() < distanceSquaredThreshold ) ||
			(  (p2-s5).lengthSquared() < distanceSquaredThreshold ) ||
			(  (p3-s5).lengthSquared() < distanceSquaredThreshold ) ||
			(  (p4-s5).lengthSquared() < distanceSquaredThreshold )) {
				return true;
		}
	}
	else if ((timeStamp < halfwayTime) && (stepIndex > 0))
	{
		// check with the foot behind
		Util::Point s5 = Util::Point(_cachedFootstepOptions[_footstepPlan[stepIndex-1]].footX, 0.0f, _cachedFootstepOptions[_footstepPlan[stepIndex-1]].footZ);
		if ((  (p1-s5).lengthSquared() < distanceSquaredThreshold ) ||
			(  (p2-s5).lengthSquared() < distanceSquaredThreshold ) ||
			(  (p3-s5).lengthSquared() < distanceSquaredThreshold ) ||
			(  (p4-s5).lengthSquared() < distanceSquaredThreshold )) {
				return true;
		}
	}
	else
	{
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





void FootstepAgent::addGoal(const SteerLib::AgentGoalInfo & newGoal) { 
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
bool FootstepAgent::_reachedCurrentGoal()
{
	#ifdef _DEBUG_1
	std::cout << "in reachedCurrentGoal\n";
	std::cout << "_currentGoal.targetLocation-_position).lengthSquared() = " << (_currentGoal.targetLocation-_position).lengthSquared() << "\n";
	// std::cout << (ped_reached_target_distance_threshold * ped_reached_target_distance_threshold)
	#endif
	SteerLib::AgentGoalInfo goalInfo = _goalQueue.front();
	return ( (_currentGoal.targetLocation-_position).lengthSquared() < (ped_reached_target_distance_threshold * ped_reached_target_distance_threshold) ||
			(goalInfo.goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL &&
								Util::boxOverlapsCircle2D(goalInfo.targetRegion.xmin, goalInfo.targetRegion.xmax,
										goalInfo.targetRegion.zmin, goalInfo.targetRegion.zmax, this->position(), this->radius())
										));
}


//
// reachedCurrentWaypoint()
//
bool FootstepAgent::_reachedCurrentWaypoint()
{
	#ifdef _DEBUG_1
	std::cout << "in reachedCurrentWaypoint\n";
	std::cout << "(_waypoints[_currentWaypointIndex]-_position).lengthSquared() = " << (_waypoints[_currentWaypointIndex]-_position).lengthSquared() << "\n";
	// std::cout << (ped_reached_target_distance_threshold * ped_reached_target_distance_threshold)
	#endif
	return ( (_waypoints[_currentWaypointIndex]-position()).lengthSquared() < (ped_reached_target_distance_threshold * ped_reached_target_distance_threshold) );
}



//
// reachedLocalTarget()
//
bool FootstepAgent::_reachedLocalTarget()
{
	#ifdef _DEBUG_1
	std::cout << "in reachedTarget\n";
	std::cout << "(_localTargetLocation-_position).lengthSquared() = " << (_localTargetLocation-_position).lengthSquared() << "\n";
	// std::cout << (ped_reached_target_distance_threshold * ped_reached_target_distance_threshold)
	#endif
	return ( (_localTargetLocation-_position).lengthSquared() < (ped_reached_target_distance_threshold * ped_reached_target_distance_threshold) );
}

bool FootstepAgent::_LocalTargetIsFinalGoal()
{
	return ( (_localTargetLocation-_currentGoal.targetLocation).lengthSquared() < (ped_reached_target_distance_threshold * ped_reached_target_distance_threshold) );

}


void FootstepAgent::disable()
{
	_disable();
}
//
// disable()
//
void FootstepAgent::_disable()
{
	// if we tried to disable a second time, most likely we accidentally ignored that it was disabled, and should catch that error.
	// assert(_enabled==true);

	//  1. remove from database
#ifdef _DEBUG_1
	std::cout << "++++++++++++++++++" << " is being disabled\n";
#endif
	AxisAlignedBox b = AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	gSpatialDatabase->removeObject(dynamic_cast<SpatialDatabaseItemPtr>(this), b);

	//  2. set enabled = false
	_enabled = false;
}



//
// runCognitivePhase()
//
void FootstepAgent::_runCognitivePhase()
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
void FootstepAgent::_runLongTermPlanningPhase()
{
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

	AutomaticFunctionProfiler profileThisFunction( &FootstepGlobals::gPhaseProfilers->longTermPhaseProfiler );

	//==========================================================================
	int myIndexPosition = gSpatialDatabase->getCellIndexFromLocation(_position);
	int goalIndex = gSpatialDatabase->getCellIndexFromLocation(_currentGoal.targetLocation);

	if (myIndexPosition != -1) {
		// run the main a-star search here
		longTermAStar.findPath((*gEnvironmentForAStar), myIndexPosition, goalIndex);

		// set up the waypoints along this path.
		// if there was no path, then just make one waypoint that is the landmark target.
		_waypoints.clear();
		int longTermAStarLength = (int) longTermAStar.getPath().size();
		if (longTermAStarLength > 2)
		{
			// note the >2 condition: if the astar path is not at least this large, then there will be a behavior bug in the AI
			// when it tries to create waypoints.  in this case, the right thing to do is create only one waypoint that is at the landmark target.
			// remember the astar lib produces "backwards" paths that start at [pathLengh-1] and end at [0].
			int nextWaypointIndex = longTermAStarLength - 1 - next_waypoint_distance;
			while (nextWaypointIndex > 0) {
				Util::Point waypoint;
				gSpatialDatabase->getLocationFromIndex(longTermAStar.getPath()[nextWaypointIndex],waypoint);
				_waypoints.push_back(waypoint);
				nextWaypointIndex -= next_waypoint_distance;
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
void FootstepAgent::_runMidTermPlanningPhase()
{

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

	AutomaticFunctionProfiler profileThisFunction( &FootstepGlobals::gPhaseProfilers->midTermPhaseProfiler );

	// if we reached the current waypoint, then increment to the next waypoint
	if (_reachedCurrentWaypoint()) {
		_currentWaypointIndex++;
	}

	// compute a local a-star from your current location to the waypoint.
	int myIndexPosition = gSpatialDatabase->getCellIndexFromLocation(_position.x, _position.z);
	int waypointIndexPosition = gSpatialDatabase->getCellIndexFromLocation(_waypoints[_currentWaypointIndex].x, _waypoints[_currentWaypointIndex].z);
	if (!midTermAStar.findPath((*gEnvironmentForAStar), myIndexPosition, waypointIndexPosition))
	{
		std::cout << "Error! Midterm path not found " << std::endl;
	}

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
	if (_midTermPathSize > next_waypoint_distance + 3) {
		// the plus 1 is because a-star counts the agent's immediate location, but we do not.
		std::cerr << "ERROR!!!  _midTermPathSize is larger than expected: should be less than or equal to " << next_waypoint_distance+1 << ", but it actually is " << _midTermPathSize << "\n";
		std::cerr << "exiting ungracefully.\n";
		exit(1);
	}

	for (int i=0; i<_midTermPathSize; i++) {  // this used to be midTermPathSize-1     why???
		_midTermPath[i] = (midTermAStar.getPath())[i];
	}

	// TODO: should we reset the localTarget here??
}

//
// runShortTermPlanningPhase()
//
void FootstepAgent::_runShortTermPlanningPhase()
{
	int closestPathNode;
	if (!_enabled) return;

#ifdef IGNORE_PLANNING
	// if we want to ignore planning, then just decide to steer towards the final target.
	// possibly update the landmark target if we arrived at it.
	if (reachedCurrentGoal()) {
		runCognitivePhase();
	}
	_localTargetLocation = _currentGoal;
	return;
#endif

	// 0. if you're at your current waypoint
	if (_reachedCurrentWaypoint()) {
		// then schedule midTermPlanning phase
		_runMidTermPlanningPhase();
		if (!_enabled) return;
	}

	AutomaticFunctionProfiler profileThisFunction( &FootstepGlobals::gPhaseProfilers->shortTermPhaseProfiler );

	Vector rightSide = Vector(_forward.z, _forward.y, -_forward.x);

	int myIndexPosition = gSpatialDatabase->getCellIndexFromLocation(_position.x, _position.z);
#ifdef _DEBUG_1
	std::cout << "_position.x = " << _position.x << ", _position.z = " << _position.z << "\n";
#endif
	closestPathNode = _midTermPathSize; // the last index in the path array  *** PLUS ONE ***
	if ((myIndexPosition != -1)&&(closestPathNode != 0))// myIndexPosition is uninitialized
	{

		// 1. find the node that you're nearest to in your current path
		// NOTE that we MUST search ALL nodes of the path here.
		// reason: the path may unintuitively snake around so that some nodes are closer than others even if they are all very far from you.
		float minDistSquared = INFINITY;
		for (int i=_midTermPathSize-1; i >= 0; i--)
		{
			Util::Point tempTargetLocation;
#ifdef _DEBUG_1
	std::cout << "_midTermPath[" << i << "] = " << _midTermPath[i] << "\n";
#endif
			gSpatialDatabase->getLocationFromIndex( _midTermPath[i], tempTargetLocation);
			Vector temp = tempTargetLocation-_position;
			float distSquared = temp.lengthSquared();
			if (minDistSquared > distSquared) { // distSquared is uninitialized
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
			int furthestTargetIndex = max(0, closestPathNode - furthest_local_target_distance);
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
			localTargetIndex++; // the last node we found was actually NOT visible, so backtrack by one. // this could cause a crash
			if (localTargetIndex <= closestPathNode) {
				// if localTargetIndex is valid
				localTarget = _midTermPath[localTargetIndex];
				gSpatialDatabase->getLocationFromIndex( localTarget, _localTargetLocation );
				if ((_localTargetLocation - _waypoints[_currentWaypointIndex]).length() < 2.0f * ped_reached_target_distance_threshold) {
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
void FootstepAgent::_runPerceptivePhase()
{
	if (!_enabled) return;

	AutomaticFunctionProfiler profileThisFunction( &FootstepGlobals::gPhaseProfilers->perceptivePhaseProfiler );

	_neighbors.clear();

	//gSpatialDatabase->getItemsInVisualField(_neighbors, _position.x-ped_query_radius, _position.x+ped_query_radius,
	//	_position.z-ped_query_radius, _position.z+ped_query_radius, dynamic_cast<SpatialDatabaseItemPtr>(this),
	//	_position, _forward, (float)(ped_query_radius*ped_query_radius));

	gSpatialDatabase->getItemsInRange(_neighbors, _position.x-ped_query_radius, _position.x+ped_query_radius,
		_position.z-ped_query_radius, _position.z+ped_query_radius, dynamic_cast<SpatialDatabaseItemPtr>(this));
#ifdef _DEBUG_1
	std::cout << "Updating _neighbors in _runPerceptivePhase ...***********************\n";
	std::cout << "landmarkqueue size = " << _goalQueue.size() << "\n";
	std::cout << "number of neighbors is " << _neighbors.size() << "\n";
#endif
}

bool FootstepAgent::overlaps(const SteerLib::SpatialDatabaseItemPtr item)
{
#ifdef ROBUST_FOOTSTEPS
	Vector collisionRightSide = normalize(Vector(this->_forward.z, 0.0f, -this->_forward.x));
	if ( item->isAgent() )
	{
		AgentInterface * agent = dynamic_cast<AgentInterface*>(item);
		// if (agent->collidesAtTimeWith(this->_position, collisionRightSide, _radius, 0.0, nextStep.footX, nextStep.footZ ))
		{
			// return true;
		}
		std::cout << "checking for intersection with : " << *agent << std::endl;
		return this->overlaps(agent->position(), 0.5f);
	}
	else
	{
		SteerLib::ObstacleInterface * obstacle = dynamic_cast<SteerLib::ObstacleInterface*>(item);
		std::set<SteerLib::SpatialDatabaseItemPtr> neighbours;
		neighbours.insert(obstacle);
		// check if the obstacle collides with the character's main body
		// if (obstacle->overlaps(collisionPosition, _radius+0.348f)) // manually tested by Glen
		// if (obstacle->overlaps(collisionPosition, _radius+0.118f))
		if (obstacle->overlaps(this->_position, _radius))
		{
			return true;
		}
		// check if the obstacle collides with the character's shoulders.
		// for now, if mustRotateShoulders is already true, we can skip this check.
		if (( obstacle->overlaps(this->_position + _radius*collisionRightSide, _radius+shoulder_comfort_zone_2)
				|| obstacle->overlaps(this->_position - _radius*collisionRightSide, _radius+shoulder_comfort_zone_2)))
		{
			return true;
		}
		return ( obstacle->overlaps(this->position(), 0.5f) ||
		this->_willFaceObstacleCheck(this->position(), this->forward(), neighbours ) );
	}
#endif
	return false;
}
