//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

//d
// Copyright (c) 2009-2010 Shawn Singh, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include "SteerLib.h"
#include "SpaceTimeAgent.h"
#include "SpaceTimeAIModule.h"
#include "SpaceTimePlanningDomain.h"
#include "planning/BestFirstSearchPlanner.h"
#include "griddatabase/GridDatabasePlanningDomain.h"
#include <iostream>
//#include <math.h>
//#include <cmath>

using namespace SteerLib;

/// @file SpaceTimeAgent.cpp
/// @brief Implements the SpaceTimeAgent class.

#define MAX_FORCE_MAGNITUDE 3.0f
#define MAX_SPEED 1.0f
#define AGENT_MASS 1.0f
#define GUARANTEED_PLAN_DURATION 80
#define GUARANTEED_PLAN_BUFFER_DURATION 40

bool showGuaranteedPlan = true;
bool showLongTermPath = true;

double globalTime = 0;

SpaceTimeAgent::SpaceTimeAgent()
{
	double framesPerSecond = 1;
	guaranteedPlanSize = GUARANTEED_PLAN_DURATION*framesPerSecond;
	guaranteedPlanBufferSize = GUARANTEED_PLAN_BUFFER_DURATION*framesPerSecond;
	guaranteedPlanIndex = guaranteedPlanBufferSize;
	planReachedGoal = false;
	maxNumNodesToExpand = INT_MAX;
	useHermiteInterpolation = false;
	noPathFound = false;
	_enabled = false;
}

SpaceTimeAgent::~SpaceTimeAgent()
{
	if (_enabled) {
		Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
		gSpatialDatabase->removeObject( this, bounds);
	}
}

void SpaceTimeAgent::init(int maxNumNodesToExpand, bool useHermiteInterpolation)
{
	this->maxNumNodesToExpand = 400000;	// maxNumNodesToExpand;
	this->useHermiteInterpolation = true;	//useHermiteInterpolation;
}

void SpaceTimeAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	// compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
	// because the value is not used in that case.
	Util::AxisAlignedBox oldBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);

	// initialize the agent based on the initial conditions
	_position = initialConditions.position;
	//_position.y = 10;
	_forward = initialConditions.direction;
	_radius = initialConditions.radius;
	_velocity = initialConditions.speed * Util::normalize(initialConditions.direction);

	// compute the "new" bounding box of the agent
	Util::AxisAlignedBox newBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);

	if (!_enabled) {
		// if the agent was not enabled, then it does not already exist in the database, so add it.
		gSpatialDatabase->addObject( this, newBounds);
	}
	else {
		// if the agent was enabled, then the agent already existed in the database, so update it instead of adding it.
		gSpatialDatabase->updateObject( this, oldBounds, newBounds);
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
				_goalQueue.back().targetLocation = gSpatialDatabase->randomPositionWithoutCollisions(1.0f, true);
			}
		}
		else {
			throw Util::GenericException("Unsupported goal type; SpaceTimeAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET.");
		}
	}

	assert(_forward.length()!=0.0f);
	assert(_goalQueue.size() != 0);
	assert(_radius != 0.0f);
}


void SpaceTimeAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
	if(noPathFound) {
		return;
	}
	std::stack< SpaceTimePoint > longTermPath;
	longTermPath.empty();

	Util::Vector vectorToGlobalGoal = _goalQueue.front().targetLocation - _position;
	
	if(vectorToGlobalGoal.length() <= _radius) {
		_goalQueue.pop();
		Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
		gSpatialDatabase->removeObject( this, bounds);
		_enabled = false;
		guaranteedPlan.clear();
		return;
	}
	if (guaranteedPlanIndex == guaranteedPlanBufferSize && !planReachedGoal) {
		guaranteedPlanIndex = 0;
		SpaceTimePlanningDomain test(gSpatialDatabase, this);
		SpaceTimePoint start;
		if(guaranteedPlan.size() == 0) {
			start.point = Point(_position.x, 0, _position.z);
			start.time = 0;
		} else {
			start.point = guaranteedPlan[guaranteedPlanSize + guaranteedPlanBufferSize - 1].position;
			start.time = guaranteedPlan[guaranteedPlanSize + guaranteedPlanBufferSize - 1].time;
		}
		SpaceTimePoint firstGoal;
		firstGoal.point = _goalQueue.front().targetLocation;
		BestFirstSearchPlanner<SpaceTimePlanningDomain, SpaceTimePoint> spaceTimeAStarPlanner;
		spaceTimeAStarPlanner.init(&test, maxNumNodesToExpand);
		spaceTimeAStarPlanner.computePlan(start, firstGoal, longTermPath);
		
		longTermPathForDrawing = longTermPath;
		std::vector<SpaceTimePoint> longTermPathVector;
		while(longTermPath.size() > 0) {
			longTermPathVector.push_back(longTermPath.top());
			longTermPath.pop();
		}
		if ((longTermPathVector[longTermPathVector.size()-1].point - firstGoal.point).length() > _radius) {
			printf("COULD NOT FIND PATH TO GOAL---------------------------------------------------\n");
			noPathFound = true;
			return;
		}

		
		int startIndex;
		if(guaranteedPlan.size() == 0) {
			startIndex = 1;
			AgentState firstState = {_position, _velocity, _forward, 0};
			guaranteedPlan.push_back(firstState);
		} else {
			startIndex = guaranteedPlanSize;
			guaranteedPlan.erase(guaranteedPlan.begin(), guaranteedPlan.begin() + guaranteedPlanBufferSize);
		}
		double timeSteps = 20;
		if (guaranteedPlan.size() < 1) {
			printf("SHOULD NEVER HAVE SIZE < 1\n");
			exit(1);
		}
		
		Util::Point a,b;
		Util::Vector aTangent, bTangent;
		bool done = false;
		for (int i = 0; i < longTermPathVector.size() - 1; i++) {
			for (int j = 0; j < timeSteps; j++) {
				Point tempPosition = _position;
				Vector tempVelocity = _velocity;
				Vector tempForward = _forward;
				a = longTermPathVector[i].point;
				b = longTermPathVector[i+1].point;
				if (i == 0) {
					if (startIndex == 1) {
						aTangent = normalize(_forward);
					} else {
						aTangent = guaranteedPlan[startIndex-1].forward;
					}
				} else {
					if ((b-a).length() < 0.0001) {	// if stopped
						aTangent = Util::Vector(0, 0, 0);
					} else {
						aTangent = (b - longTermPathVector[i-1].point)/2.0;
					}
				}
				if (i == longTermPathVector.size() - 2) {
					bTangent = (b - a);
				} else {
					if ((longTermPathVector[i+2].point - b).length() < 0.0001) {	// if approaching a stop
						bTangent = Util::Vector(0, 0, 0);
					} else {
						bTangent = (longTermPathVector[i+2].point - a)/2.0;
					}
				}
				if ((b-a).length() > 0.0001) {
					if (useHermiteInterpolation) {
						_position = HermitePosition(a, aTangent, b, bTangent, (j+1)/timeSteps);
						_velocity = HermiteVelocity(a, aTangent, b, bTangent, (j+1)/timeSteps);
					} else {
						_position = a + (j+1)/timeSteps*(b-a);
						_velocity = b - a;

					}
					if (_velocity.length() < 0.0001) {
						_forward = guaranteedPlan[guaranteedPlan.size()-1].forward;
					} else {
						_forward = normalize(_velocity);
					}
				} else {
					_position = guaranteedPlan[guaranteedPlan.size()-1].position;
					_velocity = guaranteedPlan[guaranteedPlan.size()-1].velocity;
					if (bTangent.length() < 0.0001) {
						_forward = guaranteedPlan[guaranteedPlan.size()-1].forward;
					} else {
						_forward = normalize(guaranteedPlan[guaranteedPlan.size()-1].forward + (j+1)/timeSteps*(bTangent-guaranteedPlan[guaranteedPlan.size()-1].forward));
						if (!_finite(_forward.length())) {
							_forward = guaranteedPlan[guaranteedPlan.size()-1].forward;
						}
					}
				}

				AgentState nextState = {_position, _velocity, _forward, guaranteedPlan[guaranteedPlan.size()-1].time + 1/timeSteps};
				guaranteedPlan.push_back(nextState);

				_position = tempPosition;
				_velocity = tempVelocity;
				_forward = tempForward;

				if (guaranteedPlan.size() >= guaranteedPlanSize + guaranteedPlanBufferSize) {
					done = true;
					break;
				}
			}
			if (done) {
				break;
			}
		}

		Util::Vector vectorFromPlanEndToGlobalGoal = _goalQueue.front().targetLocation - guaranteedPlan[guaranteedPlan.size()-1].position;
		if(vectorFromPlanEndToGlobalGoal.length() < _radius) {
			planReachedGoal = true;
		}
	}

	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);

	if(guaranteedPlanIndex >= guaranteedPlan.size()) {
		return;
		printf("SHOULD NEVER GET HERE\n");
		exit(1);
	}
	_position = guaranteedPlan[guaranteedPlanIndex].position;
	_velocity = guaranteedPlan[guaranteedPlanIndex].velocity;
	_forward = guaranteedPlan[guaranteedPlanIndex].forward;
	
	globalTime += dt;

	Util::AxisAlignedBox newBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	gSpatialDatabase->updateObject( this, oldBounds, newBounds);
	guaranteedPlanIndex++;

	return;
}

Util::Point SpaceTimeAgent::HermitePosition(Util::Point a, Util::Vector aTangent, Util::Point b, Util::Vector bTangent, double s){
	Util::Point result;

	for (int i = 0; i < 3; i++) {
		result[i] = (2*a[i] - 2*b[i] + aTangent[i] + bTangent[i])*s*s*s;
		result[i] += (-3*a[i] + 3*b[i] - 2*aTangent[i] - bTangent[i])*s*s;
		result[i] += aTangent[i]*s;
		result[i] += a[i];
	}

	return result;
}

Util::Vector SpaceTimeAgent::HermiteVelocity(Util::Point a, Util::Vector aTangent, Util::Point b, Util::Vector bTangent, double s){
	Util::Vector result;

	for (int i = 0; i < 3; i++) {
		result[i] += (6*a[i] - 6*b[i] + 3*aTangent[i] + 3*bTangent[i])*s*s;
		result[i] += (-6*a[i] + 6*b[i] - 4*aTangent[i] - 2*bTangent[i])*s;
		result[i] += aTangent[i];
	}

	return result;
}

Util::Point SpaceTimeAgent::Bezier(Util::Point a, Util::Point b, Util::Point c, Util::Point d, double s) {
	Util::Point result;


	return result;
}


void SpaceTimeAgent::draw()
{
#ifdef ENABLE_GUI
	// if the agent is selected, do some annotations just for demonstration
	Point position2D = Point(_position.x, 0, _position.z);
	Vector forward2D = Vector(_forward.x, 0, _forward.z);
	if (gEngine->isAgentSelected(this)) {
		Util::Ray ray;
		ray.initWithUnitInterval(position2D, forward2D);
		float t = 0.0f;
		SteerLib::SpatialDatabaseItem * objectFound;
		Util::DrawLib::drawLine(ray.pos, ray.eval(1.0f));
		if (gSpatialDatabase->trace(ray, t, objectFound, this, false)) {
			Util::DrawLib::drawAgentDisc(position2D, forward2D, _radius, Util::gBlue);
		}
		else {
			Util::DrawLib::drawAgentDisc(position2D, forward2D, _radius);
		}
	}
	else {
		Util::DrawLib::drawAgentDisc(position2D, forward2D, _radius, Util::gGray40);
	}
	if (_goalQueue.front().goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
		Util::DrawLib::drawFlag(_goalQueue.front().targetLocation);
	}
	float lineWidth;
	glGetFloatv(GL_LINE_WIDTH, &lineWidth);
	if(showGuaranteedPlan && guaranteedPlan.size() > 0) {
		Util::DrawLib::glColor(Util::gBlack);
		for(int i = 0; i < min(guaranteedPlanIndex, guaranteedPlan.size()-1); i++) {
			Point point1 = guaranteedPlan[i].position;
			point1.y = 0;
			Point point2 = guaranteedPlan[i+1].position;
			point2.y = 0;
			Util::DrawLib::drawLine(point1, point2);
		}
		if (gEngine->isAgentSelected(this)) {
			glLineWidth(10);
		}
		Util::DrawLib::glColor(Util::gGreen);
		for(int i = guaranteedPlanIndex; i < min(guaranteedPlanIndex-1+guaranteedPlanSize, guaranteedPlan.size()-1); i++) {
			Point point1 = guaranteedPlan[i].position;
			point1.y = 0;
			Point point2 = guaranteedPlan[i+1].position;
			point2.y = 0;
			Util::DrawLib::drawLine(point1, point2);
		}
		Util::DrawLib::glColor(Util::gRed);
		for(int i = guaranteedPlanIndex-1+guaranteedPlanSize; i < min(guaranteedPlanSize+guaranteedPlanBufferSize-1, guaranteedPlan.size()-1); i++) {
			Point point1 = guaranteedPlan[i].position;
			point1.y = 0;
			Point point2 = guaranteedPlan[i+1].position;
			point2.y = 0;
			Util::DrawLib::drawLine(point1, point2);
		}
	}
	glLineWidth(lineWidth);
	if(showLongTermPath) {
		if (gEngine->isAgentSelected(this)) {
			Util::DrawLib::glColor(Util::gOrange);
		} else {
			Util::DrawLib::glColor(Util::gBlue);
		}
		std::stack<SpaceTimePoint> longTermPathForDrawingTemp = longTermPathForDrawing;
		int count = 0;
		while(longTermPathForDrawingTemp.size() > 1) {
			count++;
			Point point1 = longTermPathForDrawingTemp.top().point;
			point1.y = 0;
			longTermPathForDrawingTemp.pop();
			Point point2 = longTermPathForDrawingTemp.top().point;
			point2.y = 0;
			Util::DrawLib::drawLine(point1, point2);
			double radius = 0.1;
			Util::DrawLib::drawCylinder(point1, radius, 0, radius);
			/*
			for (int i = 0; i < count; i++) {
				Util::Point point = point1;
				point.x += (count/2.0 - i)*0.05;
				point.z += count*0.05;
				Util::DrawLib::drawCylinder(point, radius, 0, radius);
			}
			*/
		}
	}
#endif
}


void SpaceTimeAgent::_doEulerStep(const Util::Vector & steeringDecisionForce, float dt)
{
	// compute acceleration, _velocity, and newPosition by a simple Euler step
	const Util::Vector clippedForce = Util::clamp(steeringDecisionForce, MAX_FORCE_MAGNITUDE);
	Util::Vector acceleration = (clippedForce / AGENT_MASS);
	_velocity = _velocity + (dt*acceleration);
	_velocity = clamp(_velocity, MAX_SPEED);  // clamp _velocity to the max speed
	//Util::Point offset = Util::Point(0, 0.1, 0);
	const Util::Point newPosition = _position + (dt*_velocity);

	// For this simple agent, we just make the orientation point along the agent's current velocity.
	if (_velocity.lengthSquared() != 0.0f) {
		_forward = normalize(_velocity);
	}

	_position = newPosition;
}

Point* SpaceTimeAgent::GetLocationAtTime(double time)
{
	int start = guaranteedPlanIndex;
	int end = guaranteedPlan.size() - 1;
	double buffer = 0.00001;	//double equality comparisons need some flexibility sometimes...
	if (end > 0) {
		if (time >= guaranteedPlan[start].time - buffer) {
			if (time <= guaranteedPlan[end].time + buffer) {
				double ratio = (time - guaranteedPlan[start].time)/(guaranteedPlan[end].time - guaranteedPlan[start].time);
				int index = ratio * (end - start) + start + 0.5;
				return &(guaranteedPlan[index].position);
			} else {
				/*
				Util::Vector vectorToGlobalGoal = _goalQueue.front().targetLocation - guaranteedPlan[end].position;
				if(vectorToGlobalGoal.length() > _radius) {
					return &(guaranteedPlan[end].position);
				}
				*/
			}
		} else {
			printf("IS IT POSSIBLE FOR THIS TO BE TRUE?\n");
			//exit(1);
		}
	} else {
		//return &_position;
	}
	return NULL;
}


void SpaceTimeAgent::disable()
{
	  assert(_enabled == true);

	  Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
	  gSpatialDatabase->removeObject( this, bounds);

	  _enabled = false;
}
