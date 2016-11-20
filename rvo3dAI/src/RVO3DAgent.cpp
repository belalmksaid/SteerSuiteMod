//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#include "RVO3DAgent.h"
#include "RVO3DAIModule.h"
#include "KdTree.h"
#include "SteerLib.h"
#include "Definitions.h"
// #include "util/Geometry.h"

/// @file RVO3DAgent.cpp
/// @brief Implements the RVO3DAgent class.

#define MAX_FORCE_MAGNITUDE 3.0f
#define MAX_SPEED 1.3f
#define AGENT_MASS 1.0f
#undef min
#undef max

using namespace Util;

const float RVO_EPSILON = 0.00001f;

/**
 * \brief   Solves a one-dimensional linear program on a specified line subject to linear constraints defined by planes and a spherical constraint.
 * \param   planes        Planes defining the linear constraints.
 * \param   planeNo       The plane on which the line lies.
 * \param   line          The line on which the 1-d linear program is solved
 * \param   radius        The radius of the spherical constraint.
 * \param   optVelocity   The optimization velocity.
 * \param   directionOpt  True if the direction should be optimized.
 * \param   result        A reference to the result of the linear program.
 * \return  True if successful.
 */
bool linearProgram1(const std::vector<Plane> &planes, size_t planeNo, const Ray &line, float radius, const Vector &optVelocity, bool directionOpt, Vector &result);

/**
 * \brief   Solves a two-dimensional linear program on a specified plane subject to linear constraints defined by planes and a spherical constraint.
 * \param   planes        Planes defining the linear constraints.
 * \param   planeNo       The plane on which the 2-d linear program is solved
 * \param   radius        The radius of the spherical constraint.
 * \param   optVelocity   The optimization velocity.
 * \param   directionOpt  True if the direction should be optimized.
 * \param   result        A reference to the result of the linear program.
 * \return  True if successful.
 */
bool linearProgram2(const std::vector<Plane> &planes, size_t planeNo, float radius, const Vector &optVelocity, bool directionOpt, Vector &result);

/**
 * \brief   Solves a three-dimensional linear program subject to linear constraints defined by planes and a spherical constraint.
 * \param   planes        Planes defining the linear constraints.
 * \param   radius        The radius of the spherical constraint.
 * \param   optVelocity   The optimization velocity.
 * \param   directionOpt  True if the direction should be optimized.
 * \param   result        A reference to the result of the linear program.
 * \return  The number of the plane it fails on, and the number of planes if successful.
 */
size_t linearProgram3(const std::vector<Plane> &planes, float radius, const Vector &optVelocity, bool directionOpt, Vector &result);

/**
 * \brief   Solves a four-dimensional linear program subject to linear constraints defined by planes and a spherical constraint.
 * \param   planes     Planes defining the linear constraints.
 * \param   beginPlane The plane on which the 3-d linear program failed.
 * \param   radius     The radius of the spherical constraint.
 * \param   result     A reference to the result of the linear program.
 */
void linearProgram4(const std::vector<Plane> &planes, size_t beginPlane, float radius, Vector &result);

RVO3DAgent::RVO3DAgent()
{
	_enabled = false;
}

RVO3DAgent::~RVO3DAgent()
{
	if (_enabled) {
		Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
		gSpatialDatabase->removeObject( this, bounds);
	}
}

SteerLib::EngineInterface * RVO3DAgent::getSimulationEngine()
{
	return gEngine;
}

void RVO3DAgent::disable()
{
	// DO nothing for now
}

void RVO3DAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	// compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
	// because the value is not used in that case.
	Util::AxisAlignedBox oldBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);

	// initialize the agent based on the initial conditions
	_position = initialConditions.position;
	_forward = initialConditions.direction;
	_radius = initialConditions.radius;
	_velocity = initialConditions.speed * Util::normalize(initialConditions.direction);

	neighborDist_ = 15.0f;
	maxNeighbors_ = 10;
	timeHorizon_ = 10.0f;
	maxSpeed_ = 1.33f;

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
				SteerLib::AgentGoalInfo _goal;
				_goal.targetLocation = gSpatialDatabase->randomPositionWithoutCollisions(1.0f, true);
				_goalQueue.push(_goal);
			}
		}
		else {
			throw Util::GenericException("Unsupported goal type; RVO3DAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET.");
		}
	}

	assert(_forward.length()!=0.0f);
	assert(_goalQueue.size() != 0);
	assert(_radius != 0.0f);
}


void RVO3DAgent::computeNeighbors()
{
	agentNeighbors_.clear();

	if (maxNeighbors_ > 0) {
		// std::cout << "About to segfault" << std::endl;
		dynamic_cast<RVO3DAIModule *>(rvoModule)->kdTree_->computeAgentNeighbors(this, neighborDist_ * neighborDist_);
		// std::cout << "Made it past segfault" << std::endl;
	}
}

void RVO3DAgent::computeNewVelocity(float dt)
{
	orcaPlanes_.clear();
	const float invTimeHorizon = 1.0f / timeHorizon_;

	/* Create agent ORCA planes. */
	for (size_t i = 0; i < agentNeighbors_.size(); ++i) {
		const RVO3DAgent *const other = agentNeighbors_[i].second;
		const Vector relativePosition = other->_position - _position;
		const Vector relativeVelocity = _velocity - other->_velocity;
		const float distSq = (relativePosition).lengthSquared();
		const float combinedRadius = _radius + other->_radius;
		const float combinedRadiusSq = (combinedRadius*combinedRadius);

		Plane plane;
		Vector u;

		if (distSq > combinedRadiusSq) {
			/* No collision. */
			const Vector w = relativeVelocity - invTimeHorizon * relativePosition;
			/* Vector from cutoff center to relative velocity. */
			const float wLengthSq = (w).lengthSquared();

			const float dotProduct = dot(w, relativePosition);

			if (dotProduct < 0.0f && (dotProduct*dotProduct) > combinedRadiusSq * wLengthSq) {
				/* Project on cut-off circle. */
				const float wLength = std::sqrt(wLengthSq);
				const Vector unitW = w / wLength;

				plane.normal = unitW;
				u = (combinedRadius * invTimeHorizon - wLength) * unitW;
			}
			else {
				/* Project on cone. */
				const float a = distSq;
				const float b = dot(relativePosition, relativeVelocity);
				const float c = (relativeVelocity).lengthSquared() - (cross(relativePosition, relativeVelocity)).lengthSquared() /
						(distSq - combinedRadiusSq);
				const float t = (b + std::sqrt(sqr(b) - a * c)) / a;
				const Vector w = relativeVelocity - t * relativePosition;
				const float wLength = (w).length();
				const Vector unitW = w / wLength;

				plane.normal = unitW;
				u = (combinedRadius * t - wLength) * unitW;
			}
		}
		else {
			/* Collision. */
			const float invTimeStep = 1.0f / dt;
			const Vector w = relativeVelocity - invTimeStep * relativePosition;
			const float wLength = (w).length();
			const Vector unitW = w / wLength;

			plane.normal = unitW;
			u = (combinedRadius * invTimeStep - wLength) * unitW;
		}

		plane.point = (0.5f * Point(u.x, u.y, u.z)) + _velocity;
		orcaPlanes_.push_back(plane);
	}

	const size_t planeFail = linearProgram3(orcaPlanes_, maxSpeed_, prefVelocity_, false, newVelocity_);

	if (planeFail < orcaPlanes_.size()) {
		linearProgram4(orcaPlanes_, planeFail, maxSpeed_, newVelocity_);
	}
}

void RVO3DAgent::insertAgentNeighbor(const RVO3DAgent *agent, float &rangeSq)
{
	if (this != agent) {
		const float distSq = (_position - agent->_position).lengthSquared();

		if (distSq < rangeSq) {
			if (agentNeighbors_.size() < maxNeighbors_) {
				agentNeighbors_.push_back(std::make_pair(distSq, agent));
			}

			size_t i = agentNeighbors_.size() - 1;

			while (i != 0 && distSq < agentNeighbors_[i - 1].first) {
				agentNeighbors_[i] = agentNeighbors_[i - 1];
				--i;
			}

			agentNeighbors_[i] = std::make_pair(distSq, agent);

			if (agentNeighbors_.size() == maxNeighbors_) {
				rangeSq = agentNeighbors_.back().first;
			}
		}
	}
}

void RVO3DAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
	SteerLib::AgentGoalInfo goalInfo = _goalQueue.front();
	prefVelocity_ = normalize(goalInfo.targetLocation - position());
	prefVelocity_.y = 0.0f;
	std::cout << "Preferred velocity is: " << prefVelocity_ << std::endl;
	std::cout << "Position: " << this->position() << std::endl;
	_velocity = newVelocity_;
	_velocity.y = 0.0f;
	_forward = normalize(_velocity);
	_position = _position + (_velocity * dt);
}

void RVO3DAgent::update(float timeStamp, float dt, unsigned int frameNumber)
{
	// for this function, we assume that all goals are of type GOAL_TYPE_SEEK_STATIC_TARGET.
	// the error check for this was performed in reset().

	Util::Vector vectorToGoal = _goalQueue.front().targetLocation - _position;

	// it is up to the agent to decide what it means to have "accomplished" or "completed" a goal.
	// for the simple AI, if the agent's distance to its goal is less than its radius, then the agent has reached the goal.
	if (vectorToGoal.lengthSquared() < _radius * _radius) {
		_goalQueue.pop();
		if (_goalQueue.size() != 0) {
			// in this case, there are still more goals, so start steering to the next goal.
			vectorToGoal = _goalQueue.front().targetLocation - _position;
		}
		else {
			// in this case, there are no more goals, so disable the agent and remove it from the spatial database.
			Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
			gSpatialDatabase->removeObject( this, bounds);
			_enabled = false;
			return;
		}
	}

	std::cout << "New velocity is: " << newVelocity_ << std::endl;
	// use the vectorToGoal as a force for the agent to steer towards its goal.
	// the euler integration step will clamp this vector to a reasonable value, if needed.
	// also, the Euler step updates the agent's position in the spatial database.
	_doEulerStep(vectorToGoal, dt);

}


void RVO3DAgent::draw()
{
#ifdef ENABLE_GUI
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
#endif
}


void RVO3DAgent::_doEulerStep(const Util::Vector & steeringDecisionForce, float dt)
{
	// compute acceleration, _velocity, and newPosition by a simple Euler step
	const Util::Vector clippedForce = Util::clamp(steeringDecisionForce, MAX_FORCE_MAGNITUDE);
	Util::Vector acceleration = (clippedForce / AGENT_MASS);
	_velocity = _velocity + (dt*acceleration);
	_velocity = clamp(_velocity, MAX_SPEED);  // clamp _velocity to the max speed
	const Util::Point newPosition = _position + (dt*_velocity);

	// For this simple agent, we just make the orientation point along the agent's current velocity.
	if (_velocity.lengthSquared() != 0.0f) {
		_forward = normalize(_velocity);
	}

	// update the database with the new agent's setup
	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	Util::AxisAlignedBox newBounds(newPosition.x - _radius, newPosition.x + _radius, 0.0f, 0.0f, newPosition.z - _radius, newPosition.z + _radius);
	gSpatialDatabase->updateObject( this, oldBounds, newBounds);

	_position = newPosition;
}

bool linearProgram1(const std::vector<Util::Plane> &planes, size_t planeNo, const Ray &line, float radius, const Vector &optVelocity, bool directionOpt, Vector &result)
{
	const float dotProduct = dot(Vector(line.pos.x, line.pos.y, line.pos.z) , line.dir);
	// Possible point of math error
	// const float discriminant = sqr(dotProduct) + sqr(radius) - absSq(line.pos);
	const float discriminant = sqr(dotProduct) + sqr(radius) - Vector(line.pos.x, line.pos.y, line.pos.z).lengthSquared();

	if (discriminant < 0.0f)
	{
		/* Max speed sphere fully invalidates line. */
		return false;
	}

	const float sqrtDiscriminant = std::sqrt(discriminant);
	float tLeft = -dotProduct - sqrtDiscriminant;
	float tRight = -dotProduct + sqrtDiscriminant;

	for (size_t i = 0; i < planeNo; ++i)
	{
		const float numerator = dot((planes[i].point - line.pos), planes[i].normal);
		const float denominator = dot(line.dir, planes[i].normal);

		if (sqr(denominator) <= RVO_EPSILON) {
			/* Lines line is (almost) parallel to plane i. */
			if (numerator > 0.0f) {
				return false;
			}
			else {
				continue;
			}
		}

		const float t = numerator / denominator;

		if (denominator >= 0.0f) {
			/* Plane i bounds line on the left. */
			tLeft = std::max(tLeft, t);
		}
		else {
			/* Plane i bounds line on the right. */
			tRight = std::min(tRight, t);
		}

		if (tLeft > tRight) {
			return false;
		}
	}

	if (directionOpt) {
		/* Optimize direction. */
		if (dot(optVelocity, line.dir) > 0.0f) {
			/* Take right extreme. */
			result = (line.pos + (tRight * line.dir)).vector();
		}
		else {
			/* Take left extreme. */
			result = (line.pos + tLeft * line.dir).vector();
		}
	}
	else {
		/* Optimize closest point. */
		const float t = dot(line.dir, (line.pos - optVelocity ));

		if (t < tLeft) {
			result = (line.pos + tLeft * line.dir).vector();
		}
		else if (t > tRight) {
			result = (line.pos + tRight * line.dir).vector();
		}
		else {
			result = (line.pos + t * line.dir).vector();
		}
	}

	return true;
}

bool linearProgram2(const std::vector<Plane> &planes, size_t planeNo, float radius, const Vector &optVelocity, bool directionOpt, Vector &result)
{
	const float planeDist = dot(planes[planeNo].point, planes[planeNo].normal);
	const float planeDistSq = sqr(planeDist);
	const float radiusSq = sqr(radius);

	if (planeDistSq > radiusSq) {
		/* Max speed sphere fully invalidates plane planeNo. */
		return false;
	}

	const float planeRadiusSq = radiusSq - planeDistSq;

	// Weird I know
	const Vector planeCenter = planeDist * planes[planeNo].normal;

	if (directionOpt) {
		/* Project direction optVelocity on plane planeNo. */
		const Vector planeOptVelocity = optVelocity - dot(optVelocity, planes[planeNo].normal) * planes[planeNo].normal;
		const float planeOptVelocityLengthSq = (planeOptVelocity).lengthSquared();

		if (planeOptVelocityLengthSq <= RVO_EPSILON) {
			result = planeCenter;
		}
		else {
			result = planeCenter + std::sqrt(planeRadiusSq / planeOptVelocityLengthSq) * planeOptVelocity;
		}
	}
	else {
		/* Project point optVelocity on plane planeNo. */
		result = optVelocity + (dot((planes[planeNo].point - optVelocity), planes[planeNo].normal) * planes[planeNo].normal);

		/* If outside planeCircle, project on planeCircle. */
		if ((result).lengthSquared() > radiusSq) {
			const Vector planeResult = result - planeCenter;
			const float planeResultLengthSq = (planeResult).lengthSquared();
			result = planeCenter + std::sqrt(planeRadiusSq / planeResultLengthSq) * planeResult;
		}
	}

	for (size_t i = 0; i < planeNo; ++i) {
		if (dot(planes[i].normal, (planes[i].point - result)) > 0.0f) {
			/* Result does not satisfy constraint i. Compute new optimal result. */
			/* Compute intersection line of plane i and plane planeNo. */
			Vector crossProduct = cross(planes[i].normal, planes[planeNo].normal);

			if ((crossProduct).lengthSquared() <= RVO_EPSILON) {
				/* Planes planeNo and i are (almost) parallel, and plane i fully invalidates plane planeNo. */
				return false;
			}

			Ray line;
			line.dir = normalize(crossProduct);
			const Vector lineNormal = cross(line.dir, planes[planeNo].normal);
			line.pos = planes[planeNo].point + (dot((planes[i].point - planes[planeNo].point), planes[i].normal) / dot(lineNormal, planes[i].normal)) * lineNormal;

			if (!linearProgram1(planes, i, line, radius, optVelocity, directionOpt, result)) {
				return false;
			}
		}
	}

	return true;
}

size_t linearProgram3(const std::vector<Plane> &planes, float radius, const Vector &optVelocity, bool directionOpt, Vector &result)
{
	if (directionOpt) {
		/* Optimize direction. Note that the optimization velocity is of unit length in this case. */
		result = optVelocity * radius;
	}
	else if ((optVelocity).lengthSquared() > sqr(radius)) {
		/* Optimize closest point and outside circle. */
		result = normalize(optVelocity) * radius;
	}
	else {
		/* Optimize closest point and inside circle. */
		result = optVelocity;
	}

	for (size_t i = 0; i < planes.size(); ++i) {
		if (dot(planes[i].normal, (planes[i].point - result)) > 0.0f) {
			/* Result does not satisfy constraint i. Compute new optimal result. */
			const Vector tempResult = result;

			if (!linearProgram2(planes, i, radius, optVelocity, directionOpt, result)) {
				result = tempResult;
				return i;
			}
		}
	}

	return planes.size();
}

void linearProgram4(const std::vector<Plane> &planes, size_t beginPlane, float radius, Vector &result)
{
	float distance = 0.0f;

	for (size_t i = beginPlane; i < planes.size(); ++i) {
		if (dot(planes[i].normal, (planes[i].point - result)) > distance) {
			/* Result does not satisfy constraint of plane i. */
			std::vector<Plane> projPlanes;

			for (size_t j = 0; j < i; ++j) {
				Plane plane;

				const Vector crossProduct = cross(planes[j].normal, planes[i].normal);

				if ((crossProduct).lengthSquared() <= RVO_EPSILON) {
					/* Plane i and plane j are (almost) parallel. */
					if (dot(planes[i].normal, planes[j].normal) > 0.0f) {
						/* Plane i and plane j point in the same direction. */
						continue;
					}
					else {
						/* Plane i and plane j point in opposite direction. */
						plane.point = Point(0,0,0) + (0.5f * (planes[i].point + planes[j].point));
					}
				}
				else {
					/* Plane.point is point on line of intersection between plane i and plane j. */
					const Vector lineNormal = cross(crossProduct, planes[i].normal);
					plane.point = planes[i].point + (dot((planes[j].point - planes[i].point), planes[j].normal) / dot(lineNormal, planes[j].normal)) * lineNormal;
				}

				plane.normal = normalize(planes[j].normal - planes[i].normal);
				projPlanes.push_back(plane);
			}

			const Vector tempResult = result;

			if (linearProgram3(projPlanes, radius, planes[i].normal, true, result) < projPlanes.size()) {
				/* This should in principle not happen.  The result is by definition already in the feasible region of this linear program. If it fails, it is due to small floating point error, and the current result is kept. */
				result = tempResult;
			}

			distance = dot(planes[i].normal, (planes[i].point - result));
		}
	}
}
