//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#ifndef __SPACE_TIME_PLANNER_H__
#define __SPACE_TIME_PLANNER_H__

#include "SteerLib.h"
#include "planning/BestFirstSearchPlanner.h"
#include "SpaceTimeAgent.h"

using namespace Util;

class SpaceTimePlanningDomain
{
public:
	SpaceTimePlanningDomain(SteerLib::GridDatabase2D* spatialDatabase, SpaceTimeAgent* agent);
	bool SpaceTimePlanningDomain::canBeTraversed(std::set<SteerLib::SpatialDatabaseItemPtr> neighbors, const SpaceTimePoint a, const SpaceTimePoint b, SpaceTimeAgent* agent);
	bool isAGoalState( const SpaceTimePoint & state, const SpaceTimePoint & idealGoalState);
	float estimateTotalCost( const SpaceTimePoint & currentState, const SpaceTimePoint & idealGoalState, float currentg);
	void generateTransitions( const SpaceTimePoint & currentState, const SpaceTimePoint & previousState, const SpaceTimePoint & idealGoalState, std::vector<SteerLib::DefaultAction<SpaceTimePoint> > & transitions );	//PUT back SteerLib::DefaultAction<Path>

private:
	SteerLib::GridDatabase2D* gSpatialDatabase;
	SpaceTimeAgent* thisAgent;
	double baseCost;
};

#endif