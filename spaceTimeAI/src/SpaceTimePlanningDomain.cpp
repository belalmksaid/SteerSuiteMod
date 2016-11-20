//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#include "SpaceTimePlanningDomain.h"

SpaceTimePlanningDomain::SpaceTimePlanningDomain(SteerLib::GridDatabase2D* spatialDatabase, SpaceTimeAgent* agent)
{
	baseCost = 0.1;
	gSpatialDatabase = spatialDatabase;
	thisAgent = agent;
}

bool SpaceTimePlanningDomain::canBeTraversed(std::set<SteerLib::SpatialDatabaseItemPtr> neighbors, const SpaceTimePoint a, const SpaceTimePoint b, SpaceTimeAgent* agent)
{
	double timeSteps = 20;
	double time;
	double personalSpace = 0;//0.25;
	Point agentPosition;
	Point* neighborAgentPosition;

	for (int i = 1; i <= timeSteps; i++) {
		time = (b.time - a.time)*(i/timeSteps) + a.time;
		agentPosition = a.point + (b.point - a.point)*(i/timeSteps);
		for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = neighbors.begin(); neighbor != neighbors.end(); ++neighbor) {
			if ((*neighbor)->isAgent()) {
				SpaceTimeAgent* neighborAgent = dynamic_cast<SpaceTimeAgent*>(*neighbor);
				if(neighborAgent != NULL && neighborAgent != agent) {
					neighborAgentPosition = neighborAgent->GetLocationAtTime(time);
					if(neighborAgentPosition != NULL) {
						double distance = (agentPosition - *neighborAgentPosition).length();
						if(distance < agent->radius() + neighborAgent->radius() + personalSpace) {
							return false;
						}
					}
				}
			} else if ((*neighbor)->overlaps(agentPosition, agent->radius())) {
				return false;
			}
		}
	}

	return true;
}


bool SpaceTimePlanningDomain::isAGoalState( const SpaceTimePoint & state, const SpaceTimePoint & idealGoalState)
{
	Point state2D = Point(state.point.x, 0, state.point.z);
	Point idealGoalState2D = Point(idealGoalState.point.x, 0, idealGoalState.point.z);
	if((state2D - idealGoalState2D).length() <= thisAgent->radius()) {
		return true;
	} else {
		return false;
	}
}

float SpaceTimePlanningDomain::estimateTotalCost( const SpaceTimePoint & currentState, const SpaceTimePoint & idealGoalState, float currentg)
{
	Point currentState2D = Point(currentState.point.x, 0, currentState.point.z);
	Point idealGoalState2D = Point(idealGoalState.point.x, 0, idealGoalState.point.z);
	double h = (currentState2D - idealGoalState2D).length()*(1 + baseCost);

	return currentg + h;
}

void SpaceTimePlanningDomain::generateTransitions( const SpaceTimePoint & currentState, const SpaceTimePoint & previousState, const SpaceTimePoint & idealGoalState, std::vector<SteerLib::DefaultAction<SpaceTimePoint> > & transitions )	//PUT back SteerLib::DefaultAction<Path> >
{
	if (currentState.time > 1000) {	// if it's taken this long so far, give up
		printf("PLAN REQUIRES TOO MUCH TIME................\n");
		return;
	}
	transitions.clear();
	std::set<SteerLib::SpatialDatabaseItemPtr> neighbors;
	neighbors.clear();
	double distance = 30;
	float xmin = currentState.point.x - distance;
	float xmax = currentState.point.x + distance;
	float zmin = currentState.point.z - distance;
	float zmax = currentState.point.z + distance;
	gSpatialDatabase->getItemsInRange(neighbors, xmin, xmax, zmin, zmax, NULL);
	std::set <SteerLib::SpatialDatabaseItemPtr>::iterator it;

	
	/*
	std::vector<double> speeds;
	speeds.push_back(0);
	speeds.push_back(1);
	//speeds.push_back(2);

	int numDirections = 8;

	Vector vectorToGoal = idealGoalState.point - currentState.point;
	for(int i = 0; i < numDirections; i++) {
		double theta = M_2_PI/numDirections*i;
		for(int j = 0; j < speeds.size(); j++) {
			if (speeds[j] > 0 || i == 0) {	// if the speed is zero we only want to do this for one direction
				double distance = speeds[j];
				SteerLib::DefaultAction<SpaceTimePoint> action;
				action.cost = distance + max(baseCost, distance*baseCost);	// add a small base cost for rest case
				Vector transitionVector(distance, 0, 0);

				action.state.point.x = currentState.point.x + cos(theta)*transitionVector.x - sin(theta)*transitionVector.z;
				action.state.point.z = currentState.point.z + sin(theta)*transitionVector.x + cos(theta)*transitionVector.z;
				action.state.point.y = 0;

				action.state.time = currentState.time + 1;
				if (canBeTraversed(neighbors, currentState, action.state, thisAgent)) {
					transitions.push_back(action);
				}
			}
		}
	}
	*/
	//for (int x = currentState.point.x - 0.5; x <= currentState.point.x + 1.5; x++) {
	//	for (int z = currentState.point.z - 0.5; z <= currentState.point.z + 1.5; z++) {
	double inc = 1;
	for (double x = currentState.point.x - inc; x <= currentState.point.x + inc * 1.01; x += inc/2) {
		for (double z = currentState.point.z - inc; z <= currentState.point.z + inc * 1.01; z += inc/2) {
			SteerLib::DefaultAction<SpaceTimePoint> action;
			action.state.point = Util::Point(x, 0, z);
			double distance = (currentState.point - action.state.point).length();
			action.cost =  distance + max(baseCost, distance*baseCost);
			action.state.time = currentState.time + 1;
			
			if (canBeTraversed(neighbors, currentState, action.state, thisAgent)) {
				transitions.push_back(action);
			}
		}
	}
}