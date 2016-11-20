//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#ifndef __EGOCENTRIC_CONSTANTS_H__
#define __EGOCENTRIC_CONSTANTS_H__

//#define PEDESTRIAN_RADIUS 0.5f

// queryGridDabatase
#define PEDESTRIAN_STATIC_OBJECT_COMFORT_ZONE 0.1f
#define PEDESTRIAN_DYNAMIC_OBJECT_COMFORT_ZONE 0.3f

// determineNodeIdToMoveToUsingActivation ()
#define MINIMUM_ACTIVATION_THRESHOLD 0.1f

// spreadActivation ()
#define RIGHT_ACTIVATION_DECAY 0.95
#define LEFT_ACTIVATION_DECAY 0.93 // to remove symmetry and avoid confusion in choosing direction.
#define FORWARD_ACTIVATION_DECAY 0.90
#define BACKWARD_ACTIVATION_DECAY 0.98

enum MapDisplayChoice
{ 
	MapDisplayChoice_NoDisplay,
	MapDisplayChoice_Traversability,
	MapDisplayChoice_Activation,
	MapDisplayChoice_NodeNumber,
	MapDisplayChoice_ActivationPed
};

// EgocentricAgent::reachedCurrentGoal()
#define EGO_PED_REACHED_TARGET_DISTANCE_THRESHOLD 2.0f // originally 0.5

// EgocentricAgent::runLongTermPlanningPhase()
#define WAYPOINT_DISTANCE 10

// for mid and shortTermPlanning
#define FURTHEST_LOCAL_TARGET_DISTANCE 20
#define EGO_NEXT_WAYPOINT_DISTANCE 60 // was 202
#define PED_MAX_NUM_WAYPOINTS 20


#define TIME_THRESHOLD 0.2f

// for steering 
// these are measured in meters/second
#define PED_MAX_SPEED                2.6f
#define PED_TYPICAL_SPEED            1.3f

#define PED_MAX_FORCE                14.0f



// egocentric2/EgocentricMap1.cpp
#define PEDESTRIAN_RADIUS 0.5f
#define MAX_SACOUNT 40
#define ACTIVATION_DECAY 0.95
//#define SPACE_TIME_PLANNING

// we use this turning rate so that the AI can have use -1.0 to 1.0 for the "amount of turning".
// also note this assumes that side() and forward() are both normalized, which is probably a good idea anyway.
#define PED_MAX_TURNING_RATE 0.1f
// factor that decides how strong the side-to-side scooting forces will be when an agent realizes it needs to scoot to the side.
#define PED_SCOOT_RATE 0.4f

#endif 
