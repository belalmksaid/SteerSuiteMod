//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#ifndef __SHADOW_REC_IO_H__
#define __SHADOW_REC_IO_H__

#include "SteerLib.h"

//
// foot rec file format:
//
// 1. main header
// 2. array of obstacles
// 2. array of agent headers
// 3. arrays of step sequences for each agent
//


enum FootStateEnum {
	FOOTSTEP_STATE_NORMAL,
	FOOTSTEP_STATE_STOPPING,
	FOOTSTEP_STATE_STARTING,
	FOOTSTEP_STATE_STATIONARY,
};

// valid transitions:
//
// NORMAL --> NORMAL      (whichFoot alternates)
// NORMAL --> STOPPING      (whichFoot alternates)
// STOPPING --> STARTING      (whichFoot DOES NOT alternate)
// STOPPING --> STATIONARY      (whichFoot does not matter when stationary (except for knowing which foot location is actually described in the data structure))
// STATIONARY --> STATIONARY       (whichFoot does not matter when stationary)
// STATIONARY --> STARTING      (whichFoot can be left or right depending on the chosen trajectory direction)
// STARTING --> NORMAL          (whichFoot alternates)
// STARTING --> STOPPING        (whichFoot alternates)
//
//


struct DynamicState {
	float x, z, dx, dz;
};

struct Footstep {
	// time at which this footstep begins.
	float startTime;

	// time at which the footstep ends.
	float endTime;

	// 2D world-space origin of the parabola.  (computed based on control parameters)
	//  the foot is slightly displaced from this based on  "baseradius" agent parameter.
	float parabolaX, parabolaZ;

	// world-space phi angle of parabola orientation.
	float parabolaOrientationPhi;

	// world-space phi angle of footstep orientation.
	// the valid set of foot orientations is tracked as an interval between the inner-most and outer-most angles.
	// angles are (as usual) in radians, and in world-space.
	float innerFootOrientationPhi;
	float outerFootOrientationPhi;

	// flag indicating which foot (left/right) is the pivot.
	bool whichFoot;

	// describes how much this footstep choice cost, including initial change in velocity and 
	float energyCost;

	// world-space dynamic state of the character's center of mass
	// at the END of the step.  (dissipating energy occurs at beginning of next step).
	DynamicState outputCOMState;

	// parameters that don't need to be stored, but helpful to analytically compute the quadratic to visualize the trajectory
	float simulationA, simulationDx, simulationIx, simulationIz, simulationJx, simulationJz;

	// 2D world-space location of the foot.
	float footX, footZ;

	// used to work properly with the search library being used
	bool isAGoalState;

	FootStateEnum state;

	// Here down, added by Cory to the format for data-driven model building (dataAI)
	bool phiIsIdeal;

	float desiredSpeed;

	float targetX;
	float targetZ;
	bool isPlanned;

	// Here down, added for the anomaly tracking (shadowAI)
	bool isAnomaly;
	float cumulativeScore;
	float instantScore;

	int contextUsed;

	DynamicState trueCOMState;
};


struct ShadowRecHeader {
	SteerLib::REC_FORMAT format;
	unsigned int numAgents;
	unsigned int numObstacles;
	unsigned int agentTableOffset;
	unsigned int obstacleDataOffset;
};


struct AgentFootHeader {
	unsigned int numFootsteps;
	unsigned int footstepTableOffset;
};

struct ObstacleData {
	float xmin;
	float xmax;
	float zmin;
	float zmax;
};

struct StepData {
	Footstep step;
	unsigned int lookAtObject;
	unsigned int padding[8];  // if there's a way to salvage backwards compatibility when modifying these data structures
};

typedef std::vector<StepData>  StepDataSequence;


class ShadowRecWriter {
public:
	ShadowRecWriter(size_t numAgents, size_t numObstacles);
	~ShadowRecWriter();
	void ShadowRecWriter::writeToFile(const std::string & filename);
	void addFootstep(unsigned int agentIndex, const StepData & s);
	void addFootstep(unsigned int agentIndex, const Footstep & footstep, unsigned int lookatID);
	void addObstacleInfo(unsigned int obstacleIndex, float xmin, float xmax, float zmin, float zmax);

protected:
	unsigned int _numAgents;
	unsigned int _numObstacles;
	ObstacleData* _obstacles;
	StepDataSequence * _agentSequences;

};


class ShadowRecReader {
public:
	explicit ShadowRecReader(const std::string & filename);
	~ShadowRecReader();
	unsigned int getNumAgents();
	unsigned int getNumObstacles();
	unsigned int getNumStepsForAgent(unsigned int agentIndex);
	ObstacleData& getObstacle(unsigned int obstacleIndex);
	StepData& getStepData(unsigned int agentIndex, unsigned int stepNumber);

private:
	Util::MemoryMapper _fileMap;
	ShadowRecHeader *    _header;
	AgentFootHeader *  _agents;
	ObstacleData* _obstacles;
	StepData ** _steps;
};

#endif