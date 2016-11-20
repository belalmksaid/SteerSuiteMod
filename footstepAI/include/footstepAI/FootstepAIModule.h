//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#ifndef __PHASE_DECIMATION_AI_MODULE_H__
#define __PHASE_DECIMATION_AI_MODULE_H__


#include "SteerLib.h"
#include "footstepAI/GridAStar.h"
#include "footstepAI/FootstepAgent.h"

#include "Logger.h"

// forward declaration
class GridEnvironment;
class PerformanceProfiler;

namespace FootstepGlobals {

	struct PhaseProfilers {
		Util::PerformanceProfiler aiProfiler;
		Util::PerformanceProfiler drawProfiler;
		Util::PerformanceProfiler footstepPlanningPhaseProfiler;
		Util::PerformanceProfiler longTermPhaseProfiler;
		Util::PerformanceProfiler midTermPhaseProfiler;
		Util::PerformanceProfiler shortTermPhaseProfiler;
		Util::PerformanceProfiler perceptivePhaseProfiler;
		//Util::PerformanceProfiler predictivePhaseProfiler;
		//Util::PerformanceProfiler reactivePhaseProfiler;
		Util::PerformanceProfiler locomotionPhaseProfiler;
	};

	extern SteerLib::EngineInterface * gEngine;
	extern SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
	//extern FootRecWriter * gFootRecWriter;
	extern GridEnvironment * gEnvironmentForAStar;
/*	extern float gSpatialIncrement;
	extern float gDesiredFPS; // TODO: how to best conceptually cleanly get this value from clock?
	extern unsigned int gLongTermPlanningPhaseInterval;
	extern unsigned int gMidTermPlanningPhaseInterval;
	extern unsigned int gShortTermPlanningPhaseInterval;
	extern unsigned int gPredictivePhaseInterval;
	extern unsigned int gReactivePhaseInterval;
	extern unsigned int gPerceptivePhaseInterval;
	extern unsigned int gNumFramesPerSpaceTimePathNode;
	extern bool gUseDynamicDecimation;*/
	extern bool gShowStats;
	extern bool gShowAllStats;
	extern bool gMakeFootstepRec;
	extern std::string gFootstepRecFilename;

	extern bool gMakeFootstepSTPRec;
	extern std::string gFootstepSTPRecFilename;

	extern PhaseProfilers * gPhaseProfilers;



	extern float preferred_step_angle;

	// walking parameters
	extern float default_com_height;
	extern float default_mass;
	extern float default_min_step_length;
	extern float default_max_step_length;
	extern float default_min_step_time;
	extern float default_max_step_time;
	extern float default_max_speed;
	extern float default_base_radius;
	extern float default_time_cost_weight;
	extern float default_trajectory_cost_weght;

	extern float shoulder_comfort_zone; // for shoulder rays
	extern float shoulder_comfort_zone_2; // for shoulder disks



	// arbitrary constants
	extern float ped_reached_target_distance_threshold;
	 // this will influence the distance for the planned local goal
	extern int furthest_local_target_distance;
	extern int next_waypoint_distance;
	extern float ped_query_radius;
	extern float ped_torso_radius;
	extern int ped_num_steps_before_forced_plan;
	extern float ped_initial_step_variation;
	extern float ped_reached_footstep_goal_threshold;

	extern int max_nodes_to_expand;
}

class FootstepAIModule : public SteerLib::ModuleInterface
{
public:
	std::string getDependencies() { return ""; }
	std::string getConflicts() { return ""; }
	std::string getData() { return _data; }
	LogData * getLogData()
	{
		LogData * lD = new LogData();
		lD->setLogger(this->_footstepLogger);
		lD->setLogData(this->_logData);
		return lD;
	}
	void init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo );
	void finish();
	void initializeSimulation();
	void postprocessSimulation();
	SteerLib::AgentInterface * createAgent() { return new FootstepAgent; }
	void destroyAgent( SteerLib::AgentInterface * agent ) { if (agent) delete agent;  agent = NULL; }
	void draw();
	void cleanupSimulation();

private:
	bool logToFile;
	std::string logFilename;
	Logger * _footstepLogger;
	std::string _data;
	std::vector<LogObject *> _logData;
};


#endif
