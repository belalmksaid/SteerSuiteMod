//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#ifndef __EGOCENTRIC_AI_MODULE__
#define __EGOCENTRIC_AI_MODULE__

/// @file EgocentricAIModule.h
/// @brief Declares the EgocentricAIModule plugin.


#include "SteerLib.h"
#include "Logger.h"

// forward declaration
class GridEnvironment;


// globally accessible to the egocentricAI plugin

namespace EgocentricGlobals
{

	struct PhaseProfilers {
		Util::PerformanceProfiler aiProfiler;
		Util::PerformanceProfiler drawProfiler;
		Util::PerformanceProfiler longTermPhaseProfiler;
		Util::PerformanceProfiler midTermPhaseProfiler;
		Util::PerformanceProfiler shortTermPhaseProfiler;
		Util::PerformanceProfiler perceptivePhaseProfiler;
		Util::PerformanceProfiler predictivePhaseProfiler;
		Util::PerformanceProfiler reactivePhaseProfiler;
		Util::PerformanceProfiler steeringPhaseProfiler;
	};


	extern SteerLib::EngineInterface * gEngineInfo;
	extern SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
	extern SteerLib::EngineInterface * gEngine;
	extern GridEnvironment * gEnvironmentForAStar;

	extern bool gUseDynamicPhaseScheduling;
	extern bool gShowStats;
	extern bool gShowAllStats;


	extern PhaseProfilers * gPhaseProfilers;

	extern float pedestrian_static_object_comfort_zone ;
	extern float pedestrian_dynamic_object_comfort_zone ;
	extern float minimum_activation_threshold ;
	extern float right_activation_decay ;
	extern float left_activation_decay ;
	extern float forward_activation_decay ;
	extern float backward_activation_decay ;
	extern float ego_ped_reached_target_distance_threshold ;
	extern int waypoint_distance ;
	extern int furthest_local_target_distance ;
	extern int ego_next_waypoint_distance ;
	extern int ped_max_num_waypoints ;
	extern float time_threshold ;
	extern float ped_max_speed                ;
	extern float ped_typical_speed            ;
	extern float ped_max_force                ;
	extern float pedestrian_radius ;
	extern int max_sacount ;
	extern float activation_decay ;
	extern float ped_max_turning_rate;
	extern float ped_scoot_rate;
}


/**
 * @brief A plugin for SteerSim that provides egocentric AI agents.
 *
 * This class is an example of a plug-in module (as opposed to a built-in module).
 * It compiles as part of a dynamic library which is loaded by SteerSim at run-time.
 *
 * The egocentricAI plugin consists of three parts:
 *  - This class inherits from SteerLib::ModuleInterface, and implements only the desired functionality.  In this case
 *    the desired functionality is to be able to create/destroy EgocentricAgent agents.
 *  - The two global functions createModule() and destroyModule() are implemented so that the engine can load the
 *    dynamic library and get an instance of our EgocentricAIModule.
 *  - The EgocentricAgent class inherits from SteerLib::AgentInterface, which is the agent steering AI used by the engine.
 *
 */
class EgocentricAIModule : public SteerLib::ModuleInterface
{
public:
	std::string getDependencies() { return ""; }
	std::string getConflicts() { return ""; }
	std::string getData() { return ""; }
	LogData * getLogData()
	{
		LogData * lD = new LogData();
		lD->setLogger(this->_logger);
		lD->setLogData(this->_logData);
		return lD;
	}
	void init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo );
	void finish();
	SteerLib::AgentInterface * createAgent();
	void destroyAgent( SteerLib::AgentInterface * agent );
	void initializeSimulation();
	void cleanupSimulation();

private:
	bool logStats;
	std::string logFilename;
	Logger * _logger;
	std::string _data;
	std::vector<LogObject *> _logData;
};

#endif
