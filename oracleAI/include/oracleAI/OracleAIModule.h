//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#ifndef __PHASE_DECIMATION_AI_MODULE_H__
#define __PHASE_DECIMATION_AI_MODULE_H__


#include "SteerLib.h"
#include "oracleAI/GridAStar.h"
#include "oracleAI/OracleAgent.h"

#include "Logger.h"

// forward declaration
class GridEnvironment;
class PerformanceProfiler;

namespace OracleGlobals {

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

	extern SteerLib::EngineInterface * gEngineInfo;
	extern SteerLib::GridDatabase2D * gSpatialDatabase;
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


	extern PhaseProfilers * gPhaseProfilers;
}

class OracleAIModule : public SteerLib::ModuleInterface
{
public:
	std::string getDependencies() { return ""; }
	std::string getConflicts() { return ""; }
	std::string getData() { return ""; }
	LogData * getLogData() { return new LogData(); }
	void init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo );
	void finish();
	void initializeSimulation();
	void postprocessSimulation();
	SteerLib::AgentInterface * createAgent() { return new OracleAgent; }
	void destroyAgent( SteerLib::AgentInterface * agent ) { if (agent) delete agent;  agent = NULL; }
	void draw();
	void cleanupSimulation();

private:
	bool logToFile;
	std::string logFilename;
	Logger * _oracleLogger;
};


#endif
