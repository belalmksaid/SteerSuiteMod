//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#ifndef __PHASE_DECIMATION_AI_MODULE_H__
#define __PHASE_DECIMATION_AI_MODULE_H__


#include "SteerLib.h"
#include "dataAI/GridAStar.h"
#include "dataAI/DataAgent.h"
#include "C5Reader/TreeInterface.h"
#include "../../../steergen/include/StateConfig.h"

// forward declaration
class GridEnvironment;
class PerformanceProfiler;

namespace DataAIGlobals {

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
		Util::PerformanceProfiler contextFeatureProfiler;
		Util::PerformanceProfiler specializedFeatureProfiler;
	};

	extern SteerLib::EngineInterface* gEngineInfo;
	extern SteerLib::GridDatabase2D* gSpatialDatabase;
	extern GridEnvironment* gEnvironmentForAStar;
	extern bool gShowStats;
	extern bool gShowAllStats;
	extern bool gMakeDataRec;
	extern std::string gDatabaseRoot;
	extern std::string gDataRecFilename;
	extern PhaseProfilers* gPhaseProfilers;
}

class DataAIModule : public SteerLib::ModuleInterface
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
	SteerLib::AgentInterface * createAgent(void);
	void destroyAgent( SteerLib::AgentInterface * agent ) { if (agent) delete agent;  agent = NULL; }
	void draw();
	~DataAIModule(void);
private:
	TreeInterface* classifier;
	StateConfig* featureSets;
	bool logToFile;
	std::string logFilename;
	bool constrainContext;
	unsigned int soleContext;
	bool enableTranscript;
	static unsigned int agentID;
	std::vector<vector<std::string>> transcripts;
};


#endif
