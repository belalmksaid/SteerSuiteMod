//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#ifndef __PHASE_DECIMATION_AI_MODULE_H__
#define __PHASE_DECIMATION_AI_MODULE_H__


#include "SteerLib.h"
#include "shadowAI/GridAStar.h"
#include "shadowAI/ShadowAgent.h"
#include "trackReader/TrackReader.h"
#include "C5Reader/TreeInterface.h"
#include "../../../steergen/include/StateConfig.h"

// forward declaration
class GridEnvironment;
class PerformanceProfiler;

namespace ShadowAIGlobals {

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
	extern bool gMakeShadowRec;
	extern std::string gDatabaseRoot;
	extern std::string gTrackReader;
	extern std::string gShadowRecFilename;


	extern PhaseProfilers * gPhaseProfilers;
}

class ShadowAIModule : public SteerLib::ModuleInterface
{
public:
	std::string getDependencies() { return ""; }
	std::string getConflicts() { return ""; }
	std::string getData() { return ""; }
	LogData * getLogData() { return new LogData(); }
	void init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo );
	void finish(void);
	void initializeSimulation(void);
	void preprocessFrame(float, float, unsigned int);
	void postprocessSimulation(void);
	SteerLib::AgentInterface * createAgent(void);
	void destroyAgent( SteerLib::AgentInterface * agent ) { if (agent) delete agent;  agent = NULL; }
	void draw(void);
	~ShadowAIModule(void);
private:
	TrackReader* track_reader; 
	TreeInterface* classifier;
	StateConfig* featureSets;
	bool logToFile;
	std::string logFilename;
	static unsigned int agentID;
	std::vector<ShadowAgent*> agentLinks;
};


#endif
