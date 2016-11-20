//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#ifndef __HIDAC_AI_MODULE__
#define __HIDAC_AI_MODULE__

/// @file SimpleAIModule.h
/// @brief Declares the SimpleAIModule plugin.


#include "SteerLib.h"
#include <vector>
#include "HIDAC_Parameters.h"
#include "Logger.h"


// globally accessible to the simpleAI plugin
extern SteerLib::EngineInterface * gEngine;
// extern SteerLib::GridDatabase2D * gSpatialDatabase;



/**
 * @brief An example plugin for the SimulationEngine that provides very basic AI agents.
 *
 * This class is an example of a plug-in module (as opposed to a built-in module).
 * It compiles as part of a dynamic library which is loaded by a SimulationEngine at run-time.
 *
 * The simpleAI plugin consists of three parts:
 *  - This class inherits from SteerLib::ModuleInterface, and implements only the desired functionality.  In this case
 *    the desired functionality is to be able to create/destroy SimpleAgent agents.
 *  - The two global functions createModule() and destroyModule() are implemented so that the engine can load the
 *    dynamic library and get an instance of our SimpleAIModule.
 *  - The SimpleAgent class inherits from SteerLib::AgentInterface, which is the agent steering AI used by the engine.
 *    this agent serves as an example of how to create your own steering AI using SteerLib features.
 *
 */



class HIDACAIModule : public SteerLib::ModuleInterface
{
public:
	
	void cleanupSimulation();
	void preprocessSimulation();
	void initializeSimulation();
	//std::string getDependencies() { return "testCasePlayer"; }
	std::string getDependencies() { return ""; }
	
	std::string getConflicts() { return ""; }
	std::string getData() { return ""; }
	LogData * getLogData() { return new LogData(); }
	void init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo );
	void finish();
	SteerLib::AgentInterface * createAgent();
	void destroyAgent( SteerLib::AgentInterface * agent );

	void preprocessFrame(float timeStamp, float dt, unsigned int frameNumber);
	void postprocessFrame(float timeStamp, float dt, unsigned int frameNumber);
	std::vector<SteerLib::AgentInterface * > agents_;

protected:
	std::string logFilename; // = "pprAI.log";
	bool logStats; // = false;
	Logger * _rvoLogger;
};

#endif
