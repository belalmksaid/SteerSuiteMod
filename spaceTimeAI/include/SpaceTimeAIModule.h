//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#ifndef __SPACE_TIME_AI_MODULE__
#define __SPACE_TIME_AI_MODULE__

/// @file SpaceTimeAIModule.h
/// @brief Declares the SpaceTimeAIModule plugin.


#include "SteerLib.h"



// globally accessible to the spaceTimeAI plugin
extern SteerLib::EngineInterface * gEngine;
extern SteerLib::GridDatabase2D * gSpatialDatabase;



/**
 * @brief An example plugin for the SimulationEngine that provides very basic AI agents.
 *
 * This class is an example of a plug-in module (as opposed to a built-in module).
 * It compiles as part of a dynamic library which is loaded by a SimulationEngine at run-time.
 *
 * The spaceTimeAI plugin consists of three parts:
 *  - This class inherits from SteerLib::ModuleInterface, and implements only the desired functionality.  In this case
 *    the desired functionality is to be able to create/destroy SpaceTimeAgent agents.
 *  - The two global functions createModule() and destroyModule() are implemented so that the engine can load the
 *    dynamic library and get an instance of our SpaceTimeAIModule.
 *  - The SpaceTimeAgent class inherits from SteerLib::AgentInterface, which is the agent steering AI used by the engine.
 *    this agent serves as an example of how to create your own steering AI using SteerLib features.
 *
 */
class SpaceTimeAIModule : public SteerLib::ModuleInterface
{
public:
	std::string getDependencies() { return ""; }
	std::string getConflicts() { return ""; }
	std::string getData() { return ""; }
	LogData * getLogData() { return new LogData(); }
	void init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo );
	void finish();
	SteerLib::AgentInterface * createAgent();
	void destroyAgent( SteerLib::AgentInterface * agent );

protected:
	int _maxNumNodesToExpand;
	bool _useHermiteInterpolation;
};

#endif
