//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


/// @file SpaceTimeAIModule.cpp
/// @brief Implements the SpaceTimeAIModule plugin.


#include "SteerLib.h"
#include "SimulationPlugin.h"
#include "SpaceTimeAIModule.h"
#include "SpaceTimeAgent.h"

//steersim.exe -config myConfig.xml -testcase ../../../testcases/bottleneck-squeeze.xml -ai SpaceTimeAI
//steersim.exe -config myConfig.xml -testcase ../../../testcases/squeeze.xml -ai SpaceTimeAI
//steersim.exe -config myConfig.xml -testcase ../../../testcases/simple-wall.xml -ai SpaceTimeAI

// globally accessible to the spaceTimeAI plugin
SteerLib::EngineInterface * gEngine;
SteerLib::GridDatabase2D * gSpatialDatabase;



PLUGIN_API SteerLib::ModuleInterface * createModule()
{
	return new SpaceTimeAIModule;
}

PLUGIN_API void destroyModule( SteerLib::ModuleInterface*  module )
{
	delete module;
}


void SpaceTimeAIModule::init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo )
{
	gEngine = engineInfo;
	gSpatialDatabase = engineInfo->getSpatialDatabase();
	
	_maxNumNodesToExpand = INT_MAX;
	_useHermiteInterpolation = false;

	SteerLib::OptionDictionary::const_iterator optionIter;
	for (optionIter = options.begin(); optionIter != options.end(); ++optionIter) {
		
		if ((*optionIter).first == "maxNumNodesToExpand") {
			_maxNumNodesToExpand = atoi ( (*optionIter).second.c_str() );
		} else if ((*optionIter).first == "useHermiteInterpolation") {
			_useHermiteInterpolation = true;
		}
	}
}

void SpaceTimeAIModule::finish()
{
	// nothing to do here
}

SteerLib::AgentInterface * SpaceTimeAIModule::createAgent()
{
	SpaceTimeAgent* newAgent = new SpaceTimeAgent();
	newAgent->init(_maxNumNodesToExpand, _useHermiteInterpolation);
	return newAgent; 
}

void SpaceTimeAIModule::destroyAgent( SteerLib::AgentInterface * agent )
{
	delete agent;
}
