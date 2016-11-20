//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



/// @file HybridAIModule.cpp
/// @brief Implements the HybridAIModule plugin.


#include "SteerLib.h"
#include "SimulationPlugin.h"
#include "HybridAIModule.h"
#include "HybridAgent.h"


// globally accessible to the HybridAI plugin
SteerLib::EngineInterface * gEngine;
SteerLib::SpatialDataBaseInterface * gSpatialDatabase;



PLUGIN_API SteerLib::ModuleInterface * createModule()
{
	return new HybridAIModule;
}

PLUGIN_API void destroyModule( SteerLib::ModuleInterface*  module )
{
	delete module;
}


void HybridAIModule::init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo )
{
	gEngine = engineInfo;
	gSpatialDatabase = engineInfo->getSpatialDatabase();
	std::string recFilename = "";

	SteerLib::OptionDictionary::const_iterator optionIter;
	for (optionIter = options.begin(); optionIter != options.end(); ++optionIter)
	{
		if ((*optionIter).first == "recfile")
		{
			recFilename = (*optionIter).second;
		}
		else {
			// throw Util::GenericException("unrecognized option \"" + Util::toString((*optionIter).first) + "\" given to recFilePlayer module.");
		}
	}

	_aiModules.clear();


	_pprModule = new PPRAIModule();
	_rvo2dModule = new RVO2DAIModule();
	_footstepModule = new FootstepAIModule();
	// _ccModule = new CCAIModule();
	_recFilePlayerModule = new SteerLib::RecFilePlayerModule();

	_aiModules.push_back(_pprModule);
	_aiModules.push_back(_rvo2dModule);
	_aiModules.push_back(_footstepModule);
	// _aiModules.push_back(_ccModule);

	if ( recFilename != "")
	{
		_aiModules.push_back(_recFilePlayerModule);
	}

	for (int i = 0; i < _aiModules.size(); i++ )
	{
		_aiModules.at(i)->init(options, engineInfo);
	}
}

void HybridAIModule::initializeSimulation()
{
	for (int i = 0; i < _aiModules.size(); i++ )
	{
		_aiModules.at(i)->initializeSimulation();
	}
}

void HybridAIModule::cleanupSimulation()
{
	for (int i = 0; i < _aiModules.size(); i++ )
	{
		_aiModules.at(i)->cleanupSimulation();
	}
}

void HybridAIModule::preprocessSimulation()
{
	for (int i = 0; i < _aiModules.size(); i++ )
	{
		_aiModules.at(i)->preprocessSimulation();
	}
}

void HybridAIModule::preprocessFrame(float timeStamp, float dt, unsigned int frameNumber)
{
	for (int i = 0; i < _aiModules.size(); i++ )
	{
		_aiModules.at(i)->preprocessFrame(timeStamp, dt, frameNumber);
	}
}

void HybridAIModule::postprocessFrame(float timeStamp, float dt, unsigned int frameNumber)
{
	for (int i = 0; i < _aiModules.size(); i++ )
	{
		_aiModules.at(i)->postprocessFrame(timeStamp, dt, frameNumber);
	}
}



void HybridAIModule::finish()
{
	// nothing to do here
	// _pprModule->finish();
	// _rvo2dModule->finish();
	for (int i = 0; i < _aiModules.size(); i++ )
	{
		_aiModules.at(i)->finish();
	}
}

SteerLib::AgentInterface * HybridAIModule::createAgent()
{
	return new HybridAgent;
}

void HybridAIModule::destroyAgent( SteerLib::AgentInterface * agent )
{
	delete agent;
}
