//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


/// @file RVO3DAIModule.cpp
/// @brief Implements the RVO3DAIModule plugin.


#include "SteerLib.h"
#include "SimulationPlugin.h"
#include "RVO3DAIModule.h"
#include "RVO3DAgent.h"


// globally accessible to the simpleAI plugin
SteerLib::EngineInterface * gEngine;
SteerLib::SpatialDataBaseInterface * gSpatialDatabase;



PLUGIN_API SteerLib::ModuleInterface * createModule()
{
	return new RVO3DAIModule;
}

PLUGIN_API void destroyModule( SteerLib::ModuleInterface*  module )
{
	delete module;
}


void RVO3DAIModule::init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo )
{
	gEngine = engineInfo;
	gSpatialDatabase = engineInfo->getSpatialDatabase();
	kdTree_ = new KdTree;
	kdTree_->setSimulator(engineInfo);
}

void RVO3DAIModule::preprocessSimulation()
{
	// kdTree_->buildObstacleTree();
}

void RVO3DAIModule::finish()
{
	// nothing to do here
}

void RVO3DAIModule::preprocessFrame(float timeStamp, float dt, unsigned int frameNumber)
{

	if ( frameNumber == 1)
	{
		// Adding in this extra one because it seemed sometimes agents would forget about obstacles.
		// kdTree_->buildObstacleTree();
	}
	if ( !agents_.empty() )
	{
		kdTree_->buildAgentTree();
	}
}

void RVO3DAIModule::postprocessFrame(float timeStamp, float dt, unsigned int frameNumber)
{
	// do nothing for now
	int i = 0;
	i = i + i;
}
SteerLib::AgentInterface * RVO3DAIModule::createAgent()
{
	RVO3DAgent * agent = new RVO3DAgent;
	agent->rvoModule = this;
	agent->id_ = agents_.size();
	agents_.push_back(agent);
	return agent;

}

void RVO3DAIModule::destroyAgent( SteerLib::AgentInterface * agent )
{
	delete agent;
}


void RVO3DAIModule::cleanupSimulation()
{
	agents_.clear();
	// kdTree_->deleteObstacleTree(kdTree_->obstacleTree_);
	kdTree_->agents_.clear();
}
