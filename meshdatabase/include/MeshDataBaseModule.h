//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//
/*
 * NavmeshModule.h
 *
 *  Created on: 2014-11-27
 *      Author: gberseth
 */

#ifndef MESHDATABASEMODULE_H_
#define MESHDATABASEMODULE_H_


#include "interfaces/SpatialDataBaseModuleInterface.h"
#include "interfaces/SpatialDataBaseInterface.h"
#include "MeshDataBase.h"
#include "SteerLib.h"
#include "SimulationPlugin.h"

// #include "Logger.h"

namespace MeshDataBaseGlobals
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


	extern SteerLib::EngineInterface * gEngine;
	extern SteerLib::SpatialDataBaseInterface * gSpatialDatabase;

	extern PhaseProfilers * gPhaseProfilers;
}

class MeshDataBaseModule : public SteerLib::SpatialDataBaseModuleInterface
{
public:
	std::string getDependencies() { return ""; }

	std::string getConflicts() { return ""; }
	std::string getData() { return ""; }
	LogData * getLogData() { return new LogData(); }

	void init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo );


	void finish();

	void initializeSimulation();
	void preprocessSimulation();

	void preprocessFrame(float timeStamp, float dt, unsigned int frameNumber);

	void postprocessFrame(float timeStamp, float dt, unsigned int frameNumber);


	void postprocessSimulation() ;

	void draw();

	SteerLib::SpatialDataBaseInterface * getSpatialDataBase() { return _spatialDatabase; }
	// PlanningDomainInterface * getPathPlanner() { return _pathPlanner; }
	void saveStaticGeometryToObj(std::string filename);
protected:
	SteerLib::EngineInterface * _engine;

	std::string _meshFileName;

	MeshDataBase * _spatialDatabase;
};


#endif /* NAVMESHMODULE_H_ */
