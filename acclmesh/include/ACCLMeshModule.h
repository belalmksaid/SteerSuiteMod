//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//
/*
 * ACCLMeshModule.h
 *
 *  Created on: Mar 6, 2015
 *      Author: gberseth
 */

#ifndef ACCLMESH_ACCLMESHMODULE_H_
#define ACCLMESH_ACCLMESHMODULE_H_

#include "SteerLib.h"
#include "interfaces/PlanningDomainModuleInterface.h"

namespace ACCLMesh
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

namespace ACCLMesh  {

class ACCLMeshModule : public SteerLib::PlanningDomainModuleInterface {
public:
	ACCLMeshModule();
	virtual ~ACCLMeshModule();

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


	virtual void processKeyboardInput(int key, int action );
	virtual void processMouseMovementEvent(int deltaX, int deltaY );
	virtual void processMouseButtonEvent(int button, int action);

	SteerLib::PlanningDomainInterface * getPathPlanner() { return _pathPlanner; }
	void saveStaticGeometryToObj(std::string filename);
protected:
	SteerLib::EngineInterface * _engine;

	std::string _meshFileName;
	std::string _objFileName;
	SteerLib::SpatialDataBaseInterface * _spatialDatabase;
	SteerLib::PlanningDomainInterface * _pathPlanner;

	size_t _targetNode;
	bool _moving;
	Util::Vector _obs_displacement;
};

} /* namespace ACCLMesh */

#endif /* ACCLMESH_SRC_ACCLMESHMODULE_H_ */
