//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//
/*
 * ACCLMeshModule.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: gberseth
 */

#include "ACCLMeshModule.h"
#include "Mesh.h"
#include "ACCLMeshDomain.h"
#include "SimulationPlugin.h"
#include "MeshUtils.h"
#include "ACCLMeshEnvironment.h"

#include "glfw/include/GL/glfw.h"

/*
 *
 * Example command:
 * ../build/bin/steersim -module acclmesh,load_obj=../acclmesh/data/staircase2.obj -testcase ../acclmesh/data/blank.xml -config ../acclmesh/data/ACCLMesh-config.xml -numFrames 1000
 */

namespace ACCLMesh
{
	SteerLib::EngineInterface * gEngine;
	SteerLib::SpatialDataBaseInterface * gSpatialDatabase;

	PhaseProfilers * gPhaseProfilers;
}

namespace ACCLMesh {

	ACCLMeshModule::ACCLMeshModule() {
		// TODO Auto-generated constructor stub
		_moving = false;
		_targetNode = 0;
		_obs_displacement = Util::Vector(0.0,0.0,0.0);
	}

	ACCLMeshModule::~ACCLMeshModule() {
		// TODO Auto-generated destructor stub
	}


	void ACCLMeshModule::init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo )
	{

		_engine = engineInfo;
		// iterate over all the options
		SteerLib::OptionDictionary::const_iterator optionIter;
		for (optionIter = options.begin(); optionIter != options.end(); ++optionIter) {
			std::cout << "option: " << (*optionIter).first << std::endl;
			if ((*optionIter).first == "technique") {
				// _techniqueName = (*optionIter).second;
			}
			else if ((*optionIter).first == "saveGeometry") {
				_meshFileName = (*optionIter).second;
			}
			else if ((*optionIter).first == "load_obj") {
				_objFileName = (*optionIter).second;
				std::cout << "Loading obj file: " << _objFileName << std::endl;
			}
			else if ((*optionIter).first == "targetNode") {
				_targetNode = atoi((*optionIter).second.c_str());
				// std::cout << "Loading obj file: " << _objFileName << std::endl;
			}
			else {
				throw Util::GenericException("unrecognized option \"" + Util::toString((*optionIter).first) + "\" given to navmesh module.");
			}
		}

		// std::cout << "Number of obstacles in engine: " << _engine->getObstacles().size() << std::endl;
		// gEngine = _engine;
		ACCLMesh * mesh = loadACCLMeshFromFile(_objFileName);
		std::cout << "Created mesh: " << mesh << " has vert count: " << mesh->get_vert_size() << std::endl;

		ACCLMeshDomain * acclmesh = new ACCLMeshDomain(engineInfo, mesh);
		acclmesh->_targetNode=_targetNode;
		this->_pathPlanner = acclmesh;


		gSpatialDatabase = engineInfo->getSpatialDatabase();
		// _sample = createSolo();
		// std::cout << "" << _engine->getStaticGeometry().first.at(0);
		// std::cout << "Got some geometry" << engineInfo << std::endl;

		// this->_spatialDatabase->init(options, _engine);
		// _benchmarkTechnique->setEngineInterface(_engine);
	}


	void ACCLMeshModule::initializeSimulation()
	{
		std::cout << "initialize Number of obstacles in engine: " << _engine->getObstacles().size() << std::endl;
	}

	void ACCLMeshModule::preprocessSimulation()
	{
		if (this->_meshFileName != "")
		{
			this->saveStaticGeometryToObj(this->_meshFileName);
		}
		// This needs to be here because obstacles are not put in _engine until after init();

		this->getPathPlanner()->refresh();

		// Mesh * mesh = new Mesh();
		// std::pair<std::vector<Util::Point>,std::vector<size_t> > mesh_stuff = dynamic_cast<ACCLMeshDomain*>( this->getPathPlanner())->getNavMeshGeometry();

		// mesh->init(mesh_stuff.first,mesh_stuff.second);


	}

	void ACCLMeshModule::preprocessFrame(float timeStamp, float dt, unsigned int frameNumber)
	{

	}

	void ACCLMeshModule::postprocessFrame(float timeStamp, float dt, unsigned int frameNumber) {
		// std::cout << "found the business? " << _engine->getAgents().size() << std::endl;
		// _benchmarkTechnique->update(_metricsCollectorModule->getSimulationMetrics(), timeStamp, dt);
		// _benchmarkTechnique->update( _engine, timeStamp, dt);
	}


	void ACCLMeshModule::postprocessSimulation() {
	}


	void ACCLMeshModule::draw()
	{
	#ifdef ENABLE_GUI
		// _spatialDatabase->draw();
		this->_pathPlanner->draw();
		// std::cout << "drawing navmesh Module" << std::endl;

	#endif

	}

	void ACCLMeshModule::finish()
	{


	}

	void ACCLMeshModule::saveStaticGeometryToObj(std::string fileName)
	{
		std::pair<std::vector<Util::Point>,std::vector<size_t> > mesh_stuff = _engine->getStaticGeometry();
		std::fstream fileStream;
		fileStream.open(fileName.c_str(),std::ios::out);

		fileStream << "# SteerSuite Geometry" << std::endl;
		fileStream << "o Scenario Geometry" << std::endl;

		fileStream << std::endl;


		for (size_t point=0; point < mesh_stuff.first.size(); point++)
		{
			Util::Point p = mesh_stuff.first.at(point);
			fileStream << "v " <<p.x << " " << p.y << " " << p.z <<std::endl;
		}

		fileStream << std::endl;

		for (size_t face=0; face < mesh_stuff.second.size(); face+=3)
		{ // wavefront vert indicies start at 1
			fileStream << "f " <<mesh_stuff.second.at(face)+1 << " " <<
					mesh_stuff.second.at(face+1)+1 << " " <<
					mesh_stuff.second.at(face+2)+1 << std::endl;
		}

		fileStream.close();
	}

	void ACCLMeshModule::processKeyboardInput(int key, int action )
	{
		std::cout << "ACCLMesh:: keyboard Event" << std::endl;
		if ((key == (int)'B'))
		{
			if (action==GLFW_PRESS)
			{
				_moving = true;
				std::cout << "ACCLMesh:: moving object" << std::endl;
			}
			else if ((action==GLFW_RELEASE))
			{
				_moving = false;
				std::cout << "ACCLMesh:: STOP** moving object" << std::endl;
			}
		}
	}

	void ACCLMeshModule::processMouseMovementEvent(int deltaX, int deltaY )
	{
		float move_scale=0.03;
		if (_moving)
		{
			std::cout << "ACCLMesh:: Mouse delta is: " << "(" << deltaX << ", " << deltaY << ")" << std::endl;
			ACCLMeshDomain* acclmesh = dynamic_cast<ACCLMeshDomain* >(this->_pathPlanner);
			if ( acclmesh != NULL )
			{
				Util::Vector delta = Util::Vector(deltaX, 0.0, deltaY)*move_scale;
				_obs_displacement = _obs_displacement + delta;
				// Util::Vector delta = Util::Vector(0.0, 1.0, 0.0)*move_scale;
				acclmesh->moveObstacle(_obs_displacement);
				this->_pathPlanner->refresh();
				for (size_t a=0; a < this->_engine->getAgents().size(); a++)
				{ // reset agents paths
					SteerLib::AgentInitialConditions agentInterface = SteerLib::AgentInterface::getAgentConditions(this->_engine->getAgents().at(a));
					this->_engine->getAgents().at(a)->reset(
							agentInterface,
							this->_engine);
				}
			}
			else
			{
				std::cout << "Error casting ACCLMeshDomainfor obstacle move. " << std::endl;
			}


		}
	}

	void ACCLMeshModule::processMouseButtonEvent(int button, int action)
	{

	}

	// External exported functions
	PLUGIN_API SteerLib::ModuleInterface * createModule()
	{
		return new ACCLMeshModule();
	}

	PLUGIN_API SteerLib::PlanningDomainInterface * getPathPlanner(SteerLib::PlanningDomainModuleInterface*  module)
	{
		return module->getPathPlanner();
	}

	PLUGIN_API void destroyModule( SteerLib::ModuleInterface*  module )
	{
		if (module) delete module; module = NULL;
	}

} /* namespace ACCLMesh */


