//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//
/*
 * MeshDataBaseModule.cpp
 *
 *  Created on: 2014-11-29
 *      Author: glenpb
 *
 * ./bin/navmeshBuilder -ai modules/sfAI -testcase ../testcases/4-way-oncomming.xml -module navmesh -config config-navmesh.xml
 */

#include "SteerLib.h"
#include "MeshDataBaseModule.h"
#include "MeshDataBase.h"
#include "MeshUtils.h"
#include "obstacles/PolygonObstacle.h"


namespace MeshDataBaseGlobals
{
	SteerLib::EngineInterface * gEngine;
	SteerLib::SpatialDataBaseInterface * gSpatialDatabase;

	PhaseProfilers * gPhaseProfilers;
}


using namespace SteerLib;
using namespace MeshDataBaseGlobals;

void MeshDataBaseModule::init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo )
{

	_engine = engineInfo;

	// iterate over all the options
	SteerLib::OptionDictionary::const_iterator optionIter;
	for (optionIter = options.begin(); optionIter != options.end(); ++optionIter) {
		if ((*optionIter).first == "technique") {
			// _techniqueName = (*optionIter).second;
		}
		else if ((*optionIter).first == "loadGeometry") {
			_meshFileName = (*optionIter).second;
		}
		else {
			throw Util::GenericException("unrecognized option \"" + Util::toString((*optionIter).first) + "\" given to meshdatabase module.");
		}
	}

	std::cout << "Number of obstacles in engine: " << _engine->getObstacles().size() << std::endl;
	std::cout << "Number of agents in engine: " << _engine->getAgents().size() << std::endl;
	// gEngine = _engine;
	ACCLMesh::ACCLMesh * mesh = loadACCLMeshFromFile(_meshFileName);
	// Mesh * mesh = new Mesh();
	std::cout << "loaded mesh: " << _meshFileName << ", " << mesh << ", num verts " << mesh->get_vert_size() << std::endl;
	std::cout << "*****Refreshing mesh" << std::endl;
	mesh->calculate_surface_acceleration(1.0, 0.070);
	// mesh->meshClearanceRefinement(2.0);
	mesh->checkForBadEdges();
	mesh->refine_mesh_by_acceleration2();
	std::cout << "number of acceleration verts: " << mesh->m_curvatureData.acceleration.size() << std::endl;
	std::cout << "number of mesh verts: " << mesh->get_vert_size() << std::endl;

	std::vector<std::vector<size_t> > _obstacles = mesh->getObstacles();
	std::vector<std::vector<Util::Point> > _obs;
	for (size_t obs=0; obs < _obstacles.size(); obs++)
	{
		std::vector<Util::Point> ob;
		for (size_t _vert=0; _vert < _obstacles.at(obs).size(); _vert++)
		{
			Util::Point _p = mesh->get_vertex(_obstacles.at(obs).at(_vert));
			// _p.y=0.0;
			ob.push_back(_p);
		}
		 std::reverse(ob.begin(),ob.end());
		_obs.push_back(ob);
	}

	for (size_t t_obs=0; t_obs < _obs.size(); t_obs++)
	{
		PolygonObstacle * po = new PolygonObstacle(_obs.at(t_obs));
		_engine->addObstacle(po);
	}


	this->_spatialDatabase = new MeshDataBase(mesh, _engine);
	// this->_pathPlanner = new RecastNavMeshPlanner(engineInfo);

	// gSpatialDatabase = engineInfo->getSpatialDatabase();
	// _sample = createSolo();
	// std::cout << "" << _engine->getStaticGeometry().first.at(0);
	// std::cout << "Got some geometry" << engineInfo << std::endl;

	/*mesh->add_vertex(0,0,0);
	mesh->add_vertex(12,0,0);
	mesh->add_vertex(0,12,0);
	mesh->add_face(0,1,2);
	*/
	// this->_spatialDatabase->init(options, _engine);
	// _benchmarkTechnique->setEngineInterface(_engine);
}


void MeshDataBaseModule::initializeSimulation()
{
	std::cout << "initialize Number of obstacles in engine: " << _engine->getObstacles().size() << std::endl;
}

void MeshDataBaseModule::preprocessSimulation()
{
	if (this->_meshFileName != "")
	{
		// this->saveStaticGeometryToObj(this->_meshFileName);
	}
	// This needs to be here because obstacles are not put in _engine until after init();

	// this->getPathPlanner()->refresh();

	// Mesh * mesh = new Mesh();
	// std::pair<std::vector<Util::Point>,std::vector<size_t> > mesh_stuff = dynamic_cast<RecastNavMeshPlanner *>( this->getPathPlanner())->getNavMeshGeometry();

	// mesh->init(mesh_stuff.first,mesh_stuff.second);


	// _spatialDatabase = new NavMeshDataBase(mesh);
	for (size_t agent=0; agent < _engine->getAgents().size(); agent++)
	{
		_engine->selectAgent(_engine->getAgents().at(agent));
	}
	// TODO this is not a great place to do this
	// this->_spatialDatabase->_agent_face_map.resize(gEngine->getAgents().size());
	this->_spatialDatabase->buildObstacleTree();

}

void MeshDataBaseModule::preprocessFrame(float timeStamp, float dt, unsigned int frameNumber)
{
	if ( frameNumber == 1)
	{
		// Adding in this extra one because it seemed sometimes agents would forget about obstacles.
		this->_spatialDatabase->buildObstacleTree();
	}

	this->_spatialDatabase->buildAgentTree();

}

void MeshDataBaseModule::postprocessFrame(float timeStamp, float dt, unsigned int frameNumber) {
	// std::cout << "found the business? " << _engine->getAgents().size() << std::endl;
	// _benchmarkTechnique->update(_metricsCollectorModule->getSimulationMetrics(), timeStamp, dt);
	// _benchmarkTechnique->update( _engine, timeStamp, dt);
}


void MeshDataBaseModule::postprocessSimulation() {
}


void MeshDataBaseModule::draw()
{
#ifdef ENABLE_GUI
	// _spatialDatabase->draw();
	// std::cout << "drawing navmesh Module" << std::endl;

	// std::cout << "Path " << _navTool->getPath().at(0) << std::endl;
	/*
	std::vector<Util::Point> points;
	for (size_t p=0; p < _navTool->getPath().size(); p +=3)
	{
		points.push_back( Util::Point( _navTool->getPath().at(p), _navTool->getPath().at(p+1), _navTool->getPath().at(p+2) ) + Util::Point(0.0,0.6,0.0) );
	}

	DrawLib::drawStar(points.at(0), Util::Vector(1,0,0), 0.34f, gBlue);
	for (size_t p=1; p < points.size(); p++)
	{
		DrawLib::drawLine(points.at(p), points.at(p-1), gBlack, 5.0f);
		DrawLib::drawStar(points.at(p), Util::Vector(1,0,0), 0.34f, gOrange);
		// std::cout << "point " << p << ", " << points.at(p) << std::endl;
	}
*/
#endif

}

void MeshDataBaseModule::finish()
{


}

void MeshDataBaseModule::saveStaticGeometryToObj(std::string fileName)
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

// External exported functions
PLUGIN_API SteerLib::ModuleInterface * createModule()
{
	return new MeshDataBaseModule();
}

PLUGIN_API SpatialDataBaseInterface * getSpatialDataBase(SteerLib::SpatialDataBaseModuleInterface*  module)
{
	return module->getSpatialDataBase();
}


PLUGIN_API void destroyModule( SteerLib::ModuleInterface*  module )
{
	if (module) delete module; module = NULL;
}




