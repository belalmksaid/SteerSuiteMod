//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//
/*
 * ACCLMeshDomain.cpp
 *
 *  Created on: 2015-09-01
 *      Author: gberseth
 */

#include "ACCLMeshDomain.h"
#include "astar/AStarLite.h"
#include "ACCLMeshEnvironment.h"

namespace ACCLMesh {

	ACCLMeshDomain::ACCLMeshDomain(SteerLib::EngineInterface * engineInfo , ACCLMesh *  mesh)
	{
		// TODO Auto-generated constructor stub
		_engine = engineInfo;
		_original_mesh = mesh;
		_original_mesh->drawState = 1;
		_mesh = new ACCLMesh();
		// _mesh = mesh;
		_mesh->initFromMesh(_original_mesh);
		_mesh->drawState = 1;
		_targetNode=0;

	}

	ACCLMeshDomain::~ACCLMeshDomain() {
		// TODO Auto-generated destructor stub
	}

	ACCLMesh * ACCLMeshDomain::getNavMesh()
	{
		return _mesh;
	}

	bool ACCLMeshDomain::findPath (Util::Point &startPosition, Util::Point &endPosition, std::vector<Util::Point> & path,
			unsigned int _maxNodesToExpandForSearch)
	{
		AStarLite searchAlgorithm;
		size_t startNode = this->_mesh->closestVert(startPosition);
		size_t targetNode = this->_mesh->closestVert(endPosition);
		ACCLMeshEnvironment acclmeshPlanningEnvironment(this, 100, 500);
		acclmeshPlanningEnvironment.setMaxNumNodesToExpand(50000);
		if ( !searchAlgorithm.findPath( acclmeshPlanningEnvironment, startNode, targetNode ) )
		{
			std::cout << "******* Plan NOT found ********" << std::endl;
			// std::cout << goalState << std::endl;
			return false;
		}
		else
		{
			std::cout << "******* Plan found ********" << std::endl;
			// std::cout << goalState << std::endl;

		}

		// footstepPlanProfiler.stop();

		if (searchAlgorithm.getPath().size() != 0)
		{
			// std::cout << "Found a path: " << searchAlgorithm.getPath().size() << " nodes long" << std::endl;
			for (size_t p=0; p < searchAlgorithm.getPath().size(); p++ )
			{
				size_t index = searchAlgorithm.getPath().at(searchAlgorithm.getPath().size()-p-1);
				std::cout << index << ", " ;
				path.push_back(this->_mesh->get_vertex(index) + (this->_mesh->get_vnormal(index)*0.05f));
			}
			// std::cout << std::endl ;
			return true;

		}
		return false;
	}

	bool ACCLMeshDomain::findSmoothPath (Util::Point &startPosition, Util::Point &endPosition, std::vector<Util::Point> & path,
			unsigned int _maxNodesToExpandForSearch)
	{
		// Util::Point pos = _engine->getAgents().at(0)->position();
		return false;
	}

	bool ACCLMeshDomain::refresh()
	{
		// this->_sample->reset();
		// std::cout << "this is the planner:" << this << std::endl;
		// This needs to be here because obstacles are not put in _engine until after init();
		// Mesh * mesh = new Mesh();
		// mesh->init(xyzPositions,triVerts);
		// std::pair<std::vector<Util::Point>,std::vector<size_t> > mesh_stuff = box.getGeometry();
		// std::cout << "About to get some more geometry" << _engine << std::endl;
		// this call does not want to work... It has given me problems before... engin doesn't like being passed between many functions'
		// std::pair<std::vector<Util::Point>,std::vector<size_t> > mesh_stuff = _engine->getStaticGeometry();
		// mesh->init(mesh_stuff.first,mesh_stuff.second);

		// geom->loadMesh(&ctx, meshPath);

		_mesh->initFromMesh(_original_mesh);
		this->moveObstacle(this->_obs_displacement);

		_tmpPath.clear();
		this->_mesh->calculate_surface_acceleration(0.6, 0.10);
		std::cout << "*****Refreshing mesh" << std::endl;
		// this->_mesh->calculate_surface_acceleration(1.0, 0.010);
		this->_mesh->meshClearanceRefinement(2.0);
		// this->_mesh->checkForBadEdges();
		this->_mesh->refine_mesh_by_acceleration2();
		std::cout << "number of acceleration verts: " << this->_mesh->m_curvatureData.acceleration.size() << std::endl;
		std::cout << "number of mesh verts: " << this->_mesh->get_vert_size() << std::endl;

		// TODO should be able to plan here
		// Util::PerformanceProfiler footstepPlanProfiler;
		// footstepPlanProfiler.reset();
		// footstepPlanProfiler.start();
/*
		AStarLite searchAlgorithm;

		ACCLMeshEnvironment acclmeshPlanningEnvironment(this, 100, 500);
		acclmeshPlanningEnvironment.setMaxNumNodesToExpand(50000);
		if ( !searchAlgorithm.findPath( acclmeshPlanningEnvironment, 100, _targetNode ) )
		{
			std::cout << "******* Plan NOT found ********" << std::endl;
			// std::cout << goalState << std::endl;
		}
		else
		{
			std::cout << "******* Plan found ********" << std::endl;
			// std::cout << goalState << std::endl;

		}

		// footstepPlanProfiler.stop();

		if (searchAlgorithm.getPath().size() != 0)
		{
			std::cout << "Found a path: " << searchAlgorithm.getPath().size() << " nodes long" << std::endl;
			for (size_t p=0; p < searchAlgorithm.getPath().size(); p++ )
			{
				std::cout << searchAlgorithm.getPath().at(searchAlgorithm.getPath().size()-p-1) << ", " ;
				this->_tmpPath.push_back(this->_mesh->get_vertex(searchAlgorithm.getPath().at(searchAlgorithm.getPath().size()-p-1)));
			}
			std::cout << std::endl ;

		}
		*/
		return true;
	}

	std::pair<std::vector<Util::Point> , std::vector<size_t>> ACCLMeshDomain::getNavMeshGeometry()
	{
		std::pair<std::vector<Util::Point> , std::vector<size_t>> obj_data;
		return obj_data;
	}
	std::pair<std::vector<Util::Point> , std::vector<size_t>> ACCLMeshDomain::getEnvironemntGeometry()
	{
		std::pair<std::vector<Util::Point> , std::vector<size_t>> obj_data;
		return obj_data;
	}

	void ACCLMeshDomain::draw()
	{
	#ifdef ENABLE_GUI
		if ( this->_mesh )
		{
			// std::cout << "drawing mesh" << std::endl;
			this->_mesh->drawMesh();
		}

		// Draw current path
		/*
		for (size_t p=0; p < (this->_tmpPath.size()-1); p++ )
		{
			Util::DrawLib::drawLine(this->_tmpPath.at(p), this->_tmpPath.at(p+1), Util::gYellow, 3.0);
		}
		*/
	#endif
	}

	void ACCLMeshDomain::moveObstacle(Util::Vector delta)
	{
		this->_obs_displacement = delta;
		size_t index=0;
		std::vector<size_t> obstacle = {88, 87, 86, 85, 84, 83, 82, 81, 80,
				35, 46, 66, 69, 73, 44, 45, 77};
		for (size_t p=0; p < obstacle.size(); p++)
		{
			this->_mesh->set_vertex(obstacle[p], this->_mesh->get_vertex(obstacle[p]) + this->_obs_displacement);
		}
	}


} /* namespace ACCLMesh */
