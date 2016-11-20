
//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#include "math.h"
#include "ACCLMeshEnvironment.h"


//using namespace FootstepGlobals;

namespace ACCLMesh {

	ACCLMeshEnvironment::ACCLMeshEnvironment( ACCLMeshDomain * mesh, size_t startVert, size_t targetVert  )
	{
		_numNodesExpanded = 0;
		_mesh = mesh;
		_startNode = startVert;
		_targetNode = targetVert;
	}


	float ACCLMeshEnvironment::getHeuristic(int start, int target) const
	{
		// const Footstep & currentStep = agent->_cachedFootstepOptions[start];
		// const Footstep & targetStep = agent->_cachedFootstepOptions[target];
		// Glen I think lengthSquared works ok here.
		// (_mesh->get_vertex(start) - _mesh->get_vertex(target)).lengthSquared();

		return (_mesh->getNavMesh()->get_vertex(start) - _mesh->getNavMesh()->get_vertex(target)).length();
	}


	bool ACCLMeshEnvironment::_isValidNode(size_t id) const
	{
		if ( ( id >= this->_mesh->getNavMesh()->m_curvatureData.acceleration.size() ) ||
				( this->_mesh->getNavMesh()->m_curvatureData.acceleration.at(id) <
				this->_mesh->getNavMesh()->m_curvatureData.max_acceleration_allowed))
		{
			return true;
		}

		return false;
	}

	void ACCLMeshEnvironment::getSuccessors(int nodeId, int lastNodeId, vector<Successor> & result) const
	{
		result.clear();

		// seems like the planner usually does not expand more than a few thousand nodes in bad cases
		// (the worst observed so far was about 3000 nodes expanded, 5000 nodes generated)
		// so, we can cut-off at 10,000 expanded nodes without worrying too much that we are loosing something important.
		// this prevents a pathological pedestrian from stalling the system.
		_numNodesExpanded++;

		// On average only 70 nodes are expanded so I really dropped the bound to help speed through bad scenarios.
		if (_numNodesExpanded > _maxNumNodesToExpand)
		{
			// std::cout << "Footsteps expanded over " << _numNodesExpanded << " nodes before giving up." << std::endl;
			return;
		}

		if ( lastNodeId == _targetNode )
		{
			Successor finalState(1, 0.0f);
			result.push_back(finalState);
			std::cout << "Footsteps expanded over " << _numNodesExpanded << " nodes to reach goal." << std::endl;
			return;
		}

		// std::cout << "Planning, Number of verts in mesh: " << _mesh->getNavMesh()->get_vert_size() << std::endl;
		std::vector<size_t> v_neighbours= _mesh->getNavMesh()->get_vert_neighbours(nodeId);

		for (size_t v=0; v < v_neighbours.size(); v++)
		{
			if ( (v_neighbours.at(v) != lastNodeId) &&
					this->_isValidNode(v_neighbours.at(v)) )
			{ // only add if not above acceleration and not last node
				// Geodesic cost?
				float dist = ( this->_mesh->getNavMesh()->get_vertex(nodeId) -
						this->_mesh->getNavMesh()->get_vertex(v_neighbours.at(v)) ).length();
				double acceleration = 0.0;
				if ( this->_mesh->getNavMesh()->m_curvatureData.acceleration.size() < nodeId )
				{
					// acceleration = this->_mesh->getNavMesh()->m_curvatureData.acceleration.at(nodeId)*1000.0;
				}
				Successor pathOption(v_neighbours.at(v), dist);
				result.push_back(pathOption);
			}
		}


	}

	bool ACCLMeshEnvironment::isValidNodeId(int nodeId) const
	{
		// TODO for testing code, do a real verification.

		// for performance code, return true
		return true;
	}

}

