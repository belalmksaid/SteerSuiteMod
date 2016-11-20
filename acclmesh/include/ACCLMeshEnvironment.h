//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//
/*
 * ACCLMeshEnvironment.h
 *
 *  Created on: Mar 6, 2015
 *      Author: glenpb
 */

#ifndef ACCLMESH_ACCLMESHENVIRONMENT_H_
#define ACCLMESH_ACCLMESHENVIRONMENT_H_

#include "astar/Environment.h"
#include "ACCLMeshDomain.h"

namespace ACCLMesh {

	class ACCLMeshEnvironment : public Environment
	{
	public:
		ACCLMeshEnvironment( ACCLMeshDomain * mesh, size_t startVert, size_t targetVert );
		float getHeuristic(int start, int target) const;
		void getSuccessors(int nodeId, int lastNodeId, vector<Successor> & result) const;
		bool isValidNodeId(int nodeId) const;

	protected:
		ACCLMeshDomain * _mesh;
		bool _isValidNode(size_t id) const;

	public:
		mutable unsigned int _numNodesExpanded;
		void setMaxNumNodesToExpand(unsigned int nodes) { this->_maxNumNodesToExpand = nodes; }

	private:
		unsigned int _maxNumNodesToExpand;
		size_t _startNode;
		size_t _targetNode;
	};

}




#endif /* ACCLMESH_INCLUDE_ACCLMESHENVIRONMENT_H_ */
