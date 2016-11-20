//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//
/*
 * ACCLMeshDomain.h
 *
 *  Created on: 2015-09-01
 *      Author: gberseth
 */

#ifndef ACCLMESHDOMAIN_H_
#define ACCLMESHDOMAIN_H_

#include "interfaces/PlanningDomainInterface.h"
#include "ACCLMesh.h"
#include "interfaces/EngineInterface.h"

namespace ACCLMesh {

class ACCLMeshDomain : public SteerLib::PlanningDomainInterface {
public:
	ACCLMeshDomain(SteerLib::EngineInterface * engineInfo, ACCLMesh * mesh);
	virtual ~ACCLMeshDomain();

	/// @name Path planning queries
		//@{
		/// Returns "true" if a path was found from startLocation to goalLocation, or "false" if no complete path was found; in either case, the path (complete if returning true, or partial path if returning false) is stored in outputPlan as a sequence of grid cell indices.

		virtual bool findPath (Util::Point &startPosition, Util::Point &endPosition, std::vector<Util::Point> & path,
				unsigned int _maxNodesToExpandForSearch);

		virtual bool findSmoothPath (Util::Point &startPosition, Util::Point &endPosition, std::vector<Util::Point> & path,
				unsigned int _maxNodesToExpandForSearch);

		/// update navmesh
		virtual bool refresh();
		//@}

		// If there is anything to draw
		virtual void draw();

		void moveObstacle(Util::Vector delta);

		virtual std::pair<std::vector<Util::Point> , std::vector<size_t>> getNavMeshGeometry();
		virtual std::pair<std::vector<Util::Point> , std::vector<size_t>> getEnvironemntGeometry();

		SteerLib::EngineInterface * _engine;

		ACCLMesh * getNavMesh();
		size_t _targetNode;

	private:
		ACCLMesh * _mesh;
		ACCLMesh * _original_mesh;
		Util::Vector _obs_displacement;
		std::vector<Util::Point> _tmpPath;
};

} /* namespace ACCLMesh */
#endif /* ACCLMESHDOMAIN_H_ */
