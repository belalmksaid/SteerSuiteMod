//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//
/*
 * ACCLMesh.h
 *
 *  Created on: Mar 6, 2015
 *      Author: gberseth
 */

#ifndef ACCLMESH_ACCLMESH_H_
#define ACCLMESH_ACCLMESH_H_

// #include "SteerLib.h"
#include "Mesh.h"

namespace ACCLMesh {

class ACCLMesh : public Mesh {
public:
	ACCLMesh();
	virtual ~ACCLMesh();
	void clear();

public:

	curvature_data m_curvatureData;
	double accelerationCutFunction(size_t he);
	double tightCutFunction(size_t he);
	void refine_mesh_by_acceleration2();
	void splitHalfEdge(size_t he, double dist);
	void addEdge(size_t hePrev, size_t heNextPrev);
	void setFaceForHalfEdges(size_t face, std::vector<size_t> hes);
	/// calculate the curvature with respect to the unit speed scale and max acceleration
	void calculate_surface_acceleration(double unitSize, double maxacceleration);
	/// calculates the curvature for the mesh surface
	void calculate_curvature();
	double mean_cotan_curvature(size_t vert);
	double gaussian_cotan_curvature(size_t vert);
	double total_cotan_curvature(size_t vert);
	double max_principal_curvatures(size_t vert);
	double gaussian_curvature(size_t vert);
	double acclmesh_curvature(size_t vert);

	double calculateFaceArea(const size_t face) const;
	virtual size_t closestVert(Util::Point p);
	bool isBoundaryVertex(size_t vert);
	bool isBoundaryHalfEdge(half_edge * he);
	std::vector<std::vector<size_t> > getObstacles();
	void meshClearanceRefinement(double clearance);
	bool checkForBadEdges();
	double accelerationForBadEdges(size_t he);

	virtual void drawMesh();
	virtual void initFromMesh(Mesh *mesh);
	virtual std::vector<size_t> get_faces_for_vert(size_t oldV) const;

};

inline void ACCLMesh::clear(){
	subdiv_iter = 0;
	edit_count = 0;

	m_heData.clear();
	m_faceData.clear();
	m_vertData.clear();
	m_vertices.clear();
	// _simplifyPriorityQueue.clear();
	// selected_verts.clear();
	// selected_verts_neighbours.clear();

	// clear curvature data
	this->m_curvatureData.acceleration.clear();
	this->m_curvatureData.curvature.clear();

}

} /* namespace ACCLMesh */


#endif /* ACCLMESH_SRC_ACCLMESH_H_ */

