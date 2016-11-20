//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//
/*
 * ACCLMesh.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: gberseth
 */

#include "ACCLMesh.h"

#include "util/DrawLib.h"

#include <cmath>

// * Not sure if I want to add this library to SteerSuite yet
#include "GTEngine/Include/Mathematics/GteDistTriangle3Triangle3.h"
#include "GTEngine/Include/Mathematics/GteDistPointTriangle.h"
#include "GTEngine/Include/Mathematics/GteDistPointSegment.h"
// #include <GTEngine.h>
#include "GTEngine/Include/Mathematics/GteTriangle.h"
#include "GTEngine/Include/Mathematics/GteVector.h"
#include "GTEngine/Include/Mathematics/GteSegment.h"


#include <map>
#include <set>
#include <iostream>
#include <memory>
#include <fstream>
#include <vector>
#include <algorithm>
#include <limits>

#ifdef _WIN32
	#ifdef max()
	#undef max()
	#endif
	#ifdef min()
	#undef min()
	#endif
#include <functional>
#endif


namespace ACCLMesh {

	ACCLMesh::ACCLMesh()
	{

	}

	ACCLMesh::~ACCLMesh() {
		// TODO Auto-generated destructor stub
	}

	/*
		Accelerations at every vertext mush have already been calculated
	*/
	double ACCLMesh::accelerationCutFunction(size_t he)
	{ // linear cut function for now.
		double acc_under = this->m_curvatureData.acceleration[this->m_heData[he].vert];
		double acc_over = this->m_curvatureData.acceleration[twin(this->m_heData[he]).vert];
		// assert acc_over > mac_acc > acc_under
		return (this->m_curvatureData.max_acceleration_allowed-acc_under) / (acc_over - acc_under);
	}



	/**
		Returns arbitrary tight bound
	*/
	double ACCLMesh::tightCutFunction(size_t he)
	{
		return 0.90;
	}

	/**
		Calculates the area of a face using

	*/
	double ACCLMesh::calculateFaceArea(const size_t face) const
	{

		half_edge he = this->m_heData[this->m_faceData[face]];
		Util::Vector a = normalize(this->get_vertex(he.vert) - (this->get_vertex(next(he).vert)));
		Util::Vector b = normalize(this->get_vertex(next(he).vert) - (this->get_vertex(next(next(he)).vert)));
		// Radians
		double angle = std::acos(dot(a,b));

		return 0.5f * (a.norm() * b.norm()) * std::sin(angle);


		// std::cout << "vert " << i << " vert " << next << " dot " << (vectors.at(i).dot( vectors.at(next) ) ) << std::endl;
	}


	double ACCLMesh::mean_cotan_curvature(size_t vert)
	{

		vvert_iterator it;
		Util::Vector curvature = Util::Vector(0,0,0);
		double area = 0;
		this->init_iterator( it, vert );
		do {
			if ( it.m_cur->face != HOLE_INDEX )
			{ // Don't do this for holes
				// double _epsilon = 0.004;
				Util::Vector edge_length = (this->get_vertex(it.m_cur->vert) - this->get_vertex(twin(*it.m_cur).vert));
				double edge_length_squared = (this->get_vertex(it.m_cur->vert) - this->get_vertex(twin(*it.m_cur).vert)).lengthSquared();
				// Util::Vector next_normal = (this->get_vertex(next(*it.m_cur).vert) - this->get_vertex(twin(next(*it.m_cur)).vert)).normalized();

				double cotan_weight = this->get_cotan_weight(it);
				// double he_length = (this->get_vertex(it.m_cur->vert) - this->get_vertex(twin(*it.m_cur).vert)).norm();

				curvature += (cotan_weight * edge_length); // * he_length;
				if (curvature != curvature)
				{ // Numberical issues with obtuse angles
					std::cout << "CURVATURE total: " << curvature << std::endl;
					// this->m_curvatureData.curvature.push_back(0.0);
				}
				// area += this->calculateFaceArea(it.m_cur->face);
				Util::Vector this_edge = normalize(this->get_vertex((*it.m_cur).vert) - this->get_vertex(twin(*it.m_cur).vert));
				Util::Vector next_edge = normalize(this->get_vertex(twin(next(*it.m_cur)).vert) - this->get_vertex(next(*it.m_cur).vert));
				double dot_p = dot( this_edge, next_edge );

				if ( dot_p < -(0.004) )
				{
					// std::cout << "Found obtuse Triangle" << std::endl;
					area +=  (0.125) * cotan_weight * edge_length_squared;
					if (area != area)
					{
						// Numerical issues with obtuse angles
						std::cout << "area total: " << area << " dot_p " << dot_p << std::endl;
						// this->m_curvatureData.curvature.push_back(0.0);
					}
				}
				else if ( dot_p == ( 0.0 ) )
				{
					area += (0.25) * this->calculateFaceArea(it.m_cur->face);
				}
				else
				{
					area +=  (0.50) * this->calculateFaceArea(it.m_cur->face);
				}
			}

		} while( this->advance_iterator( it ) );

		return ((1.0/(2.0*(area)))* curvature).norm()/2.0;
	}

	double ACCLMesh::gaussian_cotan_curvature(size_t vert)
	{
		vvert_iterator it;
		Util::Vector curvature = Util::Vector(0,0,0);
		double angle_sum = 0;
		double area = 0;
		double dot_p = 0;
		this->init_iterator( it, vert );
		do {
			if ( it.m_cur->face != HOLE_INDEX )
			{ // Don't do this for holes
				Util::Vector edge_length = (this->get_vertex(it.m_cur->vert) - this->get_vertex(twin(*it.m_cur).vert));
				double edge_length_squared = (this->get_vertex(it.m_cur->vert) - this->get_vertex(twin(*it.m_cur).vert)).lengthSquared();
				double cotan_weight = this->get_cotan_weight(it);

				curvature += (cotan_weight * edge_length); // * he_length;
				if (curvature != curvature)
				{ // Numberical issues with obtuse angles
					std::cout << "CURVATURE total: " << curvature << std::endl;
				}
				Util::Vector this_edge = normalize(this->get_vertex((*it.m_cur).vert) - this->get_vertex(twin(*it.m_cur).vert));
				Util::Vector next_edge = normalize(this->get_vertex(twin(next(*it.m_cur)).vert) - this->get_vertex(next(*it.m_cur).vert));
				dot_p = dot( this_edge, next_edge );
				angle_sum += std::acos(dot_p);

				if ( dot_p < -(0.004) )
				{ // obtuse triangle
					area +=  (0.125) * cotan_weight * edge_length_squared;
					if (area != area)
					{
						std::cout << "area total: " << area << " dot_p " << dot_p << std::endl;
					}
				}
				else if ( dot_p == ( 0.0 ) )
				{
					area += (0.25) * this->calculateFaceArea(it.m_cur->face);
				}
				else
				{
					area +=  (0.50) * this->calculateFaceArea(it.m_cur->face);
				}
			}
		} while( this->advance_iterator( it ) );
		// std::cout << "angle sum: " << angle_sum << " area " << area << " and dot_p is " << dot_p << std::endl;
		return (((M_PI*2.0)-angle_sum)/(area));
	}

	double ACCLMesh::total_cotan_curvature(size_t vert)
	{
		vvert_iterator it;
		Util::Vector curvature = Util::Vector(0,0,0);
		double angle_sum = 0;
		this->init_iterator( it, vert );
		do {
			if ( it.m_cur->face != HOLE_INDEX )
			{ // Don't do this for holes
				Util::Vector edge_length = (this->get_vertex(it.m_cur->vert) - this->get_vertex(twin(*it.m_cur).vert));
				double edge_length_squared = (this->get_vertex(it.m_cur->vert) - this->get_vertex(twin(*it.m_cur).vert)).lengthSquared();
				double cotan_weight = this->get_cotan_weight(it);

				curvature += (cotan_weight * edge_length); // * he_length;
				if (curvature != curvature)
				{ // Numberical issues with obtuse angles
					std::cout << "CURVATURE total: " << curvature << std::endl;
				}
				Util::Vector this_edge = normalize(this->get_vertex((*it.m_cur).vert) - this->get_vertex(twin(*it.m_cur).vert));
				Util::Vector next_edge = normalize(this->get_vertex(twin(next(*it.m_cur)).vert) - this->get_vertex(next(*it.m_cur).vert));
				double dot_p = dot( this_edge, next_edge );
				angle_sum += std::acos(dot_p);

			}
		} while( this->advance_iterator( it ) );

		return (((M_PI*2.0)-angle_sum));
	}

	double ACCLMesh::max_principal_curvatures(size_t vert)
	{
		double mean_curvature = this->mean_cotan_curvature(vert);
		double gaussian_curvature = this->gaussian_cotan_curvature(vert);
		double tri_x = (mean_curvature*mean_curvature) - gaussian_curvature;
		if ( tri_x < 0.0 )
		{
			// std::cout << "tri_x < 0" << std::endl;
			tri_x = 0.0;
		}
		tri_x = (sqrt(tri_x));
		double k_max = std::fabs(mean_curvature + (tri_x));
		double k_min = std::fabs(mean_curvature - (tri_x));

		if ( k_max > k_min)
		{
			return k_max;
		}
		return k_min;

	}

	/*
	 * when roof-like or sphere-like use mean curvature
	 * when saddle us total curvature
	 */
	double ACCLMesh::acclmesh_curvature(size_t vert)
	{
		double mean_curvature = this->mean_cotan_curvature(vert);
		double gaussian_curvature = this->gaussian_cotan_curvature(vert);
		double tri_x = (mean_curvature*mean_curvature) - gaussian_curvature;
		if ( tri_x < 0.0 )
		{ // What does this mean??
			std::cout << "tri_x < 0" << std::endl;
			tri_x = 0.0;
		}
		tri_x = (sqrt(tri_x));
		double k_max = (mean_curvature + (tri_x));
		double k_min = (mean_curvature - (tri_x));

		double epsilon = 0.004;
		if ( (k_min < -(0.0+epsilon)) && (k_max > (0.0+epsilon)) )
		{ // saddle point
			// return this->total_cotan_curvature(vert);
			return this->max_principal_curvatures(vert);
			// return gaussian_curvature;
		}
		else
		{
			return mean_curvature;
		}

		return 0.0;
	}

	void ACCLMesh::calculate_curvature()
	{
		this->m_curvatureData.min_curvature = std::numeric_limits<double>::max();
		this->m_curvatureData.max_curvature = std::numeric_limits<double>::min();
		this->m_curvatureData.curvature.clear();
		// this->m_curvatureData.normalizedCurvature.clear();

		double sum = 0;
		for (size_t vert=0; vert < this->get_vert_size(); vert++)
		{
			// double curvature = this->mean_curvature(vert);
			// double curvature = std::fabs(this->gaussian_cotan_curvature(vert));
			double curvature = std::fabs(this->max_principal_curvatures(vert));
			// double curvature = std::fabs(this->total_cotan_curvature(vert));
			// double curvature = this->mean_cotan_curvature(vert);
			// double curvature = std::fabs(this->acclmesh_curvature(vert));
			if ( curvature < this->m_curvatureData.min_curvature )
			{
				this->m_curvatureData.min_curvature = curvature;
			}
			else if ( curvature > this->m_curvatureData.max_curvature )
			{
				this->m_curvatureData.max_curvature = curvature;
			}
			// this->m_curvatureData.curvature.push_back(curvature);
			// std::cout << "Current total: " << sum << " adding " << curvature << std::endl;
			if (curvature != curvature)
			{ // Numberical issues with obtuse angles
				std::cout << "Current total: " << sum << " adding " << curvature << std::endl;
				this->m_curvatureData.curvature.push_back(0.001);
			}
			else if (curvature == 0.0)
			{
				this->m_curvatureData.curvature.push_back(0.001);
			}
			else
			{
				this->m_curvatureData.curvature.push_back(curvature);
				sum += curvature;
			}


		}
		this->m_curvatureData.average_curvature = sum/(1.0*this->m_curvatureData.curvature.size());
		std::cout << "Max curvature: " << this->m_curvatureData.max_curvature << std::endl;
		std::cout << "Min curvature: " << this->m_curvatureData.min_curvature << std::endl;
		std::cout << "Avg curvature: " << this->m_curvatureData.average_curvature << std::endl;
		std::cout << "Total curvature: " << sum << std::endl;
		std::cout << "Num Curvature Points: " << this->m_curvatureData.curvature.size() << std::endl;

		this->edit_count++;
	}

	void ACCLMesh::calculate_surface_acceleration(double unitSize, double maxacceleration)
	{
		double velocity_squared = unitSize * unitSize;
		this->m_curvatureData.acceleration.clear();
		this->m_curvatureData.max_acceleration_allowed = maxacceleration;
		this->calculate_curvature();

		for (size_t v=0; v < this->m_curvatureData.curvature.size(); v++)
		{
			this->m_curvatureData.acceleration.push_back(this->m_curvatureData.curvature.at(v) * velocity_squared);
		}


		std::cout << "Max acceleration: " << this->m_curvatureData.max_curvature * velocity_squared << std::endl;
		std::cout << "Min acceleration: " << this->m_curvatureData.min_curvature * velocity_squared << std::endl;
		std::cout << "Avg acceleration: " << this->m_curvatureData.average_curvature * velocity_squared << std::endl;
		std::cout << "Num Vertices: " << this->m_curvatureData.acceleration.size() << std::endl;


	}

	void ACCLMesh::refine_mesh_by_acceleration2()
	{
		/**
			Only split half edges that have one vert < max_acceleration and one
			vert > max acceleration.
		*/
		ACCLMesh *newMesh = new ACCLMesh();
		newMesh->initFromMesh(this);

		std::cout << "Refining ACCLMesh " << std::endl;
		std::set<size_t> visitedEdges;
		size_t hes = this->m_heData.size();
		for ( size_t i = 0; i < hes; i++)
		{// Add vertices between good and bad vertices in the mesh
			if (visitedEdges.find(i) == visitedEdges.end())
			{ // not already visited
				visitedEdges.insert(i);


				if ( ((this->m_curvatureData.acceleration[this->m_heData[i].vert] < this->m_curvatureData.max_acceleration_allowed) &&
					(this->m_curvatureData.acceleration[twin(this->m_heData[i]).vert] > this->m_curvatureData.max_acceleration_allowed))) // ||
					// ((this->m_curvatureData.acceleration[this->m_heData[i].vert] < this->m_curvatureData.max_acceleration_allowed) &&
					// (this->m_curvatureData.acceleration[twin(this->m_heData[i]).vert] > this->m_curvatureData.max_acceleration_allowed)))
				{ // needs to be split
					double dist = accelerationCutFunction(i);
					// double dist = tightCutFunction(i);
					newMesh->splitHalfEdge(i, dist);
					visitedEdges.insert(this->m_heData[i].twin);
				}

			}
		}

		std::cout << "Done splitting edges" << std::endl;
		// I think I can navigate by faces (old faces). Each ld face need to be converted into  new ones.
	// The half edge that the face points to should
		size_t faces = this->get_face_size();
		// std::set<size_t> visitedFaces;
		std::vector<size_t> faces_to_delete;
		for (int f=0; f < faces; f++)
		{


			// Case when all three vertices are good, i.e. do nothing
			if ( (this->m_curvatureData.acceleration[this->m_heData[this->m_faceData[f]].vert] < this->m_curvatureData.max_acceleration_allowed) &&
				(this->m_curvatureData.acceleration[next(this->m_heData[this->m_faceData[f]]).vert] < this->m_curvatureData.max_acceleration_allowed) &&
				(this->m_curvatureData.acceleration[next(next(this->m_heData[this->m_faceData[f]])).vert] < this->m_curvatureData.max_acceleration_allowed))
			{
				continue;
			}
			else if ((this->m_curvatureData.acceleration[this->m_heData[this->m_faceData[f]].vert] > this->m_curvatureData.max_acceleration_allowed) &&
				(this->m_curvatureData.acceleration[next(this->m_heData[this->m_faceData[f]]).vert] > this->m_curvatureData.max_acceleration_allowed) &&
				(this->m_curvatureData.acceleration[next(next(this->m_heData[this->m_faceData[f]])).vert] > this->m_curvatureData.max_acceleration_allowed))
			{ // all verts are bad, remove face
				// Should work, no new verticies were created fr this type
				// newMesh->delete_face(f);
				faces_to_delete.push_back(f);
				/*

				size_t he = newMesh->m_faceData[f];
				std::vector<size_t> hes;
				hes.push_back(he);
				hes.push_back(newMesh->m_heData[he].next);
				hes.push_back(newMesh->m_heData[newMesh->m_heData[he].next].next);
				newMesh->setFaceForHalfEdges(HOLE_INDEX, hes);
				// fix faceData
				// newMesh->m_faceData[f] = he;
				*/
			}
			else
			{
				size_t he;
				if ( ((this->m_curvatureData.acceleration[this->m_heData[this->m_faceData[f]].vert] > this->m_curvatureData.max_acceleration_allowed) &&
					(this->m_curvatureData.acceleration[next(this->m_heData[this->m_faceData[f]]).vert] < this->m_curvatureData.max_acceleration_allowed) &&
					(this->m_curvatureData.acceleration[next(next(this->m_heData[this->m_faceData[f]])).vert] < this->m_curvatureData.max_acceleration_allowed)) ||
					((this->m_curvatureData.acceleration[this->m_heData[this->m_faceData[f]].vert] < this->m_curvatureData.max_acceleration_allowed) &&
					(this->m_curvatureData.acceleration[next(this->m_heData[this->m_faceData[f]]).vert] > this->m_curvatureData.max_acceleration_allowed) &&
					(this->m_curvatureData.acceleration[next(next(this->m_heData[this->m_faceData[f]])).vert] < this->m_curvatureData.max_acceleration_allowed)) ||
					((this->m_curvatureData.acceleration[this->m_heData[this->m_faceData[f]].vert] < this->m_curvatureData.max_acceleration_allowed) &&
					(this->m_curvatureData.acceleration[next(this->m_heData[this->m_faceData[f]]).vert] < this->m_curvatureData.max_acceleration_allowed) &&
					(this->m_curvatureData.acceleration[next(next(this->m_heData[this->m_faceData[f]])).vert] > this->m_curvatureData.max_acceleration_allowed)))
				{ // Two are good one is bad

					if ( (this->m_curvatureData.acceleration[this->m_heData[this->m_faceData[f]].vert] < this->m_curvatureData.max_acceleration_allowed) &&
						(this->m_curvatureData.acceleration[next(this->m_heData[this->m_faceData[f]]).vert] < this->m_curvatureData.max_acceleration_allowed) )
					{
						he = this->m_faceData[f];
					}
					else if ( (this->m_curvatureData.acceleration[next(this->m_heData[this->m_faceData[f]]).vert] < this->m_curvatureData.max_acceleration_allowed) &&
					( this->m_curvatureData.acceleration[next(next(this->m_heData[this->m_faceData[f]])).vert] < this->m_curvatureData.max_acceleration_allowed) )
					{
						he = this->m_heData[this->m_faceData[f]].next;
					}
					else
					{
						he = next(this->m_heData[this->m_faceData[f]]).next;
					}
				}
				else
				{// Two are bad one is good
					if ( (this->m_curvatureData.acceleration[this->m_heData[this->m_faceData[f]].vert] > this->m_curvatureData.max_acceleration_allowed) &&
						(this->m_curvatureData.acceleration[next(this->m_heData[this->m_faceData[f]]).vert] > this->m_curvatureData.max_acceleration_allowed) )
					{// Find bad edge
						he = this->m_faceData[f];
					}
					else if ( (this->m_curvatureData.acceleration[next(this->m_heData[this->m_faceData[f]]).vert] > this->m_curvatureData.max_acceleration_allowed) &&
					( this->m_curvatureData.acceleration[next(next(this->m_heData[this->m_faceData[f]])).vert] > this->m_curvatureData.max_acceleration_allowed) )
					{
						he = this->m_heData[this->m_faceData[f]].next;
					}
					else
					{
						he = next(this->m_heData[this->m_faceData[f]]).next;
					}// get edge pointing into v2

				}
				/*
					g1
				   / \
				  /   v1
				 /     \
				g2--v2--b1
				Need to delete edges {(v2,b1),(b1,v1)} and create new edges between (v2,v1) and (v2,g1)
				Want
					g1
				   /| \
				  / |  v1
				 /  | /
				g2--v2   b1

				Find good edge g1,g2
			*/
				// have to add two edges, lets do one face at a time.
				// get edge pointing into v2 on new mesh

				he = newMesh->m_heData[he].next;
				// he = (newMesh->next(newMesh->next(newMesh->m_heData[he]))).next;
				newMesh->addEdge(he, (newMesh->next(newMesh->next(newMesh->m_heData[he]))).next );
				newMesh->m_faceData.push_back(he);
				size_t face = newMesh->get_face_size()-1;
				std::vector<size_t> hes;
				hes.push_back(he);
				hes.push_back(newMesh->m_heData[he].next);
				hes.push_back(newMesh->m_heData[newMesh->m_heData[he].next].next);
				newMesh->setFaceForHalfEdges(face, hes);
				// maintane faceData
				newMesh->m_faceData[f] = newMesh->next(newMesh->m_heData[he]).twin;

				/* Now
					g1
				   /| \
				  / |  v1
				 /  |   \
				g2--v2---b1
				  he
	*/

				he = newMesh->next(newMesh->m_heData[he]).twin;

				newMesh->addEdge(he, newMesh->next(newMesh->m_heData[he]).next );
				newMesh->m_faceData.push_back(he);
				face = newMesh->get_face_size()-1;
				hes.clear();
				hes.push_back(he);
				hes.push_back(newMesh->m_heData[he].next);
				hes.push_back(newMesh->m_heData[newMesh->m_heData[he].next].next);
				newMesh->setFaceForHalfEdges(face, hes);

				newMesh->m_faceData[f] = newMesh->next(newMesh->m_heData[he]).twin;;

				/* Now
						g1
						/| \
						/ |  \
						/  |   \
					/   |he  v1
					/    |   / \
					/     |  /   \
					/      | /     \
				g2 - - -v2 - - - b1
	*/

				// This doesn't always work with my methods
				he = newMesh->next(newMesh->m_heData[he]).twin;
				hes.clear();
				hes.push_back(he);
				hes.push_back(newMesh->m_heData[he].next);
				hes.push_back(newMesh->m_heData[newMesh->m_heData[he].next].next);
				// newMesh->m_faceData[f] = he;
				// std::sort(hes.begin(), hes.end());
				// std::reverse(hes.begin(), hes.end());
				// size_t _hes[3] = {hes[0], hes[1], hes[2]};

				// newMesh->delete_half_edges_impl(_hes);
				newMesh->setFaceForHalfEdges(f, hes);
				newMesh->m_faceData[f] = he;
				// newMesh->delete_face(f);
			}

		}




		/*
		for (size_t face=0; face < faces_to_delete.size(); face++)
		{
			// newMesh->delete_face(faces_to_delete.at(face));
			// detail::delete_face( newMesh->m_faceData, newMesh->m_heData, face );
		}*/
		std::cout << "Done modifying faces for boundary" << std::endl;
		this->initFromMesh(newMesh);
		this->edit_count++;
		delete newMesh;

	}


	/*
	 * Now checks to make sure this vert is a good vert
	 */
	size_t ACCLMesh::closestVert(Util::Point p)
	{
		long closestVert=-1;
		float closestDist = std::numeric_limits<float>::max();
		for (size_t v=0; v < this->m_curvatureData.acceleration.size(); v++)
		{
			if ( ((p - this->get_vertex(v)).length() < closestDist) &&
					(this->m_curvatureData.acceleration.at(v) < this->m_curvatureData.max_acceleration_allowed))
			{
				closestDist = (p - this->get_vertex(v)).length();
				closestVert = v;
			}
		}
		return closestVert;
	}


	typedef double Real;
	typedef int N;


	void ACCLMesh::meshClearanceRefinement(double clearance)
	{
		using namespace gte;
		const double epsilon = 0.01f;

		std::cout << "Refining mesh, removing triangles wrt clearance" << std::endl;
		DCPQuery<Real, Vector<3, Real>, Triangle<3, Real>>::Result
			        stResult;

		for (size_t _vert=0; _vert < this->get_vert_size(); _vert++)
		{
			Util::Point _p = this->get_vertex(_vert);
			std::array<Real, 3> _point_verts = {_p[0],_p[1],_p[2]};
			Vector<3, Real> point(_point_verts);
			for (size_t _face = 0; _face < this->get_face_size(); _face++)
			{
				Util::Point __v = this->get_vertex(this->m_heData[this->m_faceData[_face]].vert);
				std::array<Real, 3> _v0 = {__v[0],__v[1],__v[2]};
				Vector<3, Real> v0(_v0);
				__v = this->get_vertex(next(this->m_heData[this->m_faceData[_face]]).vert);
				std::array<Real, 3> _v1 = {__v[0],__v[1], __v[2]};
				Vector<3, Real> v1(_v1);
				__v = this->get_vertex(next(next(this->m_heData[this->m_faceData[_face]])).vert);
				std::array<Real, 3> _v2 = {__v[0],__v[1],__v[2]};
				Vector<3, Real> v2(_v2);

				Triangle3<Real> triangle(v0, v1, v2);
				// Vector(std::array<Real, N> const& values);
				DCPQuery<Real, Vector3<Real>, Triangle3<Real>> query;
				auto ptResult = query(point, triangle);
				// std::cout << "distance from face: " << _face << " to origin is " << ptResult.distance << std::endl;
				if ( ptResult.distance < clearance )
				{
					Util::Point closet_point(ptResult.closest[0], ptResult.closest[1], ptResult.closest[2]);
					Util::Vector vec_to_closest_point(closet_point[0]- point[0], closet_point[1]- point[1],
							closet_point[2]- point[2]);
					//std::cout << "distance from face: " << _face << " to origin is " << ptResult.distance << " vector to " <<
						//	vec_to_closest_point <<std::endl;
					vec_to_closest_point = normalize(vec_to_closest_point);

					std::vector<size_t> face_neighbours = this->get_faces_for_vert(_vert);
					for (size_t t_face=0; t_face < face_neighbours.size(); t_face++)
					{
						if (dot(vec_to_closest_point,this->get_fnormal(face_neighbours.at(t_face))) > (0+epsilon))
						{
							// TODO make sure the point is over the face.
							// from http://stackoverflow.com/questions/25512037/how-to-determine-if-a-point-lies-over-a-triangle-in-3d
							Util::Point a = this->get_vertex(this->m_heData[this->m_faceData[face_neighbours.at(t_face)]].vert);
							// Util::Point a = this->get_vertex(next(next(this->m_heData[this->m_faceData[t_face]])).vert);
							Util::Point b = this->get_vertex(next(this->m_heData[this->m_faceData[face_neighbours.at(t_face)]]).vert);
							Util::Point c = this->get_vertex(next(next(this->m_heData[this->m_faceData[face_neighbours.at(t_face)]])).vert);
							// Util::Point c = this->get_vertex(this->m_heData[this->m_faceData[t_face]].vert);
							Util::Vector ba = b-a;
							Util::Vector cb = c-b;
							Util::Vector ac = a-c;
							Util::Vector normal = this->get_fnormal(face_neighbours.at(t_face));
							// Util::Point normal = ac.cross(ba);

							Util::Vector px = closet_point - a;
							Util::Vector px_normal = cross(ba,px);
							if (dot(px_normal,normal) < (0+epsilon) )
								continue;

							px = closet_point - b;
							px_normal = cross(cb,px);
							if (dot(px_normal,normal) < (0+epsilon) )
								continue;

							px = closet_point - c;
							px_normal = cross(ac,px);
							if (dot(px_normal,normal) < (0+epsilon) )
								continue;

							// The point should be above the face now/
							this->m_curvatureData.acceleration[_vert] = this->m_curvatureData.max_acceleration_allowed+1;
						}
					}

				}
			}
		}
		this->edit_count++;

	}


	double ACCLMesh::accelerationForBadEdges(size_t he)
	{
		std::vector<size_t> _verts;
		_verts.push_back(this->m_heData[he].vert);
		_verts.push_back(next(next(this->m_heData[he])).vert);
		_verts.push_back(twin(this->m_heData[he]).vert);
		_verts.push_back(next(next(twin(this->m_heData[he]))).vert);


		Util::Point _new_vert_location = this->get_vertex(this->m_heData[he].vert) +
									(( this->get_vertex(twin(this->m_heData[he]).vert) -
									this->get_vertex(this->m_heData[he].vert))/2.0f);

		// Compute distance for realtive weighting
		double distance_sum = 0.0;
		for (size_t _vert=0; _vert < _verts.size(); _vert++)
		{
			distance_sum += (get_vertex(_verts.at(_vert)) - _new_vert_location).norm();
		}

		// Compute acceleration with inverse distance weighting
		double _acceleration = 0.0;
		for (size_t _vert=0; _vert < _verts.size(); _vert++)
		{
			_acceleration +=  ((get_vertex(_verts.at(_vert)) - _new_vert_location).norm()/distance_sum)*
					this->m_curvatureData.acceleration.at(_verts.at(_vert));
		}

		return _acceleration;

	}

	/*
	 * the acceleration for the mesh should have already been calculated
	 */
	bool ACCLMesh::checkForBadEdges()
	{

		std::cout << "Refining Mesh Edges" << std::endl;
		std::set<size_t> visitedEdges;
		// splitHalfEdgeProper(20);
		std::vector<size_t> _edgesToSplit;
		std::vector<double> _new_vert_accelerations;
		size_t hes = this->m_heData.size();
		for ( size_t i = 0; i < hes; i++)
		{// Add vertices between good and bad vertices in the mesh
			if (visitedEdges.find(i) == visitedEdges.end())
			{ // not already visited

				visitedEdges.insert(i);
				visitedEdges.insert(this->m_heData[i].twin);

				if ( (m_heData[i].face == HOLE_INDEX) || (m_heData[m_heData[i].twin].face == HOLE_INDEX ))
				{ // ignore boundary edges
					// std::cout << "Skipping boundary edges" << std::endl;
					continue;
				}

				std::vector<size_t> _verts;
				_verts.push_back(this->m_heData[i].vert);
				_verts.push_back(next(next(this->m_heData[i])).vert);
				_verts.push_back(twin(this->m_heData[i]).vert);
				_verts.push_back(next(next(twin(this->m_heData[i]))).vert);
				// std::vector<double> _curvatures;
				// double _curvature_sum = 0;
				size_t verts_above_thresh = 0;
				for (size_t v=0; v< _verts.size(); v++)
				{
					if ( this->m_curvatureData.acceleration[_verts.at(v)] > this->m_curvatureData.max_acceleration_allowed )
					{
						verts_above_thresh++;
					}
					// _curvatures.push_back(this->mean_cotan_curvature(_verts.at(v)));
					// _curvature_sum += _curvatures.at(v);
				}
				/* Now
					v4
				   /  \
				  /    \
				 /      \
				v2------v3
				 \      /
				  \    /
				   \  /
				    v1
				    Might need to flip the edge if either
				    <v1, v2, v3> or <v2, v3, v4> are all above the threshold
				    Problem is symetric*
	*/
				if ( verts_above_thresh == 3 )
				{ // check that the those verts on not on the ends of this edge, if so flip edge
					// std::cout << "Found a potentially bad edge" << std::endl;
					if ( ( this->m_curvatureData.acceleration.at(this->m_heData[i].vert) >
					this->m_curvatureData.max_acceleration_allowed ) &&
						( this->m_curvatureData.acceleration.at(twin(this->m_heData[i]).vert) >
										this->m_curvatureData.max_acceleration_allowed ))
					{// flip edge
						// std::cout << "Found bad edge" << std::endl;
						// this->flip_edge(this->m_heData[i]);
						// check that is was not a bad flip
						_edgesToSplit.push_back(i);
						_new_vert_accelerations.push_back(this->accelerationForBadEdges(i));
						// Can't change topology while performing this check
						// splitHalfEdgeProper(i);
						/*
						double _tmp_curvature_sum = 0;
						for (size_t v=0; v< _curvatures.size(); v++)
						{
							// _curvatures.push_back(this->mean_cotan_curvature(_verts.at(v)));
							std::cout << "checking curvatures " << _curvatures.at(v) << " != " << this->mean_cotan_curvature(_verts.at(v)) << std::endl;

							_tmp_curvature_sum += this->mean_cotan_curvature(_verts.at(v));
							if ( _curvatures.at(v) != this->mean_cotan_curvature(_verts.at(v)))
							{
								std::cout << "Flipping edge back because it changed the curvature." << std::endl;
								// this->flip_edge(this->m_heData[i]);

								// break;
							}

						}
						*/
						// std::cout << "checking curvatures sums tmp, old " << _tmp_curvature_sum << ",  " << _curvature_sum << std::endl;
					}

				}
			}
		}

		for (size_t _edge=0; _edge < _edgesToSplit.size(); _edge++)
		{
			// std::cout << "Splitting edge: " << _edgesToSplit.at(_edge) << std::endl;
			size_t _new_vert = splitHalfEdgeProper(_edgesToSplit.at(_edge));
			this->m_curvatureData.acceleration.push_back(_new_vert_accelerations.at(_edge));
			// TODO kind of hacky way to make sure these new verts are in the ROI
			// this->selected_verts_neighbours.push_back(_new_vert);
			// this->verify();
		}

		this->edit_count++;
		return true;
	}




	/*
	 * Gets all of the obstacles as there boundaries, a list of verts.
	 */
	std::vector<std::vector<size_t> > ACCLMesh::getObstacles()
	{

		std::vector<std::vector<size_t> > obstacles;
		std::set<size_t> visited_verts;
		for (size_t _vert=0; _vert < this->get_vert_size(); _vert++)
		{
			if ( visited_verts.find(_vert) != visited_verts.end())
			{ // If this vert has already been visited skip it
				continue;
			}
			visited_verts.insert(_vert);

			if ( this->isBoundaryVertex(_vert))
			{ // Found a new boundary vert, Get the ring of verts connected to this
				std::vector<size_t> obs;
				obs.push_back(_vert);
				size_t last_vert = -1;
				std::vector<size_t> _vert_neighbours = this->get_vert_neighbours(_vert);
				for (size_t t_vert=0; t_vert < _vert_neighbours.size(); t_vert++ )
				{
					if ( this->isBoundaryVertex(_vert_neighbours.at(t_vert)) &&
							this->isBoundaryHalfEdge(this->find_edge(_vert, _vert_neighbours.at(t_vert)))
							)
					{
						obs.push_back(_vert_neighbours.at(t_vert));
						last_vert = _vert_neighbours.at(t_vert);
						visited_verts.insert(_vert_neighbours.at(t_vert));
						break;
					}
				}
				while ( last_vert != _vert)
				{// loop over the vert rings to find the next verts until looped back to original vert
					std::vector<size_t> t_vert_neighbours = this->get_vert_neighbours(last_vert);
					for (size_t t_vert=0; t_vert < _vert_neighbours.size(); t_vert++ )
					{
						if ( this->isBoundaryVertex(t_vert_neighbours.at(t_vert)) &&
								this->find_edge(last_vert, t_vert_neighbours.at(t_vert)) && // this can be null because verts can be the same
								( this->isBoundaryHalfEdge(this->find_edge(last_vert, t_vert_neighbours.at(t_vert))) ) &&
								(t_vert_neighbours.at(t_vert) != last_vert)  )
						{
							obs.push_back(t_vert_neighbours.at(t_vert));
							last_vert = t_vert_neighbours.at(t_vert);
							visited_verts.insert(t_vert_neighbours.at(t_vert));
							break;
						}
					}
				}
				obs.pop_back(); // stored the original _vert at the end
				obstacles.push_back(obs);
			}
		}

		return obstacles;
	}

	/**
		This my not work properly
	*/
	std::vector<size_t> ACCLMesh::get_faces_for_vert(size_t oldV) const
	{
		std::vector<size_t> vertFaces;
		size_t he = this->m_vertData[oldV];
		size_t heO = he;
		if ( this->m_heData[he].face != HOLE_INDEX)
		{
			vertFaces.push_back(this->m_heData[he].face);
		}
		he = twin(this->m_heData[he]).next;
		for ( size_t i = 0; he != heO; i++ )
		{
			if ( this->m_heData[he].face != HOLE_INDEX)
			{
				vertFaces.push_back(this->m_heData[he].face);
			}
			he = twin(this->m_heData[he]).next;
		}
		return vertFaces;
	}


	/*
	 1.0 > dest > 0.0
	*/
	void ACCLMesh::splitHalfEdge(size_t he, double dist)
	{ // splits the half edge along the he
		// The order of operations in this is very important so that pointers are not lost.
		// he.origin + (he*dist)
		size_t v = this->add_vertex( ( get_vertex(this->m_heData[he].vert) ) + ((get_vertex(twin(this->m_heData[he]).vert) - get_vertex(this->m_heData[he].vert)) * dist) );

		half_edge he_o = this->m_heData[he];
		half_edge he_t = twin(this->m_heData[he]);

		half_edge he1;

		he1.next = he_o.next;
		he1.face = he_o.face;
		he1.twin = he_o.twin;
		he1.vert = v;

		this->m_heData.push_back( he1 );
		this->m_heData[he].next = this->m_heData.size()-1;
		// TODO, I don't think this is getting populated properly
		this->m_vertData[v] = this->m_heData.size()-1;

		half_edge he2;
		he2.next = he_t.next;
		he2.face = he_t.face;
		he2.twin = he;
		he2.vert = v;

		this->m_heData.push_back( he2 );
		this->m_heData[this->m_heData[he].twin].next = this->m_heData.size()-1;

		this->m_heData[this->m_heData[he].twin].twin = this->m_heData[he].next;
		this->m_heData[he].twin = this->m_heData.size()-1;

	}


	/*
	 * The acceleration on the mesh should have already been calculated
	 */
	bool ACCLMesh::isBoundaryVertex(size_t vert)
	{

		if ( vert >= this->m_curvatureData.acceleration.size() )
		{
			return true;
		}
		return false;

	}

	/*
	 * Check to see if this edge is a boundary edge
	 * It is a boundary edge if the dge verts don't have acceleration and
	 * opposite verts one have accl above the threshold
	 * The other vert can have low accl or be a boundary vert
	 */
	bool ACCLMesh::isBoundaryHalfEdge(half_edge * _he)
	{
		// half_edge _he = this->m_heData[he];

		if ( (_he->vert >= this->m_curvatureData.acceleration.size()) &&
				(twin(*_he).vert >= this->m_curvatureData.acceleration.size()))
		{ // both verts should not have accelerations assigned
			if ( (next(next(*_he)).vert < this->m_curvatureData.acceleration.size()) &&
					(this->m_curvatureData.acceleration.at(next(next(*_he)).vert) >
					this->m_curvatureData.max_acceleration_allowed) )
			{
				return true;
			}
			if ( (next(next(twin(*_he))).vert < this->m_curvatureData.acceleration.size()) &&
					(this->m_curvatureData.acceleration.at(next(next(twin(*_he))).vert) >
					this->m_curvatureData.max_acceleration_allowed) )
			{
				return true;
			}
		}
		return false;
	}

	void ACCLMesh::initFromMesh(Mesh *mesh)
	{
		int editCount = this->get_edit_count();
		this->clear();
		// Helps make sure DrawMesh is updated
		this->edit_count = editCount;

		// add all the vertices
		this->m_faceData.insert(this->m_faceData.end(), mesh->m_faceData.begin(), mesh->m_faceData.end());
		this->m_heData.insert(this->m_heData.end(), mesh->m_heData.begin(), mesh->m_heData.end());
		// this->m_simplifyData.insert(this->m_simplifyData.end(), mesh->m_simplifyData.begin(), mesh->m_simplifyData.end());
		// this->m_simplifyQueue.insert(this->m_simplifyQueue.end(), mesh->m_simplifyQueue.begin(), mesh->m_simplifyQueue.end());
		this->m_vertData.insert(this->m_vertData.end(), mesh->m_vertData.begin(), mesh->m_vertData.end());
		this->m_vertices.insert(this->m_vertices.end(), mesh->m_vertices.begin(), mesh->m_vertices.end());
		/*
		this->_simplifyPriorityQueue.insert(mesh->_simplifyPriorityQueue.begin(), mesh->_simplifyPriorityQueue.end());

		this->selected_verts.insert(this->selected_verts.end(), mesh->selected_verts.begin(), mesh->selected_verts.end());
		this->selected_verts_neighbours.insert(this->selected_verts_neighbours.end(),
			mesh->selected_verts_neighbours.begin(), mesh->selected_verts_neighbours.end());
		this->LaplacianT = mesh->LaplacianT;
		this->anchor = mesh->anchor;
		 */
		// copy curvature
		ACCLMesh * amesh = dynamic_cast<ACCLMesh *>(mesh);
		if ( amesh != NULL)
		{
			this->m_curvatureData.curvature.insert(this->m_curvatureData.curvature.end(), amesh->m_curvatureData.curvature.begin(), amesh->m_curvatureData.curvature.end());
			this->m_curvatureData.acceleration.insert(this->m_curvatureData.acceleration.end(), amesh->m_curvatureData.acceleration.begin(), amesh->m_curvatureData.acceleration.end());
			this->m_curvatureData.average_curvature = amesh->m_curvatureData.average_curvature;
			this->m_curvatureData.max_acceleration_allowed = amesh->m_curvatureData.max_acceleration_allowed;
			this->m_curvatureData.max_curvature = amesh->m_curvatureData.max_curvature;
			this->m_curvatureData.min_curvature = amesh->m_curvatureData.min_curvature;
		}
		/*
		for (int i=0; i < mesh->get_vert_size(); i++)
		{
			this->add_vertex(mesh->get_vertex(i));
		}

		// construct all of the faces
		for (int i=0; i < mesh->get_face_size(); i++)
		{
			size_t neighbours[3];
			mesh->getIndicesForFace(i, neighbours);
			this->add_face(neighbours);
		}*/

	}

	/*
		First edge is the edge pointing into the vertex origin of the new halfedge
		Seond half-edge is the one pointing to the vertex as the end of the half-edge
		      /  \
	  hePrev /    \
			 ------
			/      \ heNextPrev
		   /        \
	*/
	void ACCLMesh::addEdge(size_t hePrev, size_t heNextPrev)
	{

		half_edge newEdge;
		newEdge.next = this->m_heData[heNextPrev].next;
		newEdge.vert = next(this->m_heData[hePrev]).vert;
		newEdge.face = this->m_heData[newEdge.next].face;

		half_edge newEdgeTwin;
		newEdgeTwin.next = this->m_heData[hePrev].next;
		newEdgeTwin.vert = next(this->m_heData[heNextPrev]).vert;
		newEdgeTwin.face = this->m_heData[newEdgeTwin.next].face;

		this->m_heData.push_back(newEdge);
		newEdgeTwin.twin = this->m_heData.size()-1;
		this->m_heData.push_back(newEdgeTwin);
		this->m_heData.at(this->m_heData.size() - 2).twin = this->m_heData.size()-1 ;

		this->m_heData[heNextPrev].next = this->m_heData.at(this->m_heData.size() - 2).twin;
		this->m_heData[hePrev].next = newEdgeTwin.twin;
	}

	void ACCLMesh::setFaceForHalfEdges(size_t face, std::vector<size_t> hes)
	{
		for (size_t i = 0; i < hes.size(); i++)
		{
			this->m_heData[hes[i]].face = face;
		}
	}


	void ACCLMesh::drawMesh()
	{// Draw mesh triangles

		// Draw triangles
		Util::Vector adjust = Util::Vector(0.0f,-0.4f,0.0f);
		if ( this->drawState == 0 )
		{
			glBegin(GL_TRIANGLES);
			{
				glColor4f(0.2,1.0,1.0, 0.7);
				for (size_t face=0; face < this->get_face_size(); face++ )
				{
					Util::Point p1;
					Util::Vector norm(0.0,1.0,0.0);
					half_edge he = this->get_he_for_face(face);
					p1 = this->get_vertex(he.vert);
					// TODO having issue with bad vert numbers, most likely for meshes with lots of holes.
					// norm = this->get_vnormal(he.vert);
					double acceleration_i = this->m_curvatureData.acceleration.at(he.vert);
					if ( acceleration_i > this->m_curvatureData.max_acceleration_allowed )
					{
						glColor4f(0.0,0.0,0.0, 0.7);
					}
					else
					{
						glColor4f(0.2,(1.0-(acceleration_i / this->m_curvatureData.max_acceleration_allowed)),1.0, 0.7);
					}
					glNormal3f(norm.x, norm.y, norm.z);
					glVertex3f(p1.x,p1.y,p1.z);

					he = next(he);
					p1 = this->get_vertex(he.vert);
					// norm = this->get_vnormal(he.vert);
					acceleration_i = this->m_curvatureData.acceleration.at(he.vert);
					if ( acceleration_i > this->m_curvatureData.max_acceleration_allowed )
					{
						glColor4f(0.0,0.0,0.0, 0.7);
					}
					else
					{
						glColor4f(0.2,(1.0-(acceleration_i / this->m_curvatureData.max_acceleration_allowed)),1.0, 0.7);
					}
					glNormal3f(norm.x, norm.y, norm.z);
					glVertex3f(p1.x,p1.y,p1.z);

					he = next(he);
					p1 = this->get_vertex(he.vert);
					// norm = this->get_vnormal(he.vert);
					acceleration_i = this->m_curvatureData.acceleration.at(he.vert);
					if ( acceleration_i > this->m_curvatureData.max_acceleration_allowed )
					{
						glColor4f(0.0,0.0,0.0, 0.7);
					}
					else
					{
						glColor4f(0.2,(1.0-(acceleration_i / this->m_curvatureData.max_acceleration_allowed)),1.0, 0.7);
					}
					glNormal3f(norm.x, norm.y, norm.z);
					glVertex3f(p1.x,p1.y,p1.z);
				}
			}
		}
		else
		{
			glBegin(GL_TRIANGLES);
			{
				for (size_t face=0; face < this->get_face_size(); face++ )
				{
					if ( (( this->get_he_for_face(face).vert < this->m_curvatureData.acceleration.size()) &&
									this->m_curvatureData.acceleration.at(this->get_he_for_face(face).vert) >
									this->m_curvatureData.max_acceleration_allowed) ||
						 (( next(this->get_he_for_face(face)).vert < this->m_curvatureData.acceleration.size()) &&
									this->m_curvatureData.acceleration.at(next(this->get_he_for_face(face)).vert) >
									this->m_curvatureData.max_acceleration_allowed) ||
						 (( next(next(this->get_he_for_face(face))).vert < this->m_curvatureData.acceleration.size()) &&
									this->m_curvatureData.acceleration.at(next(next(this->get_he_for_face(face))).vert) >
									this->m_curvatureData.max_acceleration_allowed) )
					{
						glColor4f(0.0,0.0,0.0, 0.7);
					}
					else
					{
						glColor4f(0.2,1.0,1.0, 0.7);
					}
					Util::Point p1;
					Util::Vector norm(0.0,1.0,0.0);
					half_edge he = this->get_he_for_face(face);
					p1 = this->get_vertex(he.vert);
					// TODO having issue with bad vert numbers, most likely for meshes with lots of holes.
					// norm = this->get_vnormal(he.vert);
					glNormal3f(norm.x, norm.y, norm.z);
					glVertex3f(p1.x,p1.y-0.4f,p1.z);

					he = next(he);
					p1 = this->get_vertex(he.vert);
					// norm = this->get_vnormal(he.vert);
					glNormal3f(norm.x, norm.y, norm.z);
					glVertex3f(p1.x,p1.y-0.4f,p1.z);

					he = next(he);
					p1 = this->get_vertex(he.vert);
					// norm = this->get_vnormal(he.vert);
					glNormal3f(norm.x, norm.y, norm.z);
					glVertex3f(p1.x,p1.y-0.4f,p1.z);
				}
			}
		}
		glEnd();
		// Draw half edges, thicker edges for boundaries

		for (size_t he_id=0; he_id < m_heData.size(); he_id++)
		{
			half_edge he = m_heData.at(he_id);
			if ((he.face == HOLE_INDEX) || (twin(he).face == HOLE_INDEX))
			{
				Util::DrawLib::drawLine(this->get_vertex(he.vert)+adjust, this->get_vertex(next(he).vert)+adjust, Util::gDarkBlue, 1.0);
			}
			else if ((( he.vert < this->m_curvatureData.acceleration.size()) &&
						this->m_curvatureData.acceleration.at(he.vert) >
						this->m_curvatureData.max_acceleration_allowed) ||
					(( twin(he).vert < this->m_curvatureData.acceleration.size()) &&
						this->m_curvatureData.acceleration.at(twin(he).vert) >
						this->m_curvatureData.max_acceleration_allowed) )
			{ // TODO
				Util::DrawLib::drawLine(this->get_vertex(he.vert)+adjust, this->get_vertex(next(he).vert)+adjust, Util::gGray40, 0.8);
			}
			else
			{
				Util::DrawLib::drawLine(this->get_vertex(he.vert)+adjust, this->get_vertex(next(he).vert)+adjust, Util::gGray20, 0.8);
			}
		}
	}


} /* namespace ACCLMesh */


