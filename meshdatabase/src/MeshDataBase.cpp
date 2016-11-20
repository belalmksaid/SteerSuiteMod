//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//
/*
 * MeshDataBase.cpp
 *
 *  Created on: 2014-11-27
 *      Author: gberseth
 */

#include "MeshDataBase.h"
#include "util/DrawLib.h"

using namespace SteerLib;

#include "Mesh.h"
#include "obstacles/BoxObstacle.h"

#include "SteerLib.h"
#include "util/DrawLib.h"

#include <vector>

#include "MeshDataBaseModule.h"


using namespace SteerLib;
using namespace Util;
using namespace MeshDataBaseGlobals;

void MeshDataBase::init()
{
 // Does nothing
}

/*
 * Because this is on a mesh maybe this trace should be perpendicular to the
 * triangle the up vector.
 */
bool MeshDataBase::trace(const Util::Ray & r, float & t, SpatialDatabaseItemPtr &hitObject, SpatialDatabaseItemPtr exclude, bool excludeAgents)
{
	// return false;
	return _kdTreeDatabase->trace(r, t, hitObject, exclude, excludeAgents);


	// TODO make this not the slowest thing ever...
/*
	Util::Vector n = this->getUpVector(exclude);
	Util::Vector direction = r.dir - ((dot(r.dir, n)/n.lengthSquared())*n);
	// direction.y = (direction.y + r.dir.y) / 2.0f;
	// direction = direction * r.dir.length();
	for (size_t face=0; face < this->_mesh->get_face_size(); face++)
	{
		if ( this->_mesh->ray_intersect_triangle(r.pos, r.pos+(r.dir*r.maxt), face) )
		{
			return true;
		}
	}
	return false;
*/
}

std::vector<size_t> MeshDataBase::getFacesWithin(double dist)
{
	std::vector<size_t> faces;
	return faces;
}

MeshDataBase::MeshDataBase(ACCLMesh::ACCLMesh * mesh, EngineInterface * gEngine)
{
	this->setMesh(mesh);
	this->_face_occupency.resize(this->_mesh->get_face_size());
	// TODO this need to be done somewhere better, and properly
	// this->_agent_face_map.resize(gEngine->getAgents().size());
	this->_agent_face_map.resize(1100);
	_obstacels = mesh->getObstacles();
	this->_kdTreeDatabase = new KdTreeDataBase(gEngine, -100, 100, -100, 100);
	std::cout << "There are " << _obstacels.size() << " obstacles in the mesh" << std::endl;
}

void MeshDataBase::getItemsInRange(std::set<SpatialDatabaseItemPtr> & neighborList, float xmin, float xmax, float zmin, float zmax, SpatialDatabaseItemPtr exclude)
{
	_kdTreeDatabase->getItemsInRange(neighborList, xmin, xmax, zmin, zmax, exclude);
}

void MeshDataBase::buildObstacleTree()
{
	_kdTreeDatabase->buildObstacleTree();
}

void MeshDataBase::buildAgentTree()
{
	_kdTreeDatabase->buildAgentTree();
}


void MeshDataBase::draw()
{
	// this->_mesh->drawMesh();
	// draw the triangles

	// Draw triangle occupancy
	glBegin(GL_TRIANGLES);
	{
		/*
		glColor4f(0.1,1.0,0.1, 0.7);
		for (size_t face=0; face < this->_face_occupency.size(); face++ )
		{
			if ( this->_face_occupency.at(face).size() > 0)
			{
				Util::Point p1;
				p1 = _mesh->get_vertex(_mesh->get_he_for_face(face).vert);
				glVertex3f(p1.x,p1.y,p1.z);
				p1 = _mesh->get_vertex(_mesh->next(_mesh->get_he_for_face(face)).vert);
				glVertex3f(p1.x,p1.y,p1.z);
				p1 = _mesh->get_vertex(_mesh->next(_mesh->next(_mesh->get_he_for_face(face))).vert);
				glVertex3f(p1.x,p1.y,p1.z);
			}
		}
		*/
	}
	glEnd();

	Util::Vector adjust = Util::Vector(0.0f,-0.4f,0.0f);
	for (size_t obs=0; obs < this->_obstacels.size(); obs++)
	{
		for (size_t _vert=0; _vert < this->_obstacels.at(obs).size()-1; _vert++)
		{
			Util::DrawLib::drawLine(_mesh->get_vertex(this->_obstacels.at(obs).at(_vert))+adjust,
					this->_mesh->get_vertex(this->_obstacels.at(obs).at(_vert+1))+adjust, Util::gDarkMagenta, 2.0);
		}
		Util::DrawLib::drawLine(_mesh->get_vertex(this->_obstacels.at(obs).at(0))+adjust,
				this->_mesh->get_vertex(this->_obstacels.at(obs).at(this->_obstacels.at(obs).size()-1))+adjust, Util::gDarkMagenta, 2.0);
	}


	/*
	glBegin(GL_TRIANGLES);
	{
		glColor4f(0.2,1.0,1.0, 0.7);
		for (size_t face=0; face < _mesh->get_face_size(); face++ )
		{
			Util::Point p1,p2,p3;
			p1 = _mesh->get_vertex(_mesh->get_he_for_face(face).vert);
			glVertex3f(p1.x,p1.y,p1.z);
			p1 = _mesh->get_vertex(next(_mesh->get_he_for_face(face)).vert);
			glVertex3f(p1.x,p1.y,p1.z);
			p1 = _mesh->get_vertex(next(next(_mesh->get_he_for_face(face))).vert);
			glVertex3f(p1.x,p1.y,p1.z);
		}
	}
	glEnd();
	*/
}

Util::Point MeshDataBase::randomPositionWithoutCollisions(float radius, bool excludeAgents)
{

	srand (static_cast <unsigned> (time(0)));

	bool found_hit = false;
	float rx = 0;
	float rz = 0;
	float dist = 0;
	float r = 0;
	float r2 = 0;

	AxisAlignedBox region(-100, 100 , 0, 1, -100, 100 );

	r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	rx = (r * (region.xmax - region.xmin)) + region.xmin;

	r2 = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	rz = (r2 * (region.zmax - region.zmin)) + region.zmin;

	Util::Point out = Util::Point(rx,0,rz);
	std::cout << "Found random position " << out << std::endl;
	return out;
	// return Util::Point(0,0,0);

	// return this->_kdTreeDatabase->randomPositionWithoutCollisions(radius, excludeAgents);
}
/// Finds a random 2D point, within the specified region, that has no other objects within the requested radius.
Util::Point MeshDataBase::randomPositionInRegionWithoutCollisions(const Util::AxisAlignedBox & region, float radius, bool excludeAgents)
{
	return Util::Point(0,0,0);
}

/// Finds a random 2D point, within the specified region, that has no other objects within the requested radius, using an exising (already seeded) Mersenne Twister random number generator.
Util::Point MeshDataBase::randomPositionInRegionWithoutCollisions(const Util::AxisAlignedBox & region, float radius, bool excludeAgents, MTRand & randomNumberGenerator)
{
	return Util::Point(0,0,0);
}

/// Finds a random 2D point, within the specified region, that has no other objects within the requested radius, using an exising (already seeded) Mersenne Twister random number generator.
bool MeshDataBase::randomPositionInRegionWithoutCollisions(const Util::AxisAlignedBox & region, SpatialDatabaseItemPtr item, bool excludeAgents, MTRand & randomNumberGenerator)
{
	return true;
}


/// Finds a random 2D point, within the specified region, using an exising (already seeded) Mersenne Twister random number generator.
Util::Point MeshDataBase::randomPositionInRegion(const Util::AxisAlignedBox & region, float radius,MTRand & randomNumberGenerator)
{
	return Util::Point(0,0,0);
}



void MeshDataBase::computeAgentNeighbors(SpatialDatabaseItemPtr agent, float rangeSq) const
{
	_kdTreeDatabase->computeAgentNeighbors(agent, rangeSq);
}
void MeshDataBase::computeObstacleNeighbors(SpatialDatabaseItemPtr agent, float rangeSq) const
{
	// std::cout << "Computing obstacle neighbours in MeshDataBase" << std::endl;
	_kdTreeDatabase->computeObstacleNeighbors(agent, rangeSq);
}

/// Gets the location of this agent, really used to get the y-location
Util::Point MeshDataBase::getLocation(SpatialDatabaseItemPtr item)
{ // agent->id() < this->_agent_face_map.size()
	if ( item->isAgent() )
	{
		AgentInterface * agent = dynamic_cast<AgentInterface *>(item);
		size_t face = this->_agent_face_map[agent->id()];
		std::vector<Util::Point> _points;
		_points.push_back(_mesh->get_vertex(_mesh->m_heData[_mesh->m_faceData[face]].vert));
		_points.push_back(_mesh->get_vertex(_mesh->next(_mesh->m_heData[_mesh->m_faceData[face]]).vert));
		_points.push_back(_mesh->get_vertex(_mesh->next(_mesh->next(_mesh->m_heData[_mesh->m_faceData[face]])).vert));

		Util::Vector _weights = convertToBarycentric(_points, agent->position());
		// return this->_mesh->getFaceMidpoint(face);
		return (_points.at(0) * _weights.x) + (_points.at(1) * _weights.y) + (_points.at(2) * _weights.z);

	}
	/// Otherwise item was an obstacle
	return Util::Point (0.0,1.0,0.0);
}
/// Get Normal for item, This get the normal for an item, used to determine the orientation of the item
Util::Vector MeshDataBase::getUpVector(SpatialDatabaseItemPtr exclude1)
{
	return Util::Vector (0.0,1.0,0.0);
}

void MeshDataBase::addObject( SpatialDatabaseItemPtr item,
		const Util::AxisAlignedBox & newBounds )
{
	// Find triangle this is inside
	_kdTreeDatabase->addObject(item, newBounds);

	if ( item->isAgent() )
	{
		AgentInterface * agent = dynamic_cast<AgentInterface *>(item);
		long face = -1;
		size_t vert = this->_mesh->closestVert(agent->position());
		vvert_iterator it;
		this->_mesh->init_iterator( it, vert );

		do{
			face = it.m_cur->face;
			if ( this->_mesh->ray_intersect_triangle(agent->position()-this->getUpVector(agent), agent->position()+this->getUpVector(agent), face) )
			{
				this->_face_occupency.at(face).insert(agent->id());
				this->_agent_face_map[agent->id()] = face;
				return;
			}
		}while( this->_mesh->advance_iterator( it ) );
		// std::cout << "Number of faces: " << this->_mesh->get_face_size() << std::endl;
		// std::cout << "Adding agent->id() to agent face map: " << agent->id() << " = " << face << std::endl;

	}
	// add this item into that triangle
}
/// Removes an object from the database.  <b>It is the user's responsibility to make sure oldBounds is correct.</b>
void MeshDataBase::removeObject( SpatialDatabaseItemPtr item,
		const Util::AxisAlignedBox &oldBounds )
{
	_kdTreeDatabase->removeObject(item, oldBounds);
	if ( item->isAgent() )
	{
		AgentInterface * agent = dynamic_cast<AgentInterface *>(item);
		size_t oldface = this->_agent_face_map[agent->id()];
		for (std::set<size_t>::iterator a_it = this->_face_occupency.at(oldface).begin();
				a_it != this->_face_occupency.at(oldface).end(); a_it++)
		{
			if ( *a_it == agent->id() )
			{
				this->_face_occupency.at(oldface).erase(a_it);
				this->_agent_face_map[agent->id()] = 0; // TODO this is not right, can't use size_t
				break;
			}

		}
	}

}

Util::Point MeshDataBase::closestVert(Util::Point pos)
{
	size_t vert = this->_mesh->closestVert(pos);
	return this->_mesh->get_vertex(vert);
}
/// Updates an existing object in the database.  <b>It is the user's responsibility to make sure oldBounds is correct.</b>
/*
 * There is the chance that an agent goes past a few triangles
 */
void MeshDataBase::updateObject( SpatialDatabaseItemPtr item,
		const Util::AxisAlignedBox & oldBounds,
		const Util::AxisAlignedBox & newBounds )
{
	// Find triangle this is inside
	// agent should already be in database
	_kdTreeDatabase->updateObject(item, oldBounds, newBounds);
	if ( item->isAgent() )
	{
		AgentInterface * agent = dynamic_cast<AgentInterface *>(item);
		size_t oldface = this->_agent_face_map[agent->id()];
		if ( this->_mesh->ray_intersect_triangle(agent->position()-this->getUpVector(agent), agent->position()+this->getUpVector(agent), oldface) )
		{ // still in same triangle
			return;
		}
		else
		{ // In some Neighbour Triangle
			long face = -1;
			size_t vert = this->_mesh->closestVert(agent->position());
			fface_iterator it;
			this->_mesh->init_iterator( it, oldface );

			do{
				long tmp_face = this->_mesh->deref_iterator(it);
				if ( (tmp_face != HOLE_INDEX) && this->_mesh->ray_intersect_triangle(agent->position()-this->getUpVector(agent), agent->position()+this->getUpVector(agent), tmp_face) )
				{
					// remove old associations
					// add new
					// this->_agent_face_map[agent->id()] = face;
					face = tmp_face;
					break;
				}
			}while( this->_mesh->advance_iterator( it ) );


			if ( face != -1 )
			{
				for (std::set<size_t>::iterator a_it = this->_face_occupency.at(oldface).begin();
						a_it != this->_face_occupency.at(oldface).end(); a_it++)
				{
					if ( *a_it == agent->id() )
					{
						this->_face_occupency.at(oldface).erase(a_it);
						this->_agent_face_map[agent->id()] = face;
						break;
					}

				}
				this->_face_occupency.at(face).insert(agent->id());
			}
			else
			{ // agent must have skipped past a few tris
				this->removeObject(agent, oldBounds);
				this->addObject(agent, newBounds);
			}
		}

		if ( !this->_mesh->ray_intersect_triangle(agent->position()-this->getUpVector(agent), agent->position()+this->getUpVector(agent), this->_agent_face_map[agent->id()]) )
		{
			std::cout << "Agent is not in the tri it is supposed to be in " << std::endl;

		}
		// std::cout << "Number of faces: " << this->_mesh->get_face_size() << std::endl;
		// std::cout << "Adding agent->id() to agent face map: " << agent->id() << " = " << face << std::endl;

	}
	// update this item into that triangle
}
