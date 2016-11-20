// Author: MahyarKoy @Jan2016

#ifndef _GRID_PLAN_H_
#define _GRID_PLAN_H_

#ifdef _WIN32
#define SPACESYNTAX_API __declspec(dllexport)
#else
#define SPACESYNTAX_API
#endif

#include <vector>
#include <iostream>
// #include "SteerLib.h"
#include "util/Geometry.h"
#include <Eigen3.2.7/Dense>

#ifdef _WIN32
// on win32, there is an unfortunate conflict between exporting symbols for a
// dynamic/shared library and STL code.  A good document describing the problem
// in detail is http://www.unknownroad.com/rtfm/VisualStudio/warningC4251.html
// the "least evil" solution is just to simply ignore this warning.
#pragma warning( push )
#pragma warning( disable : 4251 )
#endif

namespace SpaceSyntax
{
	enum SPACESYNTAX_API ObstacleType : unsigned int
	{
		SHADOW = 0,
		BLOCK = 1,
	};

	class SPACESYNTAX_API Obstacle
	{
	public:
		Obstacle(Util::Point p1, Util::Point p2, float tiltDegree, ObstacleType type)
			: _pMin(p1), _pMax(p2), _type(type), _tiltDegree(tiltDegree)
		{}
		Obstacle(Util::Point topLeft, Util::Point botLeft, Util::Point botRight, Util::Point topRight, float heightMin = 0, float heightMax = 2, ObstacleType type = ObstacleType::BLOCK)
		{
			_pMin = botLeft;
			_pMax = topRight;
			_tiltDegree = (topLeft.z - botLeft.z) / (topLeft.x - botLeft.x);
			_tiltDegree = atanf(_tiltDegree);
			_type = type;
			_pMin.y = heightMin;
			_pMax.y = heightMax;
		}
		~Obstacle() {}
		Util::Point get_pMin() { return _pMin; }
		Util::Point get_pMax() { return _pMax; }
		ObstacleType get_Type() { return _type; }
		float get_tilt() { return _tiltDegree; }
		void operator=(Obstacle obstacle) // TODO removed the reference here.
		{
			this->_pMax = obstacle.get_pMax();
			this->_pMin = obstacle.get_pMin();
			this->_type = obstacle.get_Type();
			this->_tiltDegree = obstacle.get_tilt();
		}

	private:
		Util::Point _pMin, _pMax;
		ObstacleType _type;
		float _tiltDegree;
	};

	class SPACESYNTAX_API GridPlan
	{
	public:
		GridPlan(float x1, float z1, float x2, float z2, float height, unsigned int gridNumX, unsigned int gridNumZ);
		~GridPlan() {}
		
		//grid modification
		void set_gridNumX(unsigned int gridNumX) { _xNum = gridNumX;  _xSize = (_xMax - _xMin) / _xNum; }
		void set_gridNumZ(unsigned int gridNumZ) { _zNum = gridNumZ;  _zSize = (_zMax - _zMin) / _zNum; }
		void set_gridSize(float x1, float z1, float x2, float z2);
		void set_height(float height) { _height = height; }

		// grid access handling
		unsigned int get_gridNumX() { return _xNum; }
		unsigned int get_gridNumZ() { return _zNum; }
		unsigned int get_gridSizeX() { return _xSize; }
		unsigned int get_gridSizeZ() { return _zSize; }
		Util::Point get_gridMinPoint() { return Util::Point(_xMin, _height, _zMin); }
		Util::Point get_gridMaxPoint() { return Util::Point(_xMax, _height, _zMax); }
		unsigned int check_cell(unsigned int x, unsigned int z);
		Util::Point get_location(unsigned int x, unsigned int z);
		ObstacleType ray_cast(Util::Point p1, Util::Point p2);
		void get_gridLocation(Util::Point point, unsigned int& x, unsigned int& z);
		bool ray_cast_v2(float& px1r, float& pz1r, float& px2r, float& pz2r, float x1r, float z1r, float x2r, float z2r);
		bool hasLineOfSight(Util::Point p1, Util::Point p2);
		unsigned int get_gridID(unsigned int gx, unsigned int gz) { return gz*_xNum + gx; }
		
		// grid obstacle handling
		void add_obstacle(Obstacle obstacle) { _obstacleList.push_back(obstacle); }
		void add_obstacles(std::vector<Obstacle> obstacleList);
		void modify_obstacle(unsigned index, Obstacle obstacle);
		void remove_obstacle(unsigned index) { _obstacleList.erase(_obstacleList.begin() + index); }
		void clear_obstacles() { _obstacleList.clear(); }
		std::vector<Obstacle> get_obstacles() { return _obstacleList; }
		void rotate_axis(float& x1r, float& z1r, float tilt, float x1, float z1);
		std::vector<Obstacle> _obstacleList;

	private:
		float solve_line_z(float xTarget, float x1, float z1, float tangent) { return tangent*(xTarget - x1) + z1; }
		float solve_line_x(float zTarget, float x1, float z1, float tangent) { return (zTarget - z1) / tangent + x1; }
		float _xMin, _zMin, _xMax, _zMax, _height;
		unsigned int _xNum, _zNum;
		float _xSize, _zSize;
		Util::Point _gridOrigin;
	};

}

#endif
