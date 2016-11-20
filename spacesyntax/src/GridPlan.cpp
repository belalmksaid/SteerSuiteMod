// Author: MahyarKoy @Jan2016

#include "GridPlan.h"
#include "SteerLib.h"
#include <vector>
#include <iostream>
#include <cmath>

using namespace SpaceSyntax;
using namespace std;

GridPlan::GridPlan(float x1, float z1, float x2, float z2, float height, unsigned int gridNumX, unsigned int gridNumZ)
{
	_xNum = gridNumX;
	_zNum = gridNumZ;
	_height = height;
	set_gridSize(x1, z1, x2, z2);
}

void GridPlan::set_gridSize(float x1, float z1, float x2, float z2)
{
	_xMin = x1;
	_zMin = z1;
	_xMax = x2;
	_zMax = z2;
	if (_xMin > _xMax)
	{
		float tmp = _xMin;
		_xMin = _xMax;
		_xMax = tmp;
	}
	if (_zMin > _zMax)
	{
		float tmp = _zMin;
		_zMin = _zMax;
		_zMax = tmp;
	}

	_xSize = (_xMax - _xMin) / _xNum;
	_zSize = (_zMax - _zMin) / _zNum;
}

Util::Point GridPlan::get_location(unsigned int x, unsigned int z)
{
	Util::Point result;
	result.x = _xMin + (x+0.5)*_xSize;
	result.y = _height;
	result.z = _zMin + (z+0.5)*_zSize;
	return result;
}

void GridPlan::get_gridLocation(Util::Point point, unsigned int& x, unsigned int& z)
{
	x = (point.x - _xMin) / _xSize;
	z = (point.z - _zMin) / _zSize;
}

unsigned int GridPlan::check_cell(unsigned int x, unsigned int z)
{
	Util::Point point;
	point = get_location(x, z);
	for (auto itr = _obstacleList.begin(); itr != _obstacleList.end(); itr++)
	{
		if (itr->get_pMin().x <= point.x && itr->get_pMin().z <= point.z && itr->get_pMin().y <= point.y &&
			itr->get_pMax().x >= point.x && itr->get_pMax().z >= point.z && itr->get_pMax().y >= point.y &&
			itr->get_Type() != ObstacleType::SHADOW)
			return ObstacleType::BLOCK;
	}
	return ObstacleType::SHADOW;
}

ObstacleType GridPlan::ray_cast(Util::Point p1, Util::Point p2)
{
	float xT = -1.5, zT = -4.5, zT2 = -3.5;
	for (auto itr = _obstacleList.begin(); itr != _obstacleList.end(); itr++)
	{
		float x1, z1, x2, z2, target;
		if (itr->get_Type() == ObstacleType::SHADOW)
			continue;
		x1 = itr->get_pMin().x;
		z1 = itr->get_pMin().z;
		x2 = itr->get_pMax().x;
		z2 = itr->get_pMax().z;

		// Do axis rotation to get axis aligned obstacle
		float tilt, x1r, z1r, x2r, z2r, px1r, pz1r, px2r, pz2r, tangent;
		tilt = itr->get_tilt();
		rotate_axis(x1r, z1r, tilt, x1, z1);
		rotate_axis(x2r, z2r, tilt, x2, z2);
		rotate_axis(px1r, pz1r, tilt, p1.x, p1.z);
		rotate_axis(px2r, pz2r, tilt, p2.x, p2.z);

		tangent = (pz2r - pz1r) / (px2r - px1r);
		float xpMin = px1r, zpMin = pz1r, xpMax = px2r, zpMax = pz2r;
		if (xpMin > xpMax)
		{
			float tmp = xpMin;
			xpMin = xpMax;
			xpMax = tmp;
		}
		if (zpMin > zpMax)
		{
			float tmp = zpMin;
			zpMin = zpMax;
			zpMax = tmp;
		}
		if (x1r > x2r)
		{
			float tmp = x1r;
			x1r = x2r;
			x2r = tmp;
		}
		if (z1r > z2r)
		{
			float tmp = z1r;
			z1r = z2r;
			z2r = tmp;
		}

		// check if the obstacle is within points
		if (xpMin > x2r || xpMax < x1r || zpMin > z2r || zpMax < z1r)
			continue;

		// check if any of the points are inside the obstacle
		if (px1r >= x1r && px1r <= x2r && pz1r >= z1r && pz1r <= z2r)
			return itr->get_Type();
		if (px2r >= x1r && px2r <= x2r && pz2r >= z1r && pz2r <= z2r)
			return itr->get_Type();

		// check for intersection: 3 checks is enough, the 4th one is extra (kept for special use)
		if (x1r >= xpMin && x1r <= xpMax)
		{
			target = solve_line_z(x1r, px1r, pz1r, tangent);
			if (target >= z1r && target <= z2r)
				if (target >= zpMin && target <= zpMax)
					return itr->get_Type();
		}

		if (x2r >= xpMin && x2r <= xpMax)
		{
			target = solve_line_z(x2r, px1r, pz1r, tangent);
			if (target >= z1r && target <= z2r)
				if (target >= zpMin && target <= zpMax)
					return itr->get_Type();
		}

		if (z1r >= zpMin && z1r <= zpMax)
		{
			target = solve_line_x(z1r, px1r, pz1r, tangent);
			if (target >= x1r && target <= x2r)
				if (target >= xpMin && target <= xpMax)
					return itr->get_Type();
		}
	}
	return ObstacleType::SHADOW;
}

bool GridPlan::ray_cast_v2(float& px1r, float& pz1r, float& px2r, float& pz2r, float x1r, float z1r, float x2r, float z2r)
{
	float tangent, target;
		tangent = (pz2r - pz1r) / (px2r - px1r);
		float xpMin = px1r, zpMin = pz1r, xpMax = px2r, zpMax = pz2r;
		if (xpMin > xpMax)
		{
			float tmp = xpMin;
			xpMin = xpMax;
			xpMax = tmp;
		}
		if (zpMin > zpMax)
		{
			float tmp = zpMin;
			zpMin = zpMax;
			zpMax = tmp;
		}
		if (x1r > x2r)
		{
			float tmp = x1r;
			x1r = x2r;
			x2r = tmp;
		}
		if (z1r > z2r)
		{
			float tmp = z1r;
			z1r = z2r;
			z2r = tmp;
		}

		// check if the obstacle is within points
		if (xpMin > x2r || xpMax < x1r || zpMin > z2r || zpMax < z1r)
			return true;

		// check if any of the points are inside the obstacle
		if (px1r >= x1r && px1r <= x2r && pz1r >= z1r && pz1r <= z2r)
			return false;
		if (px2r >= x1r && px2r <= x2r && pz2r >= z1r && pz2r <= z2r)
			return false;

		// check for intersection: 3 checks is enough
		if (x1r >= xpMin && x1r <= xpMax)
		{
			target = solve_line_z(x1r, px1r, pz1r, tangent);
			if (target >= z1r && target <= z2r)
				if (target >= zpMin && target <= zpMax)
					return false;
		}

		if (x2r >= xpMin && x2r <= xpMax)
		{
			target = solve_line_z(x2r, px1r, pz1r, tangent);
			if (target >= z1r && target <= z2r)
				if (target >= zpMin && target <= zpMax)
					return false;
		}

		if (z1r >= zpMin && z1r <= zpMax)
		{
			target = solve_line_x(z1r, px1r, pz1r, tangent);
			if (target >= x1r && target <= x2r)
				if (target >= xpMin && target <= xpMax)
					return false;
		}
	return true;
}

bool GridPlan::hasLineOfSight(Util::Point p1, Util::Point p2)
{
	if (ray_cast(p1, p2) == ObstacleType::BLOCK)
		return false;
	else
		return true;
}

void GridPlan::add_obstacles(std::vector<Obstacle> obstacleList)
{
	for (auto itr = obstacleList.begin(); itr != obstacleList.end(); itr++)
	{
		_obstacleList.push_back(*itr);
	}
}

void GridPlan::modify_obstacle(unsigned index, Obstacle obstacle)
{
	_obstacleList[index] = obstacle;
}

void GridPlan::rotate_axis(float& x1r, float& z1r, float tilt, float x1, float z1)
{
	float sinVal, cosVal;
	sinVal = sinf(tilt);
	cosVal = cosf(tilt);
	z1r = cosVal*z1 - sinVal*x1;
	x1r = sinVal*z1 + cosVal*x1;
}