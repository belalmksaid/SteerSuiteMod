/*
 * SimWorld.cpp
 *
 *  Created on: 2015-10-27
 *      Author: gberseth
 */

#include "SimWorld.h"
#include "obstacles/BoxObstacle.h"
#include "obstacles/OrientedBoxObstacle.h"
#include "obstacles/PolygonObstacle.h"

namespace SteerSuite {


SimWorld::SimWorld() {
	// TODO Auto-generated constructor stub
}

SimWorld::~SimWorld() {
	// TODO Auto-generated destructor stub
}

void SimWorld::addObstacle(double xmin,double  xmax,double  ymin,double  ymax,double  zmin,double  zmax)
{
	// std::cout << "SimWorld adding Obstacle: " << xmin << ", " << xmax << ", " << ymin <<
		// ", " << ymax << ", " << zmin << ", " << zmax << std::endl;
		SteerLib::BoxObstacle * box = new SteerLib::BoxObstacle(xmin, xmax, ymin,	ymax, zmin, zmax);
		this->_obstacles.push_back(box);
}

void SimWorld::addOrientedObstacle(Util::Point centerPosition, float lengthX, float lengthZ, float ymin, float ymax, float thetaY)
{
	// std::cout << "SimWorld adding OrientedObstacle: " << centerPosition << " theta: " << thetaY << std::endl;
	SteerLib::OrientedBoxObstacle * box = new SteerLib::OrientedBoxObstacle(centerPosition, lengthX, lengthZ,	ymin, ymax, thetaY);
	/*
	std::vector<Util::Point> verts = box->get2DStaticGeometry();
	// http://doc.cgal.org/latest/Minkowski_sum_2/
	std::vector<Util::Point> minskSum;
	std::vector<Util::Point> minskSumOut;
	double radius = 1.0;
	size_t i = 0;
	for (; i < (verts.size()-1); i++)
	{
		Util::Vector edge = verts[i + 1] - verts[i];
		Util::Vector rightNormal = normalize(rotateInXZPlane(edge, -M_PI_OVER_2));
		minskSum.push_back(verts[i] + (rightNormal * radius));
		minskSum.push_back(verts[i+1] + (rightNormal * radius));
	}
	Util::Vector edge = verts[0] - verts[i];
	Util::Vector rightNormal = normalize(rotateInXZPlane(edge, -M_PI_OVER_2));
	minskSum.push_back(verts[i] + (rightNormal * radius));
	minskSum.push_back(verts[0] + (rightNormal * radius));

	i = 0;
	for (; i < (minskSum.size() - 2); i+=2)
	{
		Util::Vector edge1 = minskSum[i + 1] - minskSum[i];
		Util::Vector edge2 = minskSum[i + 3] - minskSum[i+2];
		Util::Vector rightNormal1 = normalize(rotateInXZPlane(edge1, -M_PI_OVER_2));
		Util::Vector rightNormal2 = normalize(rotateInXZPlane(edge2, -M_PI_OVER_2));
		minskSumOut.push_back(minskSum[i] );
		minskSumOut.push_back(minskSum[i + 1]);

		double theta1 = std::atan2(edge1.z, edge1.x);
		double theta2 = std::atan2(edge2.z, edge2.x);

		// double angularDistance = theta2 - theta1;
		double dot = edge1.x*edge2.x + edge1.z*edge2.z;     // # dot product
		double det = edge1.x*edge2.z - edge1.z*edge2.x; //     # determinant
		double angularDistance = -std::atan2(det, dot);//  # atan2(y, x) or atan2(sin, cos)
		double modDistnace = 0.1;
		for (double tmpAngle = 0; tmpAngle < angularDistance; tmpAngle += modDistnace)
		{
			minskSumOut.push_back(verts[(i + 2) / 2] + rotateInXZPlane(rightNormal1, tmpAngle));
		}
	}

	Util::Vector edge1 = minskSum[i + 1] - minskSum[i];
	Util::Vector edge2 = minskSum[1] - minskSum[0];
	Util::Vector rightNormal1 = normalize(rotateInXZPlane(edge1, -M_PI_OVER_2));
	Util::Vector rightNormal2 = normalize(rotateInXZPlane(edge2, -M_PI_OVER_2));
	minskSumOut.push_back(minskSum[i]);
	minskSumOut.push_back(minskSum[i+1]);

	double theta1 = std::atan2(edge1.z, edge1.x);
	double theta2 = std::atan2(edge2.z, edge2.x);

	// double angularDistance = theta2 - theta1;
	double dot = rightNormal1.x*rightNormal2.x + rightNormal1.z*rightNormal2.z;     // # dot product
	double det = rightNormal1.x*rightNormal2.z - rightNormal1.z*rightNormal2.x; //     # determinant
	double angularDistance = -std::atan2(det, dot);//  # atan2(y, x) or atan2(sin, cos)
	double modDistnace = 0.1;
	for (double tmpAngle = 0; tmpAngle < angularDistance; tmpAngle += modDistnace)
	{
		minskSumOut.push_back(verts[0] + rotateInXZPlane(rightNormal1, tmpAngle));
	}
	// Need to insert more points between minkSum verts now, that are along the circle placed at the old vert.

	SteerLib::PolygonObstacle * pObs = new SteerLib::PolygonObstacle(minskSumOut);
	this->_obstacles.push_back(pObs);
	*/
	this->_obstacles.push_back(box);
}

void SimWorld::addOrientedObstacle(std::vector<double> centerPosition, float lengthX, float lengthZ, float ymin, float ymax, float thetaY)
{
	Util::Point p(centerPosition[0], centerPosition[1], centerPosition[2]);
	this->addOrientedObstacle(p, lengthX, lengthZ, ymin, ymax, thetaY);

}

void SimWorld::addAgent()
{

}

void SimWorld::addAgentRegion()
{

}

void SimWorld::clearWorld()
{
	for (size_t i = 0; i < this->_obstacles.size(); i++)
	{
		delete  this->_obstacles[i];
	}
	this->_obstacles.clear();
	for (size_t i = 0; i < this->_agents.size(); i++)
	{
		delete  this->_agents[i];
	}
	this->_agents.clear();

}

}

