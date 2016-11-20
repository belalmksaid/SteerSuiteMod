/*
 * SteerSuite.cpp
 *
 *  Created on: Oct 6, 2015
 *      Author: Glen
 */

#include "SteerSuite.h"
#include <iostream>

#include <exception>
#include "SteerLib.h"
#include "core/CommandLineEngineDriver.h"
#include "core/GLFWEngineDriver.h"
#include "core/QtEngineDriver.h"
#include "SimulationPlugin.h"
#include "core/SteerSim.h"
#include "obstacles/BoxObstacle.h"
/*
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/c_array.hpp>
// #include <boost/geometry/geometries/cartesian2d.hpp>
// #include <boost/geometry/geometries/adapted/c_array_cartesian.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
*/
#include <clipper/clipper.hpp>
#include <clipper/clipper_extra.hpp>

// BOOST_GEOMETRY_REGISTER_C_ARRAY_CS(cs::cartesian)

using namespace SpaceSyntax;

namespace SteerSuite {


SteerSuite::SteerSuite() {
	// TODO Auto-generated constructor stub
	_visibilityGraph = NULL;
#ifdef _WIN32
	this->setConfigFileName("../config_revit.xml");
#else
	this->setConfigFileName("build/config_unix.xml");
#endif
}

SteerSuite::SteerSuite(std::string configFileName)
{
	// TODO Auto-generated constructor stub
	_visibilityGraph = NULL;
	this->setConfigFileName(configFileName);
}

SteerSuite::~SteerSuite() {
	// TODO Auto-generated destructor stub
	delete _visibilityGraph;
}


LogData * SteerSuite::getSimData()
{
	return this->_driver->getLogData();
}


double SteerSuite::simulate()
{
	return this->simulate(NULL, 0);
}


void SteerSuite::init(SimWorld * world)
{
	// save the original cerr streambuf, so that we can restore it on an exception.
	std::streambuf * cerrOriginalStreambuf = std::cerr.rdbuf();

	std::ofstream coutRedirection;
	std::ofstream cerrRedirection;
	std::ofstream clogRedirection;

	this->_world = world;


	int argc = 3;
	// char **argv;

	// let's make our own array of strings
	/*
	char *argv[] = {
	"./steerpluginTester.exe",
	"-ai", "rvo2dAI",
	"-testcase", "testcases/blank.xml",
	"-config", "build/config_revit.xml",
	"-numFrames", "1000"
	};
	*/
	// TODO can still do better.
	char *argv[] = {
		"steerpluginTester.exe",
		"-config", (char *) this->_configFileName.c_str()
	};


	_simulationOptions = new SteerLib::SimulationOptions();
	initializeOptionsFromCommandLine(argc, argv, *_simulationOptions);

	// re-direct cout, cerr, and clog, if the user specified it through options.
	if (_simulationOptions->globalOptions.coutRedirectionFilename != "") {
		coutRedirection.open(_simulationOptions->globalOptions.coutRedirectionFilename.c_str());
		std::cout.rdbuf(coutRedirection.rdbuf());
	}
	if (_simulationOptions->globalOptions.cerrRedirectionFilename != "") {
		cerrRedirection.open(_simulationOptions->globalOptions.cerrRedirectionFilename.c_str());
		std::cerr.rdbuf(cerrRedirection.rdbuf());
	}
	if (_simulationOptions->globalOptions.clogRedirectionFilename != "") {
		clogRedirection.open(_simulationOptions->globalOptions.clogRedirectionFilename.c_str());
		std::clog.rdbuf(clogRedirection.rdbuf());
	}


	try {
		this->_driver = new CommandLineEngineDriver();
		this->_driver->init(_simulationOptions);
		for (size_t i = 0; i < this->_world->getObstacles().size(); i++)
		{
			this->_driver->getEngine()->addObstacle(this->_world->getObstacles()[i]);
			this->_driver->getEngine()->getSpatialDatabase()->addObject(this->_world->getObstacles()[i],
				this->_world->getObstacles()[i]->getBounds());
		}

	}
	catch (std::exception &e) {

		std::cerr << "\nERROR: exception caught in main:\n" << e.what() << "\n";

		// there is a chance that cerr was re-directed.  If this is true, then also echo
		// the error to the original cerr.
		// there is a chance that cerr was re-directed.  If this is true, then also echo 
		// the error to the original cerr.
		if (std::cerr.rdbuf() != cerrOriginalStreambuf) {
			std::cerr.rdbuf(cerrOriginalStreambuf);
			std::cerr << "\nERROR: exception caught in main:\n" << e.what() << "\n";
		}

		if (coutRedirection.is_open()) coutRedirection.close();
		if (cerrRedirection.is_open()) cerrRedirection.close();
		if (clogRedirection.is_open()) clogRedirection.close();

	}

	if (coutRedirection.is_open()) coutRedirection.close();
	if (cerrRedirection.is_open()) cerrRedirection.close();
	if (clogRedirection.is_open()) clogRedirection.close();
}

void SteerSuite::initRender(SimWorld * world)
{

	// save the original cerr streambuf, so that we can restore it on an exception.
	std::streambuf * cerrOriginalStreambuf = std::cerr.rdbuf();

	std::ofstream coutRedirection;
	std::ofstream cerrRedirection;
	std::ofstream clogRedirection;

	this->_world = world;


	int argc = 3;
	// char **argv;

	// let's make our own array of strings
/*
	char *argv[] = {
		"./steerpluginTester.exe",
		"-ai", "rvo2dAI",
		"-testcase", "testcases/blank.xml",
		"-config", "build/config_revit.xml",
		"-numFrames", "1000"
	};
*/
	// TODO can still do better.
	char *argv[] = {
		"steerpluginTester.exe",
		"-config", (char *) this->_configFileName.c_str()
	};

	_simulationOptions = new SteerLib::SimulationOptions();
	initializeOptionsFromCommandLine(argc, argv, *_simulationOptions);

	// re-direct cout, cerr, and clog, if the user specified it through options.
	if (_simulationOptions->globalOptions.coutRedirectionFilename != "") {
		coutRedirection.open(_simulationOptions->globalOptions.coutRedirectionFilename.c_str());
		std::cout.rdbuf(coutRedirection.rdbuf());
	}
	if (_simulationOptions->globalOptions.cerrRedirectionFilename != "") {
		cerrRedirection.open(_simulationOptions->globalOptions.cerrRedirectionFilename.c_str());
		std::cerr.rdbuf(cerrRedirection.rdbuf());
	}
	if (_simulationOptions->globalOptions.clogRedirectionFilename != "") {
		clogRedirection.open(_simulationOptions->globalOptions.clogRedirectionFilename.c_str());
		std::clog.rdbuf(clogRedirection.rdbuf());
	}


	try {
		this->_driver = GLFWEngineDriver::getInstance();
		this->_driver->init(_simulationOptions);
		for (size_t i = 0; i < this->_world->getObstacles().size(); i++ )
		{
			this->_driver->getEngine()->addObstacle(this->_world->getObstacles()[i]);
			this->_driver->getEngine()->getSpatialDatabase()->addObject(this->_world->getObstacles()[i],
					this->_world->getObstacles()[i]->getBounds());
		}

	}
	catch (std::exception &e) {

		std::cerr << "\nERROR: exception caught in main:\n" << e.what() << "\n";

		// there is a chance that cerr was re-directed.  If this is true, then also echo
		// the error to the original cerr.
		// there is a chance that cerr was re-directed.  If this is true, then also echo 
		// the error to the original cerr.
		if (std::cerr.rdbuf() != cerrOriginalStreambuf) {
			std::cerr.rdbuf(cerrOriginalStreambuf);
			std::cerr << "\nERROR: exception caught in main:\n" << e.what() << "\n";
		}

		if (coutRedirection.is_open()) coutRedirection.close();
		if (cerrRedirection.is_open()) cerrRedirection.close();
		if (clogRedirection.is_open()) clogRedirection.close();

	}

	if (coutRedirection.is_open()) coutRedirection.close();
	if (cerrRedirection.is_open()) cerrRedirection.close();
	if (clogRedirection.is_open()) clogRedirection.close();
}

/// Basically removes everything from the running simulation
void SteerSuite::resetSimulation()
{
	// Only removes obstacles for now.
	this->_driver->getEngine()->removeAllObstacles();
	this->_driver->getEngine()->getSpatialDatabase()->clearDatabase();
}
/// Basically removes everything from the running simulation
void SteerSuite::restartSimulation(SimWorld * world)
{
	this->_world = world;
	for (size_t i = 0; i < this->_world->getObstacles().size(); i++)
	{
		this->_driver->getEngine()->addObstacle(this->_world->getObstacles()[i]);
		this->_driver->getEngine()->getSpatialDatabase()->addObject(this->_world->getObstacles()[i],
			this->_world->getObstacles()[i]->getBounds());
	}

}

void SteerSuite::loadSimulation()
{
	this->_driver->loadSimulation();
}

void SteerSuite::unloadSimulation()
{
	this->_driver->unloadSimulation();
}

double SteerSuite::simulate(const double *x, const size_t length)
{

	double width=0.25;
	// std::cout << "Params: ";
	for (size_t i = 0; i < length; i+=2 )
	{
		// std::cout << "(" << x[i] << ", " << x[i+1] << ")";
		// SteerLib::BoxObstacle * box = new SteerLib::BoxObstacle(x[i]-width,x[i]+width,0,1,x[i+1]-width,x[i+1]+width);
		// this->_driver->getEngine()->addObstacle(box);
		// this->_driver->getEngine()->getSpatialDatabase()->addObject(box, box->getBounds());
	}
	std::cout << std::endl;
	this->_driver->run();

	return 2.0;
}

double SteerSuite::simulateSteeringAlgorithm(const double *x, const size_t length, SteerLib::Behaviour behave)
{

	this->_driver->loadSimulation();
	this->setSteeringAlgorithmBehaviour(behave);

	this->_driver->startSimulation();

	this->_driver->unloadSimulation();

	return 2.0;
}

void SteerSuite::setSteeringAlgorithmBehaviour(SteerLib::Behaviour behave)
{
	const std::vector<SteerLib::AgentInterface*> agentsAlias = this->_driver->getEngine()->getAgents();
	for (size_t a=0; a < agentsAlias.size(); a++)
	{
		agentsAlias[a]->setParameters(behave);
	}

}

const SteerLib::OptionDictionary & SteerSuite::getModuleOptions(const std::string moduleName)
{
	return this->_driver->getEngine()->getModuleOptions(moduleName);
}

void SteerSuite::finish()
{
	this->_driver->finish();
}

/*
typedef boost::geometry::model::d2::point_xy<double> point_2d;
typedef boost::geometry::model::polygon<point_2d> polygon_2d;

std::vector<Util::Point> boostPointsToPoints(std::vector<point_2d> pts)
{
	std::vector<Util::Point> out;
	for (size_t i = 0; i < pts.size(); i++)
	{
		out.push_back(Util::Point(pts[i].x(), 0, -pts[i].y()));
	}
	return out;
}

std::vector<point_2d>  pointsToBoostPoints(std::vector<Util::Point> pts)
{
	std::vector<point_2d> out;
	for (size_t i = 0; i < pts.size(); i++)
	{
		out.push_back(point_2d(pts[i].x, -pts[i].z));
	}
	return out;
}
*/

ClipperLib::Path vectorToPath(std::vector<Util::Point> pts, double scale)
{
	ClipperLib::Path pg;
	for (size_t i = 0; i < pts.size(); i++)
	{
		pg.push_back(ClipperLib::IntPoint((ClipperLib::cInt)(pts[i].x * scale), (ClipperLib::cInt)(pts[i].z * scale)));
	}
	return pg;
}

std::vector<Util::Point> clipperPointsToPoints(ClipperLib::Path pts, double scale)
{
	std::vector<Util::Point> out;
	for (size_t i = 0; i < pts.size(); i++)
	{
		out.push_back(Util::Point(pts[i].X/scale, 0, pts[i].Y/scale));
	}
	return out;
}

std::vector<Util::Point> createCircle(size_t points, double radius)
{
	std::vector<Util::Point> pts;
	float angleIncrement = (2 * M_PI) / (float)points;
	// angleIncrement = 90.0f;
	Util::Point turningVector = Util::Point(1 * radius, 0.0, 0 * radius);
	// point nextTurningVector = rotateInXZPlane(turningVector, angleIncrement);
	// std::cout << "Point is: (" << turningVector.X << ", " << turningVector.Y << ")" << std::endl;
	pts.push_back(turningVector);
	for (size_t i = 0; i < (points - 1); i++)
	{
		// DrawLib::drawLine(loc + turningVector, loc + nextTurningVector, color);
		turningVector = rotateInXZPlane(turningVector, -angleIncrement);
		// nextTurningVector = rotateInXZPlane(turningVector, angleIncrement);
		// std::cout << "Point is: (" << turningVector.X << ", " << turningVector.Y << ")" << std::endl;
		pts.push_back(turningVector);

	}

	// turningVector = Util::Point(1 * radius, 0.0, 0 * radius);
	// pts.push_back(turningVector);
	return pts;
}

/*
std::vector<ClipperLib::Paths> computeMinowskiSums()
{

	
	std::set<SteerLib::ObstacleInterface*> obstacles;
	std::vector<Obstacle> obstacleList;
	obstacles = _driver->getEngine()->getObstacles();
	Util::Point pMax, pMin;
	for (size_t i=0 ; i < obstacles.size(); i++)
	{
	std::vector<Util::Point> obstaclePointsA = obstacles[i]->get2DStaticGeometry();
	for (size_t j = i + 1; j < obstacles.size(); j++)
	{

	}
	//std::cout << pMin << std::endl << pMax << std::endl;
	//obstacleList.push_back(Obstacle(pMin, pMax, 0, ObstacleType::BLOCK)); //this is for axis aligned box
	}
}
*/


/*
double SteerSuite::computeIntersections(bool debugDraw)
{
	// bool debugDraw = false;
	ClipperLib::Clipper c;
	ClipperLib::Paths cp;
	ClipperLib::Paths subject, clip, solution, solution2;
	// typedef boost::geometry::model::box<point_2d> box_2d;
	double scale = 1000000.0;
	std::vector<ClipperLib::Paths> obstacles;
	std::vector<ClipperLib::Paths> obstaclesOriginal, interections;
	double radius = 0.5;
	ClipperLib::Path cir = vectorToPath(createCircle(24, radius), scale);
	clip.clear();
	clip.push_back(cir);


	std::set<SteerLib::ObstacleInterface*> obs;
	std::vector<Obstacle> obstacleList;
	obs = _driver->getEngine()->getObstacles();
	Util::Point pMax, pMin;
	for (auto itr = obs.begin(); itr != obs.end(); itr++)
	{
		std::vector<Util::Point> obstaclePoints;
		obstaclePoints = (*itr)->get2DStaticGeometry();
		ClipperLib::Path sol = vectorToPath(obstaclePoints, scale);
		subject.clear();
		subject.push_back(sol);
		solution2.clear();
		MinkowskiSum(subject[0], clip[0], solution2, true);
		obstacles.push_back(solution2);

		if (debugDraw)
		{
			obstaclesOriginal.push_back(subject);
			SteerLib::PolygonObstacle * p = new SteerLib::PolygonObstacle(clipperPointsToPoints(solution2[0], scale));
			this->_driver->getEngine()->addObstacle(p);
			this->_driver->getEngine()->getSpatialDatabase()->addObject(p, p->getBounds());
			p->setColour(Util::Color(0.0, 1.0, 0.0));
		}
	}
	*/

	/*
	
	// polygon_2d poly;

	std::vector<Util::Point> pts;
	pts.push_back(Util::Point(0.0, 0.0, -0.0));
	pts.push_back(Util::Point(2.4, 0.0, -1.3));
	pts.push_back(Util::Point(2.0, 0.0, -1.7));
	pts.push_back(Util::Point(-0.3, 0.0, -0.3));
	// pts.push_back(Util::Point(0.0, 0.0, -0.0));
	// boost::geometry::assign_points(poly, pts);
	// boost::geometry::correct(poly);
	// std::cout << "A: " << boost::geometry::dsv(poly) << std::endl;
	ClipperLib::Path pg = vectorToPath(pts, scale);
	subject.push_back(pg);

	SteerLib::PolygonObstacle * p = new SteerLib::PolygonObstacle((pts));
	
	this->_driver->getEngine()->addObstacle(p);
	this->_driver->getEngine()->getSpatialDatabase()->addObject(p,p->getBounds());
	p->setColour(Util::Color(0.0, 1.0, 0.0));


	pts.clear();
	pts.push_back(Util::Point(1.5, 0.0, -1.5));
	pts.push_back(Util::Point(4.5, 0.0, -1.5));
	pts.push_back(Util::Point(4.5, 0.0, -2.5));
	pts.push_back(Util::Point(1.5, 0.0, -2.5));
	// pts.push_back(Util::Point(1.5, 0.0, -1.5));

	ClipperLib::Path pg2 = vectorToPath(pts, scale);
	clip.push_back(pg2);

	SteerLib::PolygonObstacle * p1 = new SteerLib::PolygonObstacle((pts));

	this->_driver->getEngine()->addObstacle(p1);
	this->_driver->getEngine()->getSpatialDatabase()->addObject(p1, p1->getBounds());
	p1->setColour(Util::Color(0.0, 0.0, 1.0));

	
	c.AddPaths(subject, ClipperLib::ptSubject, true);
	c.AddPaths(clip, ClipperLib::ptClip, true);

	if (!c.Execute(ClipperLib::ctIntersection, solution))
	{
		std::cout << ("Intersection failed!\n\n");
		// return 1;
	}
	if (solution.size() > 0)
	{
		std::cout << "Area of random Polygon is " << Area(solution[0]) / (scale*scale) << std::endl;
		SteerLib::PolygonObstacle * p1 = new SteerLib::PolygonObstacle(clipperPointsToPoints(solution[0], scale));

		this->_driver->getEngine()->addObstacle(p1);
		this->_driver->getEngine()->getSpatialDatabase()->addObject(p1, p1->getBounds());
		p1->setColour(Util::Color(1.0, 0.0, 0.0));
	}
	else
	{
		std::cout << "No intersection found" << std::endl;
	}
	// boost::geometry::intersection_inserter<polygon_2d>(cb, poly, std::back_inserter(v));

	// std::cout << "Clipped output polygons" << std::endl;

	*/
/*
	double areaSum = 0;

	for (size_t i=0 ; i < obstacles.size(); i++)
	{
		// ClipperLib::Paths obstaclePointsA = obstacles[i];
		// subject.clear();
		subject = obstacles[i];
		for (size_t j = i + 1; j < obstacles.size(); j++)
		{
			clip.clear();
			clip = obstacles[j];

			c.AddPaths(subject, ClipperLib::ptSubject, true);
			c.AddPaths(clip, ClipperLib::ptClip, true);
			if (!c.Execute(ClipperLib::ctIntersection, solution))
			{
				std::cout << ("Intersection failed!\n\n");
				// return 1;
			}
			if (solution.size() > 0)
			{
				double area = Area(solution[0]) / (scale*scale);
				areaSum += area;
				// std::cout << "Area of random Polygon is " << area << std::endl;
				if (debugDraw)
				{
					interections.push_back(solution);
					SteerLib::PolygonObstacle * p1 = new SteerLib::PolygonObstacle(clipperPointsToPoints(solution[0], scale));
					this->_driver->getEngine()->addObstacle(p1);
					this->_driver->getEngine()->getSpatialDatabase()->addObject(p1, p1->getBounds());
					p1->setColour(Util::Color(1.0, 0.0, 0.0));
				}
			}
			else
			{
				// std::cout << "No intersection found" << std::endl;
			}
			c.Clear();
		}
		//std::cout << pMin << std::endl << pMax << std::endl;
		//obstacleList.push_back(Obstacle(pMin, pMax, 0, ObstacleType::BLOCK)); //this is for axis aligned box
	}

	if (debugDraw)
	{
		SVGBuilder svg;
		svg.style.penWidth = 0.1;
		svg.style.brushClr = 0x1200009C;
		svg.style.penClr = 0xCCD3D3DA;
		svg.style.pft = ClipperLib::pftEvenOdd;
		for (size_t i = 0; i < obstaclesOriginal.size(); i++)
		{
			svg.AddPaths(obstaclesOriginal[i]);
		}
		svg.style.brushClr = 0x129C0000;
		svg.style.penClr = 0xCCFFA07A;
		svg.style.pft = ClipperLib::pftEvenOdd;
		for (size_t i = 0; i < obstacles.size(); i++)
		{
			svg.AddPaths(obstacles[i]);
		}
		svg.style.brushClr = 0x6080ff9C;
		svg.style.penClr = 0xFF003300;
		svg.style.pft = ClipperLib::pftNonZero;
		for (size_t i = 0; i < interections.size(); i++)
		{
			svg.AddPaths(interections[i]);
		}
		svg.SaveToFile("solution.svg", 1.0 / scale);
	}

	return areaSum;
	
}

*/ 

/*
double SteerSuite::alignmentPenalty()
{
*/
	/*
		This should be weighted by the distances away from each other.
		Obstacle that are farther away from each other should have less
		affect on this metric.
	*/
	/*
	std::set<SteerLib::ObstacleInterface*> obs;
	obs = _driver->getEngine()->getObstacles();
	// std::vector<SteerLib::ObstacleInterface*> obstacleList(obs.begin(), obs.end());
	double thetaSum = 0;
	for (auto itr = obs.begin(); itr != obs.end(); itr++)
	{
		// SteerLib::ObstacleInterface* o = (*itr);
		SteerLib::OrientedBoxObstacle * o = dynamic_cast<SteerLib::OrientedBoxObstacle *>((*itr));
		if ( o != NULL )
		{
			double theta1 = o->getTheta();
			for (auto itr2 = itr; itr2 != obs.end(); itr2++)
			{
				SteerLib::OrientedBoxObstacle * o2 = dynamic_cast<SteerLib::OrientedBoxObstacle *>((*itr2));
				if ( o2 != NULL )
				{
					double theta2 = o2->getTheta();
					double diff = std::fabs(theta2 - theta1);
					thetaSum += std::fmod(diff, M_PI_OVER_2);
				}
			}
		}
	}
	return thetaSum;
}
*/

void SteerSuite::init_visibilityGraph(Util::Point pMin, Util::Point pMax, unsigned int gridNumX, unsigned int gridNumZ, float height)
{
	if (_visibilityGraph != NULL)
	{
		delete _visibilityGraph;
		_visibilityGraph = NULL;
	}
	_visibilityGraph = new VisibilityGraph(pMin.x, pMin.z, pMax.x, pMax.z, height, gridNumX, gridNumZ);
}

void SteerSuite::setup_visibilityGraph()
{
	std::set<SteerLib::ObstacleInterface*> obs;
	std::vector<Obstacle> obstacleList;
	obs = _driver->getEngine()->getObstacles();
	Util::Point pMax, pMin;
	for (auto itr = obs.begin(); itr != obs.end(); itr++)
	{
		pMax = Util::Point((*itr)->getBounds().xmax, (*itr)->getBounds().ymax, (*itr)->getBounds().zmax);
		pMin = Util::Point((*itr)->getBounds().xmin, (*itr)->getBounds().ymin, (*itr)->getBounds().zmin);
		std::vector<Util::Point> obstaclePoints;
		obstaclePoints = (*itr)->get2DStaticGeometry();
		//std::cout << pMin << std::endl << pMax << std::endl;
		//obstacleList.push_back(Obstacle(pMin, pMax, 0, ObstacleType::BLOCK)); //this is for axis aligned box
		obstacleList.push_back(Obstacle(obstaclePoints[0], obstaclePoints[1], obstaclePoints[2], obstaclePoints[3], (*itr)->getBounds().ymin, (*itr)->getBounds().ymax));
	}
	_visibilityGraph->clear_obstacles();
	_visibilityGraph->add_obstacles(obstacleList);
	_visibilityGraph->generate_graph_gp();
}

float SteerSuite::get_meanDegree()
{
	return _visibilityGraph->get_meanDegree();
}

float SteerSuite::get_meanTreeDepth()
{
	return _visibilityGraph->get_meanTreeDepth();
}

float SteerSuite::get_meanTreeEntropy()
{
	return _visibilityGraph->get_meanTreeEntropy();
}

void SteerSuite::add_visibilityRegion(Util::Point pMin, Util::Point pMax)
{
	_visibilityGraph->add_region_gp(pMin.x, pMin.z, pMax.x, pMax.z, RegionType::QUERY);
}

void SteerSuite::add_queryRegion(Util::Point pMin, Util::Point pMax)
{
	_visibilityGraph->add_region_gp(pMin.x, pMin.z, pMax.x, pMax.z, RegionType::QUERY);
}

void SteerSuite::add_refRegion(Util::Point pMin, Util::Point pMax)
{
	_visibilityGraph->add_region_gp(pMin.x, pMin.z, pMax.x, pMax.z, RegionType::REF);
}

void SteerSuite::add_visibilityPoint(Util::Point point)
{
	_visibilityGraph->add_node_exact(point);
}

void SteerSuite::clear_visibilityGraph()
{
	_visibilityGraph->clear_nodes();
}

std::vector<Util::Point> SteerSuite::get_visibilityNodes()
{
	std::vector<Util::Point> pointList;
	std::vector<SpaceSyntax::Node> nodeList = _visibilityGraph->get_nodeList();
	for (auto itr = nodeList.begin(); itr != nodeList.end(); itr++)
		pointList.push_back(itr->location);
	return pointList;
}

std::vector<Util::Point> SteerSuite::get_queryNodes()
{
	std::vector<Util::Point> pointList;
	std::vector<SpaceSyntax::Node> nodeList = _visibilityGraph->get_queryList();
	for (auto itr = nodeList.begin(); itr != nodeList.end(); itr++)
		pointList.push_back(itr->location);
	return pointList;
}

std::vector<Util::Point> SteerSuite::get_refNodes()
{
	std::vector<Util::Point> pointList;
	std::vector<SpaceSyntax::Node> nodeList = _visibilityGraph->get_refList();
	for (auto itr = nodeList.begin(); itr != nodeList.end(); itr++)
		pointList.push_back(itr->location);
	return pointList;
}

std::vector<std::vector<Util::Point>> SteerSuite::get_visibilityLines()
{
	std::vector<std::vector<Util::Point>> lineList;
	std::vector<SpaceSyntax::Node> nodeList = _visibilityGraph->get_nodeList();
	for (auto i = 0; i < nodeList.size(); i++)
		for (auto j = i+1; j < nodeList.size(); j++)
			if (_visibilityGraph->get_adjMatrix(i, j) == 1)
			{
				Util::Point p1, p2;
				std::vector<Util::Point> tmp;
				p1 = nodeList[i].location;
				p2 = nodeList[j].location;
				tmp.push_back(p1);
				tmp.push_back(p2);
				lineList.push_back(tmp);
			}

	return lineList;
}

float SteerSuite::get_nodeDegree(unsigned int index)
{
	return _visibilityGraph->get_nodeDegree(index); // not robust 
}

float SteerSuite::get_nodeTreeDepth(unsigned int index)
{
	return _visibilityGraph->get_treeDepth(index); // not robust 
}

float SteerSuite::get_nodeTreeEntropy(unsigned int index)
{
	return _visibilityGraph->get_treeEntropy(index); // not robust 
}

float SteerSuite::get_nodeDegree(Util::Point target)
{
	return _visibilityGraph->get_nodeDegree(target); // not robust 
}

float SteerSuite::get_nodeTreeDepth(Util::Point target)
{
	return _visibilityGraph->get_treeDepth(target); // not robust 
}

float SteerSuite::get_nodeTreeEntropy(Util::Point target)
{
	return _visibilityGraph->get_treeEntropy(target); // not robust 
}

std::vector<float> SteerSuite::get_nodeTreeProp(unsigned int index)
{
	std::vector<float> res;
	std::pair<float, float> resPair = _visibilityGraph->get_treeProp(index);
	res.push_back(resPair.first);
	res.push_back(resPair.second);
	return res;
}

std::vector<float> SteerSuite::get_nodeTreeProp(Util::Point target)
{
	std::vector<float> res;
	std::pair<float, float> resPair = _visibilityGraph->get_treeProp(target);
	res.push_back(resPair.first);
	res.push_back(resPair.second);
	return res;
}

std::vector<float> SteerSuite::get_meanTreeProp()
{
	std::vector<float> res;
	std::pair<float, float> resPair = _visibilityGraph->get_meanTreeProp();
	res.push_back(resPair.first);
	res.push_back(resPair.second);
	return res;
}

} /* namespace SteerSuite */
