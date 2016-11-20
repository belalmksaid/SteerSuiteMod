//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#include "SteerLib.h"
#include "VGAgent.h"
#include "VGAIModule.h"
#include <set>
#include <vector>
#include <fstream>
//#include <random>

using namespace SpaceSyntax;
/// @file VGAgent.cpp
/// @brief Implements the VGAgent class.

#define MAX_FORCE_MAGNITUDE 3.0f
#define MAX_SPEED 1.3f
#define AGENT_MASS 1.0f

VGAgent::VGAgent()
{
	_enabled = false;
}

VGAgent::~VGAgent()
{
	if (_enabled) {
		Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
		gSpatialDatabase->removeObject( this, bounds);
	}
	delete vg;
}

void VGAgent::disable()
{
	Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
	gSpatialDatabase->removeObject( this, bounds);
	_enabled = false;
}

void VGAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	// compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
	// because the value is not used in that case.
	Util::AxisAlignedBox oldBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);

	// initialize the agent based on the initial conditions
	_position = initialConditions.position;
	_forward = initialConditions.direction;
	_radius = initialConditions.radius;
	_velocity = initialConditions.speed * Util::normalize(initialConditions.direction);

	// compute the "new" bounding box of the agent
	Util::AxisAlignedBox newBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);

	if (!_enabled) {
		// if the agent was not enabled, then it does not already exist in the database, so add it.
		gSpatialDatabase->addObject( this, newBounds);
	}
	else {
		// if the agent was enabled, then the agent already existed in the database, so update it instead of adding it.
		gSpatialDatabase->updateObject( this, oldBounds, newBounds);
	}

	_enabled = true;

	if (initialConditions.goals.size() == 0) {
		throw Util::GenericException("No goals were specified!\n");
	}

	// iterate over the sequence of goals specified by the initial conditions.
	for (unsigned int i=0; i<initialConditions.goals.size(); i++) {
		if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
			_goalQueue.push(initialConditions.goals[i]);
			if (initialConditions.goals[i].targetIsRandom) {
				// if the goal is random, we must randomly generate the goal.
				SteerLib::AgentGoalInfo _goal;
				_goal.targetLocation = gSpatialDatabase->randomPositionWithoutCollisions(1.0f, true);
				_goalQueue.push(_goal);
			}
		}
		else {
			throw Util::GenericException("Unsupported goal type; VGAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET.");
		}
	}

	assert(_forward.length()!=0.0f);
	assert(_goalQueue.size() != 0);
	assert(_radius != 0.0f);
	
	std::set<SteerLib::ObstacleInterface*> obs;
	std::vector<Obstacle> obstacleList;
	obs = gEngine->getObstacles();
	Util::Point pMax, pMin;
	for (auto itr = obs.begin(); itr != obs.end(); itr++)
	{
		/*
		pMax = Util::Point((*itr)->getBounds().xmax, (*itr)->getBounds().ymax, (*itr)->getBounds().zmax);
		pMin = Util::Point((*itr)->getBounds().xmin, (*itr)->getBounds().ymin, (*itr)->getBounds().zmin);
		std::cout << pMin << std::endl << pMax << std::endl;
		obstacleList.push_back(Obstacle(pMin, pMax, 0, ObstacleType::BLOCK));
		*/
		std::vector<Util::Point> obstaclePoints;
		obstaclePoints = (*itr)->get2DStaticGeometry();
		obstacleList.push_back(Obstacle(obstaclePoints[0], obstaclePoints[1], obstaclePoints[2], obstaclePoints[3], (*itr)->getBounds().ymin, (*itr)->getBounds().ymax));
	}

	/*****HEATMAP PART*****/
	
	float gran = 0.5;
	// vg = new VisibilityGraph(-36.5, -12.5, 58.5, 16.5, 0.1, (int)(95 * gran), (int)(29 * gran));
	Util::Point v_min(-175.703860392201, 0, -110.025736389063);
	Util::Point v_max(175.703861091552, 0, 108.689692160872);
	int grid_x = (int)v_max.x - v_min.x;
	int grid_z = (int)v_max.z - v_min.z;
	Util::Point r_min(-160.5327, 0, -93.07484);
	Util::Point r_max(158.8249, 0, 60.05742);
	vg = new VisibilityGraph(v_min.x, v_min.z, v_max.x, v_max.z, 0.1, (int)(grid_x * gran), (int)(grid_z * gran));
	//vg = new VisibilityGraph(-10, -10, 10, 10, 0.1, 10, 10);
	vg->add_obstacles(obstacleList);
	//vg->add_region_gp(-10, -10, 9.5, 9.5, RegionType::REF);
	//vg->add_region_gp(-10, -10, 9.5, 9.5, RegionType::QUERY);
	vg->add_region_gp(r_min.x, r_min.z, r_max.x, r_max.z, RegionType::REF);
	vg->add_region_gp(r_min.x, r_min.z, r_max.x, r_max.z, RegionType::QUERY);
	//vg->add_region_gp(-10, -10, -9, -9, RegionType::REF);
	//vg->add_region_gp(-8, 7, -7, 7.5, RegionType::REF);
	//vg->add_region_gp(8, -1, 9, 0, RegionType::QUERY);
	//vg->add_region_gp(6, 6, 9, 9, RegionType::REF);
	//vg->add_region_gp(5, 7, 6, 9, RegionType::QUERY);
	//vg->add_region_gp(7, -10, 9, -8, RegionType::REF);

	//vg->generate_graph_cuda();

	vg->generate_graph_gp();
	std::cout << "CHECK CONN====" << vg->check_connection() << std::endl;
	std::cout << "Degree" << vg->get_meanDegree() << std::endl;
	std::cout << "Depth" << vg->get_meanTreeProp().first << std::endl;
	std::cout << "Entropy" << vg->get_meanTreeProp().second << std::endl;
	
	/*****ANALYSIS PART*****/
	/*
	clock_t begin, end;
	unsigned runNum = 10, gridNum = 100;
	std::ofstream afile, dfile, efile;
	std::string aname = "gpu_mah_time.txt", dname = "qhnewdata_depth.txt", ename = "qhnewdata_entropy.txt";
	// clear files
	afile.open(aname, std::ofstream::out | std::ofstream::trunc);
	afile.close();
	dfile.open(dname, std::ofstream::out | std::ofstream::trunc);
	dfile.close();
	efile.open(ename, std::ofstream::out | std::ofstream::trunc);
	efile.close();

	// open in append mode
	afile.open(aname, std::ofstream::out | std::ofstream::app);
	dfile.open(dname, std::ofstream::out | std::ofstream::app);
	efile.open(ename, std::ofstream::out | std::ofstream::app);

	vg = new VisibilityGraph(-10, -10, 10, 10, 0.1, 10, 10); //dummy
	vg->add_obstacles(obstacleList);
	float gran = 0;
	for (unsigned j = 50; j < gridNum+1; j+=10)
	{
		gran += 4;
		std::cout << "Progress Percentage === " << sizeof(char) << "__" << 100 * j / gridNum << std::endl;
		//vg->setup_gridPlan(-10, -10, 15, 10, 0.1, 25*gran, 20*gran);
		//vg->add_region_gp(-10, -10, 14.9999, 9.9999, RegionType::REF);
		//vg->add_region_gp(-10, -10, 14.9999, 9.9999, RegionType::QUERY);
		//vg->add_region_gp(-10, -5, -3.0001, 4.9999, RegionType::QUERY);
		//vg->add_region_gp(0, -10, 14.9999, -6.0001, RegionType::REF);
		//vg->add_region_gp(0, 6, 14.9999, 9.9999, RegionType::REF);
		//vg->setup_gridPlan(-36.5, -12.5, 58.5, 16.5, 0.1, (int) (95*gran), (int) (29*gran));
		//vg->add_region_gp(-36, -12, 58, 16, RegionType::REF);
		//vg->add_region_gp(-36, -12, 58, 16, RegionType::QUERY);
		vg->setup_gridPlan(-10, -10, 10, 10, 0.1, j, j);
		vg->add_region_gp(-10, -10, 9.5, 9.5, RegionType::REF);
		vg->add_region_gp(-10, -10, 9.5, 9.5, RegionType::QUERY);
		vg->add_obstacles(obstacleList);
		_nodeList = vg->get_nodeList();
		afile << j << " ";
		dfile << j << " ";
		efile << j << " ";
		//afile << gran << " ";
		//dfile << gran << " ";
		//efile << gran << " ";
		for (unsigned i = 0; i < runNum; i++)
		{
			float gpuK, gpuD, gpuE;
			std::pair<float, float> treeProp;
			double exeTime;
			begin = clock();
			vg->generate_graph_cuda();
			vg->generate_forest_cuda();
			//gpunk = vg->get_nodesDegree_cuda();
			//gpund = vg->get_nodesDepth_cuda();
			//gpune = vg->get_nodesEntropy_cuda();
			vg->get_output_cuda(gpuK, gpuD, gpuE);
			//vg->fetch_graph_cuda();
			//vg->get_output_cuda(gpuK, gpuD, gpuE);
			//treeProp = vg->get_meanTreeProp();
			//exeTime = vg->get_meanDegree();
			end = clock();
			//treeProp.first = vg->get_meanDegree();
			//treeProp.second = treeProp.first;
			//treeProp.first = gpuK;
			//treeProp.second = gpuD;
			//exeTime = gpuE;
			exeTime = ((double) 1000 * (end - begin)) / CLOCKS_PER_SEC;
			
			for (unsigned s = 0; s < vg->get_queryRegion().size(); s++)
			{
				afile << gpunk[s] << " ";
				dfile << gpund[s] << " ";
				efile << gpune[s] << " ";
			}
			afile << exeTime << " ";
			dfile << gpuD << " ";
			efile << gpuE << " ";
		}
		vg->clear_nodes();
		afile << "\n";
		dfile << "\n";
		efile << "\n";
	}
	afile.close();
	dfile.close();
	efile.close();
	return;
	*/
	
	_nodeList = vg->get_nodeList();
	_adjMatrix = vg->get_adjMatrix(); 
	for (auto i = 0; i < _nodeList.size(); i++)
	{
		float depth;
		
		/*** Degree metric storage ***/
		
		depth = vg->get_nodeDegree(i);
		depths.push_back(depth);
		/*** Depth/Entropy metric storage ***/
		/*
		Tree* graphTree;
		graphTree = vg->get_tree(i);
		depth = graphTree->get_entropy();
		depths.push_back(depth);
		delete graphTree; //IMPORTANT!!!
		*/
	}
}

void VGAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
	//vg->generate_graph();
	//vg->generate_graph_gp();
	//_nodeList = vg->get_nodeList();
	//_adjMatrix = vg->get_adjMatrix();

	// for this function, we assume that all goals are of type GOAL_TYPE_SEEK_STATIC_TARGET.
	// the error check for this was performed in reset().
	Util::AutomaticFunctionProfiler profileThisFunction( &VGAIGlobals::gPhaseProfilers->aiProfiler );

	Util::Vector vectorToGoal = _goalQueue.front().targetLocation - _position;

	// it is up to the agent to decide what it means to have "accomplished" or "completed" a goal.
	// for the VG AI, if the agent's distance to its goal is less than its radius, then the agent has reached the goal.
	if (vectorToGoal.lengthSquared() < _radius * _radius) {
		_goalQueue.pop();
		if (_goalQueue.size() != 0) {
			// in this case, there are still more goals, so start steering to the next goal.
			vectorToGoal = _goalQueue.front().targetLocation - _position;
		}
		else {
			// in this case, there are no more goals, so disable the agent and remove it from the spatial database.
			disable();
			return;
		}
	}

	// use the vectorToGoal as a force for the agent to steer towards its goal.
	// the euler integration step will clamp this vector to a reasonable value, if needed.
	// also, the Euler step updates the agent's position in the spatial database.
	_doEulerStep(vectorToGoal, dt);

}

SteerLib::EngineInterface * VGAgent::getSimulationEngine()
{
	return gEngine;
}

void VGAgent::draw()
{
#ifdef ENABLE_GUI
	// if the agent is selected, do some annotations just for demonstration
	if (gEngine->isAgentSelected(this)) {
		Util::Ray ray;
		ray.initWithUnitInterval(_position, _forward);
		float t = 0.0f;
		SteerLib::SpatialDatabaseItem * objectFound;
		Util::DrawLib::drawLine(ray.pos, ray.eval(1.0f));
		if (gSpatialDatabase->trace(ray, t, objectFound, this, false)) {
			Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gBlue);
		}
		else {
			Util::DrawLib::drawAgentDisc(_position, _forward, _radius);
		}
	}
	else {
		Util::DrawLib::drawAgentDisc(_position, _forward, _radius, Util::gGray40);
	}
	if (_goalQueue.front().goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
		Util::DrawLib::drawFlag(_goalQueue.front().targetLocation);
	}

	return;
	Util::Color nodeColor;
	float sum = vg->get_meanDegree() * _nodeList.size();
	float normDegree, gridSize;
	std::map<unsigned int, unsigned int> queryRegion, refRegion;
	queryRegion = vg->get_queryRegion();
	refRegion = vg->get_refRegion();
	//gridSize = (_nodeList[1].location.z - _nodeList[0].location.z) / 2;
	//Util::Vector front(-gridSize, 0, gridSize), side(gridSize, 0, gridSize);
	//std::cout << "MEANDEPTH>>>>>" << vg->get_meanTreeDepth() << std::endl;
	
	/*** Find max/min metric value ***/
	//float maxDegree = _adjMatrix.rowwise().sum().maxCoeff();
	float maxDegree = depths[0], minDegree = depths[0];
	sum = 0;
	for (auto i = 0; i < _nodeList.size(); i++)
	{
		if (depths[i] > maxDegree)
			maxDegree = depths[i];
		if (depths[i] < minDegree)
			minDegree = depths[i];
		sum += depths[i];
	}
	//std::cout << "MAXDEGREE>>>" << maxDegree << std::endl << "MEANDEGREE>>>" << sum / _nodeList.size() << std::endl;

	/*** Heat map for the metric ***/
	for (auto qitr = queryRegion.begin(); qitr != queryRegion.end(); qitr++)
	{
		unsigned i;
		i = qitr->second;
		float normColor, normColorB, normColorR, normColorG, maxDegreeT, minDegreeT, normval, maxval;
		Util::Point curr, p1, p2;
		std::vector<Util::Point> cellPoints;
		cellPoints = vg->get_cellPoints(i);
		Util::DrawLib::drawFlag(_nodeList[i].location, Util::gBlue);
		normColor = (depths[i] - minDegree) / (maxDegree - minDegree);
		maxval = _nodeList.size();
		//maxval = 2.5;
		normval = depths[i] / maxval;

		/*** Coloring normalized 0-1***/
		maxDegreeT = 0.4;
		minDegreeT = 0;
		normColorB = (normval - minDegreeT) / (maxDegreeT - minDegreeT);
		if (normColorB > 1)
		normColorB = 1;
		else if (normColorB < 0)
		normColorB = 0;

		maxDegreeT = 0.7;
		minDegreeT = 0.3;
		normColorR = (normval - minDegreeT) / (maxDegreeT - minDegreeT);
		if (normColorR > 1)
		normColorR = 1;
		else if (normColorR < 0)
		normColorR = 0;

		maxDegreeT = 1;
		minDegreeT = 0.6;
		normColorG = (normval - minDegreeT) / (maxDegreeT - minDegreeT);
		if (normColorG > 1)
		normColorG = 1;
		else if (normColorG < 0)
		normColorG = 0;

		//std::cout << "NORMDEGREE>>>" << normColor << "___MAX" << maxDegree << "___MIN" << minDegree<< std::endl;
		nodeColor = Util::Color(normColorR, normColorG, normColorB);
		//nodeColor = Util::Color(0.8,0.8,0.8);

		cellPoints[0].y = 0.15;
		cellPoints[1].y = 0.15;
		cellPoints[2].y = 0.15;
		cellPoints[3].y = 0.15;
		Util::DrawLib::drawQuad(cellPoints[0], cellPoints[1], cellPoints[2], cellPoints[3], nodeColor);
		//if (parentList[i] < 0)
		//	continue;
		//p1 = _nodeList[i].location;
		//p2 = _nodeList[parentList[i]].location;
		//p1.y = 0.15;
		//p2.y = 0.15;

		//Util::DrawLib::drawLine(p1, p2, Util::gOrange);
		/*** Draw whole graph***/
		for (auto j = _nodeList[i].adjList.begin(); j != _nodeList[i].adjList.end(); j++)
		{
			Util::Point p1, p2;
			if (j->nodeID <= i)
				continue;
			p1 = _nodeList[i].location;
			p2 = _nodeList[j->nodeID].location;
			p1.y = 0.2;
			p2.y = 0.2;
			//Util::DrawLib::drawLine(p1, p2, Util::gGreen, 0.1);
		}
		/*for (auto j = i + 1; j < _nodeList.size(); j++)
		{
			if (_adjMatrix(i, j) == 1)
			{
				//Util::Color fieldColor = Util::Color(distribution(generator), distribution(generator), distribution(generator));
				//if (i == 34 && (j == 40))
					//Util::DrawLib::drawLine(_nodeList[i].location, _nodeList[j].location, Util::gRed);
				//else
					Util::DrawLib::drawLine(_nodeList[i].location, _nodeList[j].location, Util::gOrange);
			}
		}*/
	}

	for (auto ritr = refRegion.begin(); ritr != refRegion.end(); ritr++)
	{
		unsigned i;
		i = ritr->second;
		float normColor, normColorB, normColorR, normColorG, maxDegreeT, minDegreeT, normval, maxval;
		Util::Point curr, p1, p2;
		std::vector<Util::Point> cellPoints;
		cellPoints = vg->get_cellPoints(i);
		Util::DrawLib::drawFlag(_nodeList[i].location, Util::gBlue);

		//std::cout << "NORMDEGREE>>>" << normColor << "___MAX" << maxDegree << "___MIN" << minDegree<< std::endl;
		//nodeColor = Util::Color(normColorR, normColorG, normColorB);
		nodeColor = Util::Color(0.95,1,0.2);

		Util::DrawLib::drawQuad(cellPoints[0], cellPoints[1], cellPoints[2], cellPoints[3], nodeColor);

		/*** Draw whole graph***/
		for (auto j = _nodeList[i].adjList.begin(); j != _nodeList[i].adjList.end(); j++)
		{
			Util::Point p1, p2;
			//if (j->nodeID <= i)
			//	continue;
			p1 = _nodeList[i].location;
			p2 = _nodeList[j->nodeID].location;
			p1.y = 0.2;
			p2.y = 0.2;
			//Util::DrawLib::drawLine(p1, p2, Util::gGreen, 0.1);
		}
	}
	/*** Display cluster connection at 3 points ***/
	/*
	Tree* vgTree;
	vgTree = vg->get_tree(270);
	std::vector<int> parentList = vgTree->get_parentList();
	std::vector<std::vector<int>> depthList = vgTree->get_depthList();
	float cr=1, cg=0.8, cb=0.8;
	for (auto dep = 0; dep < depthList.size(); dep++)
	{
		nodeColor = Util::Color(cr, cg / (dep + 1), cb / (dep + 1));
		for (auto node = 0; node < depthList[dep].size(); node++)
		{
			Util::Point p1, p2;
			unsigned int id;
			id = depthList[dep][node];
			p1 = _nodeList[id].location;
			p2 = _nodeList[parentList[id]].location;
			p1.y = 0.15;
			p2.y = 0.15;
			Util::DrawLib::drawLine(p1, p2, nodeColor);
		}
	}

	delete vgTree;
	parentList.clear();
	depthList.clear();
	vgTree = vg->get_tree(10);
	parentList = vgTree->get_parentList();
	depthList = vgTree->get_depthList();
	cr = 0.8, cg = 1, cb = 0.8;
	for (auto dep = 0; dep < depthList.size(); dep++)
	{
		nodeColor = Util::Color(cr / (dep + 1), cg, cb / (dep + 1));
		for (auto node = 0; node < depthList[dep].size(); node++)
		{
			Util::Point p1, p2;
			unsigned int id;
			id = depthList[dep][node];
			p1 = _nodeList[id].location;
			p2 = _nodeList[parentList[id]].location;
			p1.y = 0.15;
			p2.y = 0.15;
			Util::DrawLib::drawLine(p1, p2, nodeColor);
		}
	}

	delete vgTree;
	parentList.clear();
	depthList.clear();
	vgTree = vg->get_tree(250);
	parentList = vgTree->get_parentList();
	depthList = vgTree->get_depthList();
	cr = 0.8, cg = 0.8, cb = 1;
	for (auto dep = 0; dep < depthList.size(); dep++)
	{
		nodeColor = Util::Color(cr / (dep + 1), cg / (dep + 1), cb);
		for (auto node = 0; node < depthList[dep].size(); node++)
		{
			Util::Point p1, p2;
			unsigned int id;
			id = depthList[dep][node];
			p1 = _nodeList[id].location;
			p2 = _nodeList[parentList[id]].location;
			p1.y = 0.15;
			p2.y = 0.15;
			Util::DrawLib::drawLine(p1, p2, nodeColor);
		}
	}*/
#endif
}


void VGAgent::_doEulerStep(const Util::Vector & steeringDecisionForce, float dt)
{
	// compute acceleration, _velocity, and newPosition by a simple Euler step
	const Util::Vector clippedForce = Util::clamp(steeringDecisionForce, MAX_FORCE_MAGNITUDE);
	Util::Vector acceleration = (clippedForce / AGENT_MASS);
	_velocity = _velocity + (dt*acceleration);
	_velocity = clamp(_velocity, MAX_SPEED);  // clamp _velocity to the max speed
	const Util::Point newPosition = _position + (dt*_velocity);

	// For this VG agent, we just make the orientation point along the agent's current velocity.
	if (_velocity.lengthSquared() != 0.0f) {
		_forward = normalize(_velocity);
	}

	// update the database with the new agent's setup
	Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	Util::AxisAlignedBox newBounds(newPosition.x - _radius, newPosition.x + _radius, 0.0f, 0.0f, newPosition.z - _radius, newPosition.z + _radius);
	gSpatialDatabase->updateObject( this, oldBounds, newBounds);

	_position = newPosition;
}
