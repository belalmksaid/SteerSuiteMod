//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


/// @file steersim/src/Main.cpp
/// @brief Entry point of SteerSim.

#include <exception>
#include "core/CommandLineEngineDriver.h"
#include "core/GLFWEngineDriver.h"
#include "SimulationPlugin.h"
#include "core/SteerSim.h"

// #include "AntTweakBar/include/AntTweakBar.h"

#include "SteerSuite.h"
#include "SimWorld.h"

#include "src/cmaes.h"
#include <iomanip>

#include "CMAOptimize.h"
#include "RoundRobinOptimization.h"
#include "HierarchicalOptimization.h"
// #include "src/RoundRobinCMA.cpp"
// #include "RockstarOptimize.h"
#include "Graph.h"
// #include "GraphFunction.h"
// #include "GraphOptimize.h"
#include "OptimizationParameters.h"
#include "OptimizationConfiguration.h"

// #include "src/esostrategy.h"
// #include "src/esostrategy.cc"
// #include <cctype>
// #include <algorithm>


// using namespace SteerLib;
// using namespace Util;

int diversityOptimizeDegreeFromFile(std::string configFileName, SteerOpt::OptimizationConfiguration optConfig)
{
	Graphing::Graph graph;

	graph.importFromFile(optConfig._graphFileName);
	graph.computeMinkowskiSums(optConfig._clearence_distance);

	SteerSuite::SimWorld * world = new SteerSuite::SimWorld();
	SteerSuite::SteerSuite steerSuite;
	steerSuite.setConfigFileName(configFileName);

	Logger * _logger = LogManager::getInstance()->createLogger("../results/memberData.csv", LoggerType::BASIC_WRITE);
	_logger->addDataField("member", DataType::Integer);
	_logger->addDataField("metric", DataType::Float);
	_logger->addDataField("degree", DataType::Float);
	_logger->addDataField("depth", DataType::Float);
	_logger->addDataField("entropy", DataType::Float);
	_logger->addDataField("diversity", DataType::Float);
	_logger->addDataField("clearence", DataType::Float);
	_logger->addDataField("alignment", DataType::Float);
	_logger->addDataField("wall_length", DataType::Float);

	// LETS TRY TO WRITE THE LABELS OF EACH FIELD
	std::stringstream labelStream;
	unsigned int i;
	for (i = 0; i < _logger->getNumberOfFields() - 1; i++)
	{
		labelStream << _logger->getFieldName(i) << " ";
	}
	labelStream << _logger->getFieldName(i);
	// _data = labelStream.str();
	_logger->writeData(labelStream.str());

	Graphing::OptimizationParameters params;
	params.loadFromFile(optConfig._parameterFileName);
	params.saveToFile("../results/testParamsave.xml");
	graph.saveSVGToFile("../results/testOut.svg", params);
	// std::cout << "Parameters: " << std::endl <<
	//	params.toString() << std::endl;

	

	if (false)
	{ // Compute new parameter file
		// Asume only one query region
		Graphing::OptimizationParameters params_;
		Util::Point pmin = params._queryRegions[0][0];
		Util::Point pmax = params._queryRegions[0][1];
		params_._queryRegions.push_back(params._queryRegions[0]);
		params_._visibilitiRegions.push_back(params._visibilitiRegions[0]);
		for (size_t e = 0; e < graph.edges.size(); e++)
		{
			Graphing::Edge e_ = graph.edges[e];
			double xmin = graph.nodes[e_._origin](0);
			double xmax = graph.nodes[e_._end](0);
			if (graph.nodes[e_._end](0) < xmin)
			{
				xmin = graph.nodes[e_._end](0);
				xmax = graph.nodes[e_._origin](0);
			}

			double zmin = graph.nodes[e_._origin](2);
			double zmax = graph.nodes[e_._end](2);
			if (graph.nodes[e_._end](2) < zmin)
			{
				zmin = graph.nodes[e_._end](2);
				zmax = graph.nodes[e_._origin](2);
			}
			// X axis first
			std::vector<size_t> nodes_;
			nodes_.push_back(e_._origin);
			nodes_.push_back(e_._end);
			Graphing::OptimizationParameter p(0, (pmin.x - xmin) + 1, (pmax.x - xmax) - 1, nodes_, Graphing::OptimizationParameter::ParameterType::Translation) ;
			p.setTranslation(Eigen::Vector3d(1.0, 0, 0));
			Graphing::OptimizationParameter p_(0, (pmin.z - zmin) + 1, (pmax.z - zmax) - 1, nodes_, Graphing::OptimizationParameter::ParameterType::Translation);
			p_.setTranslation(Eigen::Vector3d(0.0, 0, 1.0));
			params_.addParameter(p);
			params_.addParameter(p_);
		}
		params_.saveToFile("../results/mazeParams.xml");
	}


	std::cout << "Finished importing graph" << std::endl;

	if (true)
	{ // save default value
		world->clearWorld();
		SteerOpt::RoundRobinOptimization steerFunc(world, &steerSuite);
		steerFunc.setOptimizationParameters(params);
		steerFunc.setOptimiationConfiguration(optConfig);
		
		steerFunc.setGraph(graph);
		steerFunc.setMaxEvals(optConfig._max_fevals);

		steerFunc._gridBound1 = params._visibilitiRegions[0][0];
		steerFunc._gridBound2 = params._visibilitiRegions[0][1];
		steerFunc._gridNumX = (unsigned int)((steerFunc._gridBound2.x - steerFunc._gridBound1.x) * optConfig._grid_cells_pre_meter);
		steerFunc._gridNumZ = (unsigned int)((steerFunc._gridBound2.z - steerFunc._gridBound1.z) * optConfig._grid_cells_pre_meter);
		steerFunc._height = 0, steerFunc._meanDegree = 0;

		// this->_steerSuite->simulate(x, N);
		// std::cout << "Size of visibility graph regions: " << params._visibilitiRegions.size() << std::endl;
		// std::cout << "Size of _queryRegions: " << params._queryRegions.size() << std::endl;
		// std::cout << "Size of _refRegions: " << params._refRegions.size() << std::endl;
		steerFunc._steerSuite->init_visibilityGraph(steerFunc._gridBound1,
			steerFunc._gridBound2, steerFunc._gridNumX, steerFunc._gridNumZ, steerFunc._height);
		// this->_steerSuite->add_visibilityRegion(_regionBound1, _regionBound2);
		for (size_t r = 0; r < params._queryRegions.size(); r++)
		{
			steerFunc._steerSuite->add_queryRegion(params._queryRegions[r][0], params._queryRegions[r][1]);
			if (params._refRegions.size() == 0 )
			{
				steerFunc._steerSuite->add_refRegion(params._queryRegions[r][0], params._queryRegions[r][1]);
			}
		}
		for (size_t r = 0; r < params._refRegions.size(); r++)
		{
			steerFunc._steerSuite->add_refRegion(params._refRegions[r][0], params._refRegions[r][1]);
		}

		steerSuite.init(world);

		std::vector<double> best_x(params.size(), 0.0); // initialize to a bunch of zeros
		steerFunc.preprocessEnvironment(&best_x.front(), best_x.size());

		// TODO can't see this visability graph...

		double metric = steerFunc.calcMultiObjective(&best_x.front(), best_x.size());
		std::vector<double> metrics = steerFunc._calcMultiObjective(&best_x.front(), best_x.size());

		double mean_degre = metrics[0];
		std::cout << "Final: mean degree: " << mean_degre << std::endl;
		double area = metrics[3];
		std::cout << "Area of Minkowski intersection is: " << area << std::endl;
		double alignment = metrics[4];
		std::cout << "Alignment Penalty: " << alignment << std::endl;
		double wall_length = metrics[5];
		std::cout << "Wall length: " << wall_length << std::endl;
		std::vector<float> values = steerSuite.get_meanTreeProp();
		std::cout << "**** Mean Depth: " << metrics[1] << " mean Entropy: " << metrics[2] << std::endl;
		double diversity = 0;
		std::cout << "**** Diversity" << diversity << std::endl;


		LogObject logObject;

		// get_fvalue();

		logObject.addLogData((int)-1);
		logObject.addLogData(metric);
		logObject.addLogData(mean_degre);
		logObject.addLogData(values[0]);
		logObject.addLogData(values[1]);
		logObject.addLogData(diversity);
		logObject.addLogData(area);
		logObject.addLogData(alignment);
		logObject.addLogData(wall_length);
		_logger->writeLogObject(logObject);



		steerFunc.postprocessEnvironment(&best_x.front(), best_x.size());
		steerSuite.finish();
	}

	// TODO can't see this visability graph...
	if (optConfig._objectiveFunction == "Diversity" ||
			(optConfig._objectiveFunction == "MultiObjective"))
	{
		std::cout << "Starting RoundRobin Optimization" << std::endl;
		SteerOpt::RoundRobinOptimization steerFunc(world, &steerSuite);
		steerFunc.setOptimizationParameters(params);
		steerFunc.setOptimiationConfiguration(optConfig);
		if ( !optConfig._logFileName.empty() )
		{
			std::cout << "Setting up optimizatin data logging" << std::endl;
			steerFunc.logOptimization(optConfig._logFileName);
		}
		steerFunc.setGraph(graph);
		steerFunc.setMaxEvals(optConfig._max_fevals);

		steerFunc._gridBound1 = params._visibilitiRegions[0][0];
		steerFunc._gridBound2 = params._visibilitiRegions[0][1];
		steerFunc._gridNumX = (unsigned int)((steerFunc._gridBound2.x - steerFunc._gridBound1.x) * optConfig._grid_cells_pre_meter);
		steerFunc._gridNumZ = (unsigned int)((steerFunc._gridBound2.z - steerFunc._gridBound1.z) * optConfig._grid_cells_pre_meter);
		steerFunc._height = 0, steerFunc._meanDegree = 0;

		// this->_steerSuite->simulate(x, N);
		// std::cout << "Size of visibility graph regions: " << params._visibilitiRegions.size() << std::endl;
		// std::cout << "Size of _queryRegions: " << params._queryRegions.size() << std::endl;
		// std::cout << "Size of _refRegions: " << params._refRegions.size() << std::endl;
		steerFunc._steerSuite->init_visibilityGraph(steerFunc._gridBound1,
			steerFunc._gridBound2, steerFunc._gridNumX, steerFunc._gridNumZ, steerFunc._height);
		// this->_steerSuite->add_visibilityRegion(_regionBound1, _regionBound2);
		for (size_t r = 0; r < params._queryRegions.size(); r++)
		{
			steerFunc._steerSuite->add_queryRegion(params._queryRegions[r][0], params._queryRegions[r][1]);
			if (params._refRegions.size() == 0)
			{
				steerFunc._steerSuite->add_refRegion(params._queryRegions[r][0], params._queryRegions[r][1]);
			}
		}
		for (size_t r = 0; r < params._refRegions.size(); r++)
		{
			steerFunc._steerSuite->add_refRegion(params._refRegions[r][0], params._refRegions[r][1]);
		}

		std::cout << "Done setting up optimization, starting optimization" << std::endl;

		steerFunc.optimize();

		std::cout << "Optimization complete" << std::endl;
		// These seem to return the wrong answer. I will store my own instead.

		// std::vector<double> best_x = go._cmasols.get_best_seen_candidate().get_x();
		for (size_t m = 0; m < steerFunc.getNumMembers(); m++)
		{
			world->clearWorld();
			if (optConfig._renderResults)
			{
				steerSuite.initRender(world);
			}
			else
			{
				steerSuite.init(world);
			}
			
			std::vector<double> best_x = steerFunc.getBestMember(m);
			steerFunc.preprocessEnvironment(&best_x.front(), best_x.size());

			// TODO can't see this visability graph...

			double metric = steerFunc.calcMultiObjective(&best_x.front(), best_x.size());
			std::vector<double> metrics = steerFunc._calcMultiObjective(&best_x.front(), best_x.size());

			double mean_degre = metrics[0];
			std::cout << "Final: mean degree: " << mean_degre << std::endl;
			double area = metrics[3];
			std::cout << "Area of Minkowski intersection is: " << area << std::endl;
			double alignment = metrics[4];
			std::cout << "Alignment Penalty: " << alignment << std::endl;
			double wall_length = metrics[5];
			std::cout << "Wall length: " << wall_length << std::endl;
			std::vector<float> values = steerSuite.get_meanTreeProp();
			std::cout << "**** Mean Depth: " << metrics[1] << " mean Entropy: " << metrics[2] << std::endl;
			double diversity = steerFunc.getMemberDiversity(m);
			std::cout << "**** Diversity" << diversity << std::endl;


			LogObject logObject;

			// get_fvalue();

			logObject.addLogData((int)m);
			logObject.addLogData(metric);
			logObject.addLogData(mean_degre);
			logObject.addLogData(values[0]);
			logObject.addLogData(values[1]);
			logObject.addLogData(diversity);
			logObject.addLogData(area);
			logObject.addLogData(alignment);
			logObject.addLogData(wall_length);
			_logger->writeLogObject(logObject);

			if (optConfig._renderResults)
			{
				steerSuite.simulate();
				// SteerOpt::RockstarOptimize steerFunc(world, &steerSuite);
			}
			std::stringstream filename;
			filename << "../results/member" << m << ".graph";
			steerFunc._tmp_graph.saveToFile(filename.str());
			filename.str(std::string());
			filename << "../results/member" << m << ".svg";
			steerFunc._tmp_graph.clearMinkowskiSums();
			steerFunc._tmp_graph.saveSVGToFile(filename.str(), params);


			steerFunc.postprocessEnvironment(&best_x.front(), best_x.size());
			steerSuite.finish();
		}

		


	}
	else if (optConfig._objectiveFunction == "Hierarchical")
	{
		std::cout << "Starting Hierarchical Optimization" << std::endl;
		SteerOpt::HierarchicalOptimization steerFunc(world, &steerSuite);
		steerFunc.setOptimizationParameters(params);
		steerFunc.setOptimiationConfiguration(optConfig);
		if ( !optConfig._logFileName.empty() )
		{
			std::cout << "Setting up optimizatin data logging" << std::endl;
			steerFunc.logOptimization(optConfig._logFileName);
		}
		steerFunc.setGraph(graph);
		steerFunc.setMaxEvals(optConfig._max_fevals);

		steerFunc._gridBound1 = params._visibilitiRegions[0][0];
		steerFunc._gridBound2 = params._visibilitiRegions[0][1];
		steerFunc._gridNumX = (unsigned int)((steerFunc._gridBound2.x - steerFunc._gridBound1.x) * optConfig._grid_cells_pre_meter);
		steerFunc._gridNumZ = (unsigned int)((steerFunc._gridBound2.z - steerFunc._gridBound1.z) * optConfig._grid_cells_pre_meter);
		steerFunc._height = 0, steerFunc._meanDegree = 0;

		// this->_steerSuite->simulate(x, N);
		// std::cout << "Size of visibility graph regions: " << params._visibilitiRegions.size() << std::endl;
		// std::cout << "Size of _queryRegions: " << params._queryRegions.size() << std::endl;
		// std::cout << "Size of _refRegions: " << params._refRegions.size() << std::endl;
		steerFunc._steerSuite->init_visibilityGraph(steerFunc._gridBound1,
			steerFunc._gridBound2, steerFunc._gridNumX, steerFunc._gridNumZ, steerFunc._height);
		// this->_steerSuite->add_visibilityRegion(_regionBound1, _regionBound2);
		for (size_t r=0; r < params._queryRegions.size(); r++)
		{
			steerFunc._steerSuite->add_queryRegion(params._queryRegions[r][0], params._queryRegions[r][1]);
		}
		for (size_t r=0; r < params._refRegions.size(); r++)
		{
			steerFunc._steerSuite->add_refRegion(params._refRegions[r][0], params._refRegions[r][1]);
		}

		std::cout << "Done setting up optimization, starting optimization" << std::endl;

		steerFunc.optimize();

		std::cout << "Optimization complete" << std::endl;
		// These seem to return the wrong answer. I will store my own instead.

		// std::vector<double> best_x = go._cmasols.get_best_seen_candidate().get_x();
		for (size_t m = 0; m < steerFunc.getNumMembers(); m++)
		{
			if (optConfig._renderResults)
			{
				world->clearWorld();
				steerSuite.initRender(world);
			}
			else
			{
				steerSuite.init(world);
			}

			std::vector<double> best_x = steerFunc.getBestMember(m);
			steerFunc.preprocessEnvironment(&best_x.front(), best_x.size());

			// TODO can't see this visability graph...

			double metric = steerFunc.calcMultiObjective(&best_x.front(), best_x.size());
			std::vector<double> metrics = steerFunc._calcMultiObjective(&best_x.front(), best_x.size());

			double mean_degre = metrics[0];
			std::cout << "Final: mean degree: " << mean_degre << std::endl;
			double area = metrics[3];
			std::cout << "Area of Minkowski intersection is: " << area << std::endl;
			double alignment = metrics[4];
			std::cout << "Alignment Penalty: " << alignment << std::endl;
			double wall_length = metrics[5];
			std::cout << "Wall length: " << wall_length << std::endl;
			std::vector<float> values = steerSuite.get_meanTreeProp();
			std::cout << "**** Mean Depth: " << metrics[1] << " mean Entropy: " << metrics[2] << std::endl;
			double diversity = steerFunc.getMemberDiversity(m);
			std::cout << "**** Diversity" << diversity << std::endl;


			LogObject logObject;

			// get_fvalue();

			logObject.addLogData((int)m);
			logObject.addLogData(metric);
			logObject.addLogData(mean_degre);
			logObject.addLogData(values[0]);
			logObject.addLogData(values[1]);
			logObject.addLogData(diversity);
			logObject.addLogData(area);
			logObject.addLogData(alignment);
			logObject.addLogData(wall_length);
			_logger->writeLogObject(logObject);

			if (optConfig._renderResults)
			{
				steerSuite.simulate();
				// SteerOpt::RockstarOptimize steerFunc(world, &steerSuite);
			}
			std::stringstream filename;
			filename << "../results/member" << m << ".graph";
			steerFunc._tmp_graph.saveToFile(filename.str());
			filename.str(std::string());
			filename << "../results/member" << m << ".svg";
			steerFunc._tmp_graph.clearMinkowskiSums();
			steerFunc._tmp_graph.saveSVGToFile(filename.str(), params);


			steerFunc.postprocessEnvironment(&best_x.front(), best_x.size());
			steerSuite.finish();
		}

	}


	return EXIT_SUCCESS;
}

int optimizeSteeringAlgorithm(std::string configFileName, SteerOpt::OptimizationConfiguration optConfig)
{
	Graphing::Graph graph;
	SteerSuite::SimWorld * world = new SteerSuite::SimWorld();


	SteerSuite::SteerSuite steerSuite;
	steerSuite.setConfigFileName(configFileName);
	
	graph.importFromFile(optConfig._graphFileName);

	Graphing::OptimizationParameters params;
	params.loadFromFile(optConfig._parameterFileName);
	

	std::cout << "Finished importing graph" << std::endl;
	

	SteerOpt::CMAOptimize steerFunc(world, &steerSuite);
	steerFunc.setOptimizationParameters(params);
	steerFunc.setGraph(graph);
	steerFunc.setMaxEvals(optConfig._max_fevals);
	steerFunc._gridBound1 = Util::Point(-100, 0, -100);
	steerFunc._gridBound2 = Util::Point(100, 0, 100);
	steerFunc._gridNumX = 100, steerFunc._gridNumZ = 100;
	steerFunc._height = 0, steerFunc._meanDegree = 0;

	// this->_steerSuite->simulate(x, N);
	steerFunc._steerSuite->init_visibilityGraph(steerFunc._gridBound1,
		steerFunc._gridBound2, steerFunc._gridNumX, steerFunc._gridNumZ, steerFunc._height);
	// this->_steerSuite->add_visibilityRegion(_regionBound1, _regionBound2);
	steerFunc._steerSuite->add_queryRegion(Util::Point(7.0, 0, -2.0), Util::Point(10.0, 0, 3.0));
	steerFunc._steerSuite->add_refRegion(Util::Point(-10.0, 0, -10.0), Util::Point(-7.0, 0, -7.0));
	steerFunc._steerSuite->add_refRegion(Util::Point(-10.0, 0, 7.0), Util::Point(-7.0, 0, 10.0));

	std::cout << "Done setting up optimization, starting optimization" << std::endl;

	steerFunc.optimize(optConfig._objectiveFunction);

	std::cout << "Optimization complete" << std::endl;
	// These seem to return the wrong answer. I will store my own instead.

	if ( optConfig._renderResults )
	{
		// libcmaes::Candidate best_x = steerFunc._cmasols.best_candidate();
		// gp.pheno(best_candidate().get_x_dvec()).transpose()
		std::cout << "Best f value: " << steerFunc._cmasols.best_candidate().get_fvalue() << std::endl;
		std::cout << "Best parameters: " << steerFunc._best_x << std::endl;

		std::vector<double> best_x_(steerFunc._best_x.data(),
				steerFunc._best_x.data() + steerFunc._best_x.rows() * steerFunc._best_x.cols());

		// TODO can't see this visability graph...
		if (optConfig._objectiveFunction == "PLE")
		{
			steerSuite.initRender(world);
			SteerLib::Behaviour behave;
			for (size_t p = 0; p < params.size(); p++)
			{
				std::stringstream ss;
				ss << steerFunc._best_x(p,0);
				SteerLib::BehaviourParameter bParam(params.getParameter(p)._name, ss.str());
				behave.addParameter(bParam);
			}
			steerSuite.simulateSteeringAlgorithm(&best_x_.front(), best_x_.size(), behave);
			// SteerOpt::RockstarOptimize steerFunc(world, &steerSuite);

			steerSuite.finish();
		}
		else if (optConfig._objectiveFunction == "Degree")
		{
			// std::vector<double> best_x_ = best_x.get_x();
			Graphing::Graph tmp_graph;
			tmp_graph.initFrom(graph);
			for (size_t p = 0; p < best_x_.size(); p++)
			{
				std::cout << "Best x[" << p << "] = " << best_x_[p] << std::endl;
				for (size_t ns = 0; ns < params._parameters[p]._node_ids.size(); ns++)
				{	/// Assuing the dimensions are equal to the number of nodes
					// graph.nodes[params._parameters[p]._node_ids[ns]](params._parameters[p]._dimensions[ns]) += best_x[p];
					// graph.nodes[params._parameters[p]._node_ids[ns]](params._parameters[p]._dimensions[ns]) += go._best_x[p];
					Graphing::OptimizationParameter param = params._parameters[p];
					// Eigen::Vector3d pos = graph.nodes[params._parameters[p]._node_ids[ns]];
					Eigen::Vector3d pos_d = param.getUpdatedPos(best_x_[p], tmp_graph.nodes[params._parameters[p]._node_ids[ns]]);

					tmp_graph.nodes[params._parameters[p]._node_ids[ns]] = pos_d;
				}
			}
			world->clearWorld();
			for (size_t edge = 0; edge < tmp_graph.edges.size(); edge++)
			{
				Eigen::Vector3d cp_ = (tmp_graph.nodes.at(tmp_graph.edges[edge]._origin) + tmp_graph.nodes.at(tmp_graph.edges[edge]._end)) / 2.0;
				Util::Point cp(cp_(0), cp_(1), cp_(2));
				Eigen::Vector3d edge_ = (tmp_graph.nodes.at(tmp_graph.edges[edge]._end) - tmp_graph.nodes.at(tmp_graph.edges[edge]._origin)).normalized();
				double length = (tmp_graph.nodes.at(tmp_graph.edges[edge]._origin) - tmp_graph.nodes.at(tmp_graph.edges[edge]._end)).norm();
				double theta = std::atan2(edge_(2), edge_(0)) * 180.0 / M_PI;
				world->addOrientedObstacle(cp, length, 0.1, 0.0, 1.0, -theta);
			}

			// TODO can't see this visability graph...
			steerSuite.initRender(world);
			steerSuite.setup_visibilityGraph();
			double mean_degre = (double)steerSuite.get_meanDegree();
			std::cout << "Final: mean degree: " << mean_degre << std::endl;

			steerSuite.simulate();
			// SteerOpt::RockstarOptimize steerFunc(world, &steerSuite);

			steerSuite.finish();
		}
	}

	return EXIT_SUCCESS;
}

int main(int argc, char **argv)
{
	// save the original cerr streambuf, so that we can restore it on an exception.
    std::cout << "Starting steeropt" << std::endl;
    
    std::string configFileName;
    
	

	if ( argc == 2 )
	{ // architectural optimization
		configFileName = argv[1];
		std::cout << "Running parsing optimization configuration" << std::endl;		
	}
	else
	{
		std::cout << "Improper calling format" << std::endl;
		std::cout << "Format should be: <exe> <config_file>" << std::endl;
		std::cout << "Example" << std::endl;
		std::cout << "./steeroptRun.exe ../config_win.xml" << std::endl;
	}

	SteerSuite::SteerSuite steerSuite_;
	steerSuite_.setConfigFileName(configFileName);
	SteerSuite::SimWorld * world = new SteerSuite::SimWorld();
	steerSuite_.init(world);
	const SteerLib::OptionDictionary options = steerSuite_.getModuleOptions("steeropt");
	// parse command line options
	SteerOpt::OptimizationConfiguration optConfig(options);
	steerSuite_.finish();
	delete world;


	if (optConfig._objectiveFunction == "Diversity" ||
			(optConfig._objectiveFunction == "MultiObjective") ||
			(optConfig._objectiveFunction == "Hierarchical"))
	{
		diversityOptimizeDegreeFromFile(configFileName, optConfig);
	}
	else if (optConfig._objectiveFunction == "PLE" ||
			optConfig._objectiveFunction == "Degree")
	{
		optimizeSteeringAlgorithm(configFileName, optConfig);
	}
	else
	{
		std::cout << "Unexpected optimization algorithm: " << optConfig._objectiveFunction << " not supported" << std::endl;
		return 0;
	}

}
