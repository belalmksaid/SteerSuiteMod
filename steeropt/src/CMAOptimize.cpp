/*
 * CMAOptimize.cpp
 *
 *  Created on: Oct 28, 2015
 *      Author: Glen
 */

#include "CMAOptimize.h"
#include "CMAOptimizePrivate.h"
#include "src/cmaes.h"
#include <iostream>

namespace SteerOpt {

	CMAOptimize::CMAOptimize(SteerSuite::SimWorld * world, SteerSuite::SteerSuite * steersuite) :
		SteerSimOptimize(world, steersuite),
	_gridBound1(-100, 0, -100),
	_gridBound2(100, 0, 100)
{
	_gridNumX = 100, _gridNumZ = 100;
	_height = 0.0, _meanDegree = 0.0;

}

CMAOptimize::~CMAOptimize() {
	// TODO Auto-generated destructor stub
}

void CMAOptimize::preprocessEnvironment(const double * x, int N)
{
	// GraphFunction gf;
	Graphing::Graph tmp_graph;
	tmp_graph.initFrom(this->_graph); // duplicate
									  // std::cout << "Length of graph: " << gf.graphLength(tmp_graph) << std::endl;
	std::set<size_t> nodes_;
	for (size_t p = 0; p < N; p++)
	{
		// std::cout << "x[" << p << "] = " << x[p] << std::endl;
		for (size_t ns = 0; ns < this->_params._parameters[p]._node_ids.size(); ns++)
		{	/// Assuing the dimensions are equal to the number of nodes
			Graphing::OptimizationParameter param = this->_params._parameters[p];
			Eigen::Vector3d pos = tmp_graph.nodes[param._node_ids[ns]];
			tmp_graph.nodes[param._node_ids[ns]] = param.getUpdatedPos(x[p], pos);
			nodes_.insert(param._node_ids[ns]);
		}
	}

	this->_tmp_graph = tmp_graph;
	this->_tmp_graph.updateMinkowskiSums(nodes_, this->_optConfig._clearence_distance);
	// this->_tmp_graph.computeMinkowskiSums(this->_optConfig._clearence_distance);

	/*
	std::vector<double> best_x = go._cmasols.best_candidate().get_x();

	// std::vector<double> best_x = go._cmasols.get_best_seen_candidate().get_x();
	for (size_t p = 0; p < best_x.size(); p++)
	{
	std::cout << "Best x[" << p << "] = " << best_x[p] << std::endl;
	for (size_t ns = 0; ns < params._parameters[p]._node_ids.size(); ns++)
	{	/// Assuing the dimensions are equal to the number of nodes
	// graph.nodes[params._parameters[p]._node_ids[ns]](params._parameters[p]._dimensions[ns]) += best_x[p];
	// graph.nodes[params._parameters[p]._node_ids[ns]](params._parameters[p]._dimensions[ns]) += go._best_x[p];
	Graphing::OptimizationParameter param = params._parameters[p];
	Eigen::Vector3d pos = graph.nodes[params._parameters[p]._node_ids[ns]];
	graph.nodes[params._parameters[p]._node_ids[ns]] += param.getUpdatedPos(pos, go._best_x[p]);
	}
	}
	*/

	this->_world->clearWorld();
	this->_steerSuite->resetSimulation();
	for (size_t edge = 0; edge < tmp_graph.edges.size(); edge++)
	{
		Eigen::Vector3d cp_ = (tmp_graph.nodes.at(tmp_graph.edges[edge]._origin) + tmp_graph.nodes.at(tmp_graph.edges[edge]._end)) / 2.0;
		Util::Point cp(cp_(0), cp_(1), cp_(2)); // center point
		// std::cout << "New centre point of obstacle: " << cp << std::endl;
		Eigen::Vector3d edge_ = (tmp_graph.nodes.at(tmp_graph.edges[edge]._end) - tmp_graph.nodes.at(tmp_graph.edges[edge]._origin)).normalized();
		double length = (tmp_graph.nodes.at(tmp_graph.edges[edge]._origin) - tmp_graph.nodes.at(tmp_graph.edges[edge]._end)).norm();
		double theta = std::atan2(edge_(2), edge_(0)) * 180.0 / M_PI;
		this->_world->addOrientedObstacle(cp, length, 0.1, 0.0, 1.0, -theta);
	}
	// std::cout << "Number of obstacles in environment: " << this->_steerSuite->_driver->getEngine()->getObstacles().size() << " vs " << this->_steerSuite->_world->getObstacles().size() <<  std::endl;
	// std::cout << "Number of obstacles in environment: " << this->_steerSuite->_world->getObstacles().size() <<  std::endl;
	this->_steerSuite->restartSimulation(this->_world);

	// this->_steerSuite->simulate(x, N);
	// this->_steerSuite->init_visibilityGraph(_gridBound1,
	// _gridBound2, _gridNumX, _gridNumZ, _height);
	// this->_steerSuite->add_visibilityRegion(_regionBound1, _regionBound2);
	// _steerSuite->add_queryRegion(Util::Point(7.0, 0, -2.0), Util::Point(10.0, 0, 3.0));
	// _steerSuite->add_refRegion(Util::Point(-10.0, 0, -10.0), Util::Point(-7.0, 0, -7.0));
	// _steerSuite->add_refRegion(Util::Point(-10.0, 0, 7.0), Util::Point(-7.0, 0, 10.0));
	if ( (this->_optConfig._entropy_weight > 0.0) ||
				( this->_optConfig._depth_weight < 0.0) ||
				(this->_optConfig._degree_weight > 0.0))
	{
		this->_steerSuite->setup_visibilityGraph();
	}
}


void CMAOptimize::postprocessEnvironment(const double * x, int N)
{

	// this->_steerSuite->finish();
	this->_world->clearWorld();
}

double CMAOptimize::calcMultiObjectiveObjectives(const double * x, int N)
{
	std::vector<double> in = _calcMultiObjectiveObjectives(x, N);
	double sum = 0;
	for (size_t i = 0; i < in.size(); i++)
	{
		sum += in[i];
	}

	return sum;
}

double CMAOptimize::calcMultiObjectivePenalties(const double * x, int N)
{
	std::vector<double> in = _calcMultiObjectivePenalties(x, N);
	double sum = 0;
	for (size_t i = 0; i < in.size(); i++)
	{
		sum += in[i];
	}

	return sum;
}

std::vector<double> CMAOptimize::_calcMultiObjective(const double * x, int N)
{

	// std::vector<double> out;
	std::vector<double> outObj = this->_calcMultiObjectiveObjectives(x, N);
	std::vector<double> outPen = this->_calcMultiObjectivePenalties(x, N);

	outObj.insert(outObj.end(), outPen.begin(), outPen.end());
	return outObj;
}

std::vector<double> CMAOptimize::_calcMultiObjectiveObjectives(const double * x, int N)
{
	double epsilon = 0.0001;
	double degree = 0.0;
	double entropy = 0.0;
	double depth = 0.0;
	double clearence = 0.0;
	double alignment = 0.0;
	bool print_metrics = false;
	if (this->_optConfig._degree_weight > epsilon)
	{
		degree = this->calcDegree(x, N);
		if (print_metrics)
		{
			std::cout << "**** Mean Degree: " << degree << std::endl;
		}
	}
	if ((this->_optConfig._entropy_weight > epsilon) ||
		(this->_optConfig._depth_weight < epsilon))
	{
		std::vector<float> values = this->calcDepthAndEntropy(x, N);
		depth = values[0];
		entropy = values[1];
		if (print_metrics)
		{
			std::cout << "**** Mean Depth: " << values[0] << " mean Entropy: " << values[1] << std::endl;
		}
	}

	std::vector<double> out;
	out.push_back(degree * this->_optConfig._degree_weight);
	out.push_back(depth * this->_optConfig._depth_weight);
	out.push_back(entropy * this->_optConfig._entropy_weight);

	return out;
}

std::vector<double> CMAOptimize::_calcMultiObjectivePenalties(const double * x, int N)
{
	double epsilon = 0.0001;
	double degree = 0.0;
	double entropy = 0.0;
	double depth = 0.0;
	double clearence = 0.0;
	double alignment = 0.0;
	bool print_metrics = false;
	
	if (this->_optConfig._clearance_weight > epsilon)
	{

		// double area = this->_steerSuite->computeIntersections(false);
		// area = ((area + 1) * (area + 1)) - 1;
		// std::cout << "Area of Minkowski intersection is: " << area << std::endl;
		double area = this->_tmp_graph.computeMinkowskiSumIntersectionArea();
		area = ((area + 1) * (area + 1)) - 1;
		if (print_metrics)
		{
			std::cout << "Area of Minkowski intersection is: " << area << std::endl;
		}
		clearence = area;
	}
	if (this->_optConfig._alignment_weight > epsilon)
	{
		alignment = this->_tmp_graph.alignmentPenalty();
		if (print_metrics)
		{
			std::cout << "Alignment Penalty: " << alignment << std::endl;
		}
	}

	std::vector<double> out;
	out.push_back(clearence * -this->_optConfig._clearance_weight);
	out.push_back(alignment * -this->_optConfig._alignment_weight);
	// Add wall length objectives
	//
	if (this->_optConfig._wall_length_weight > epsilon)
	{
		double lengthSum = this->_tmp_graph.sumWallLengths();
		double originallength = this->_graph.sumWallLengths();
		if (lengthSum < originallength)
		{
			out.push_back((originallength - lengthSum + 1) * (originallength - lengthSum + 1) * -this->_optConfig._wall_length_weight);
		}
		else
		{
			out.push_back(0);
		}
	}
	else
	{
		out.push_back(0);
	}

	return out;
}

double CMAOptimize::calcMultiObjective(const double * x, int N)
{
	std::vector<double> in = _calcMultiObjective(x, N);
	double sum = 0;
	for (size_t i = 0; i < in.size(); i++)
	{
		sum += in[i];
	}
	
	return sum;
}

std::vector<float> CMAOptimize::calcDepthAndEntropy(const double * x, int N)
{
	std::vector<float> values = this->_steerSuite->get_meanTreeProp();
	return values;
}

double CMAOptimize::calcDegree(const double * x, int N)
{
	
	double mean_degre = (double) this->_steerSuite->get_meanDegree();
	return mean_degre;
	
}

void CMAOptimize::optimize(std::string objective)
{
	try
	{
		//
		// allocate and use the engine driver
		//
		const int dim = this->_params.size(); // problem dimensions.
		double * lbounds = new double[dim];
		double * ubounds = new double[dim]; // arrays for lower and upper parameter bounds, respectively
											// (2.09597,0,3.88671)

		std::vector<double> x0; // (dim, 7.0);
		std::vector<double> sigma_; // (dim, 7.0);
		double sigma = 0.3;
		for (size_t p = 0; p < dim; p++)
		{
			x0.push_back(this->_params._parameters.at(p)._x0);
			lbounds[p] = this->_params._parameters.at(p)._lb;
			ubounds[p] = this->_params._parameters.at(p)._ub;
			sigma_.push_back((ubounds[p] - lbounds[p])*sigma);
		}
		std::cout.setf(std::ios_base::scientific);
		std::cout.precision(10);

		
		libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, libcmaes::linScalingStrategy> gp(lbounds, ubounds, dim); // maybe this uses sigma somehow
		libcmaes::CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, libcmaes::linScalingStrategy> > cmaparams(dim, &x0.front(), sigma, -1, 0, gp);
		cmaparams.set_algo(aCMAES);
		cmaparams.set_max_iter(_max_evals);
		if (this->_optConfig._minimize_or_maximize > 0.0)
		{
			// cmaparams.set_maximize(true);
		}
		cmaparams.set_seed(21);
		// cmaparams.set
		cmaparams.set_ftolerance(this->_optConfig._ftolerance);
		cmaparams.set_xtolerance(this->_optConfig._xtolerance);
		
		// cmaparams.set_restarts(3);
		std::cout << "Starting a steerOpt Optimization" << std::endl;
		// GraphOptimizePrivate * cmaOP = new GraphOptimizePrivate(this->_graph, this->_params);
		CMAOptimizePrivate * cmaOP = new CMAOptimizePrivate(this->_world, this->_steerSuite);
		cmaOP->setOptimizationParameters(this->_params);
		cmaOP->setGraph(this->_graph);
		cmaOP->setOptimiationConfiguration(this->_optConfig);
		

		if ( objective == "PLE" )
		{

			this->_cmasols = libcmaes::cmaes<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, libcmaes::linScalingStrategy> >(cmaOP->evalPLE, cmaparams);
		}
		else if ( objective == "Degree" )
		{
			cmaOP->_gridBound1 = this->_gridBound1;
			cmaOP->_gridBound2 = this->_gridBound2;
			cmaOP->_gridNumX = this->_gridNumX;
			cmaOP->_gridNumZ = this->_gridNumZ;
			cmaOP->_height = this->_height;
			cmaOP->_meanDegree = this->_meanDegree;
			cmaOP->_steerSuite = this->_steerSuite;
			this->_steerSuite->init(this->_world);
			this->_cmasols = libcmaes::cmaes<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, libcmaes::linScalingStrategy> >(cmaOP->evalDegree, cmaparams);
			this->_steerSuite->finish();
		}
		else if ( objective == "MultiObjective" )
		{
			cmaOP->_gridBound1 = this->_gridBound1;
			cmaOP->_gridBound2 = this->_gridBound2;
			cmaOP->_gridNumX = this->_gridNumX;
			cmaOP->_gridNumZ = this->_gridNumZ;
			cmaOP->_height = this->_height;
			cmaOP->_meanDegree = this->_meanDegree;
			cmaOP->_steerSuite = this->_steerSuite;
			this->_steerSuite->init(this->_world);
			// this->_steerSuite->initRender(this->_world);
			// libcmaes::ESOStrategy<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy>> optim(cmaOP->evalManyObjective, cmaparams);
			// libcmaes::ESOStrategy<libcmaes::CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, libcmaes::linScalingStrategy>>, libcmaes::CMASolutions, libcmaes::CMAStopCriteria<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, libcmaes::linScalingStrategy>> > optim(cmaOP->evalManyObjective, cmaparams);
			// libcmaes::ESOStrategy<libcmaes::CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, libcmaes::linScalingStrategy>>, libcmaes::CMASolutions, libcmaes::CMAStopCriteria<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, libcmaes::linScalingStrategy>> > * optim = new libcmaes::ESOStrategy<libcmaes::CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, libcmaes::linScalingStrategy>>, libcmaes::CMASolutions, libcmaes::CMAStopCriteria<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, libcmaes::linScalingStrategy> > >(cmaOP->evalManyObjective, cmaparams);
			// libcmaes::ESOptimizer<libcmaes::CMAStrategy<libcmaes::CovarianceUpdate>, libcmaes::CMAParameters<libcmaes::pwqBoundStrategy>> optim(cmaOP->evalManyObjective, cmaparams);
			libcmaes::CMAStrategy< libcmaes::ACovarianceUpdate, libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, libcmaes::linScalingStrategy> > optim(cmaOP->evalManyObjective, cmaparams);

			int round = 0;
			while (round < _max_evals) // (!optim.stop())
			{
				// dMat candidates = optim.ask();

				dMat candidates = optim.ask();
				const dMat phenocandidates = optim.get_parameters().get_gp().pheno(candidates);

				/*
				for (size_t r = 0; r < candidates.cols(); r++)
				{ // Check to make sure the candidates have connected regions

				const double *x;
				x = candidates.col(r).data();

				this->preprocessEnvironment(x, candidates.rows());
				bool connected = this->_steerSuite->check_connection();
				this->postprocessEnvironment(x, candidates.rows());
				if (!connected) // If not connected
				{ // get new candidate
				std::cout << "Found an unconnected candidate " << candidates.col(r) << std::endl;
				candidates.col(r) = optim.ask().col(r);
				r--;
				}

				}
				*/

				optim.eval(candidates, phenocandidates);
				optim.tell();
				if (this->_logger)
				{
					std::cout << "logging" << std::endl;
					LogObject logObject;
					// optim.get_solutions();
					float f_val = optim.get_solutions().best_candidate().get_fvalue();
					dVec x__ = optim.get_parameters().get_gp().pheno(optim.get_solutions().best_candidate().get_x_dvec());
					std::vector<double> x___ = optim.get_solutions().best_candidate().get_x();
					const double * x = x__.data();
					std::vector<double> x_(x__.data(), x__.data() + x__.rows() * x__.cols());
					this->preprocessEnvironment(x_);
					std::vector<double> metric_values = _calcMultiObjective(x_);

					if (_optConfig._saveIterationData)
					{
						std::stringstream ss;
						ss << "round" << round << ".svg";
						this->_tmp_graph.saveSVGToFile(ss.str(), this->_params);
						ss.str(std::string());
						ss << "round" << round << ".graph";
						this->_tmp_graph.saveToFile(ss.str());
					}

					this->postprocessEnvironment(x_);
					// get_fvalue();
					float best_f = optim.get_solutions().get_best_seen_candidate().get_fvalue();
					logObject.addLogData(round);
					logObject.addLogData(f_val);
					logObject.addLogData(best_f);
					logObject.addLogData(metric_values[0]);
					logObject.addLogData(metric_values[1]);
					logObject.addLogData(metric_values[2]);
					logObject.addLogData(metric_values[3]);
					logObject.addLogData(metric_values[4]);
					logObject.addLogData(metric_values[5]);

					// std::cout << "Best x: " << optim.getSolution(member).best_candidate().get_x_dvec().transpose() << std::endl;


					this->logData(logObject);
					round++;
				}

				// optim.inc_iter(); // important step: signals next iteration.
			}
			// this->_steerSuite->finish();
			this->_cmasols = optim.get_solutions();

			// this->_cmasols = libcmaes::cmaes<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy> >(cmaOP->evalManyObjective, cmaparams);
			this->_steerSuite->finish();
			// delete optim;
		}
		// gp.pheno(best_candidate().get_x_dvec()).transpose()
		std::cout << "best solution: " << this->_cmasols << std::endl;
		std::cout << "best parameters: " << gp.pheno(this->_cmasols.best_candidate().get_x_dvec()) << std::endl;
		this->_best_x = gp.pheno(this->_cmasols.best_candidate().get_x_dvec());
		std::cout << "best function value: " << this->_cmasols.best_candidate().get_fvalue() << std::endl;
		std::cout << "optimization took " << this->_cmasols.elapsed_time() / 1000.0 << " seconds\n";
		delete lbounds;
		delete ubounds;
		delete cmaOP;
	}
	catch (std::exception &e)
	{

		std::cerr << "\nERROR: exception caught in main:\n" << e.what() << "\n";

		// there is a chance that cerr was re-directed.  If this is true, then also echo
		// the error to the original cerr.
	}
}

void CMAOptimize::logOptimization(std::string fileName)
{
	_logger = LogManager::getInstance()->createLogger(fileName, LoggerType::BASIC_WRITE);
	_logger->addDataField("iteration", DataType::Integer);
	_logger->addDataField("f_val", DataType::Float);
	_logger->addDataField("best_f_so_far", DataType::Float);
	_logger->addDataField("degree", DataType::Float);
	_logger->addDataField("depth", DataType::Float);
	_logger->addDataField("entropy", DataType::Float);
	_logger->addDataField("clearence", DataType::Float);
	_logger->addDataField("alignment", DataType::Float);
	_logger->addDataField("wall_length", DataType::Float);

	if (!fileName.empty())
	{

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
	}
}

/*
void CMAOptimize::optimize()
{
	try
	{
		//
		// allocate and use the engine driver
		//
		const int dim = 2; // problem dimensions.
		double lbounds[dim];
		double ubounds[dim]; // arrays for lower and upper parameter bounds, respectively
		// (2.09597,0,3.88671)

		std::vector<double> x0; // (dim, 7.0);
		x0.push_back(3.26924);
		x0.push_back(0.98588);
		// (3.26924, 0, 0.98588)
		lbounds[0] = x0[0] -1;
		ubounds[0] = x0[0] +1;
		lbounds[1] = x0[1] -1;
		ubounds[1] = x0[1] +1;
		std::cout.setf( std::ios_base::scientific );
		std::cout.precision( 10 );


		double sigma = 0.13;
		libcmaes::GenoPheno<libcmaes::pwqBoundStrategy> gp(lbounds,ubounds,dim);
		libcmaes::CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy> > cmaparams(x0, sigma, -1, 0, gp);
		cmaparams.set_algo(aCMAES);
		std::cout << "Starting a steerOpt Optimization" << std::endl;
		CMAOptimizePrivate * cmaOP = new CMAOptimizePrivate(this->_world, this->_steerSuite);


		libcmaes::CMASolutions cmasols = libcmaes::cmaes<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy> >(cmaOP->eval, cmaparams);
		std::cout << "best solution: " << cmasols << std::endl;
		std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0 << " seconds\n";
	}
	catch (std::exception &e) {

		std::cerr << "\nERROR: exception caught in main:\n" << e.what() << "\n";

		// there is a chance that cerr was re-directed.  If this is true, then also echo
		// the error to the original cerr.
	}
}
*/

} /* namespace SteerOpt */
