/*
 * RoundRobinOptimization.cpp
 *
 *  Created on: Jan 16, 2016
 *      Author: Glen
 */

#include "RoundRobinOptimization.h"
// #include "src/cmaes.h"
#include "CMAOptimizePrivate.h"
#include "src/RoundRobinCMA.h"
#include "src/RoundRobinCMA.cpp"

namespace SteerOpt {

RoundRobinOptimization::RoundRobinOptimization(SteerSuite::SimWorld * world, SteerSuite::SteerSuite * steersuite) 
:CMAOptimize( world, steersuite)
{
	// TODO Auto-generated constructor stub

}

RoundRobinOptimization::~RoundRobinOptimization() {
	// TODO Auto-generated destructor stub
}


void RoundRobinOptimization::optimize()
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
	double sigma = 0.32;
	for (size_t p = 0; p < dim; p++)
	{
		x0.push_back(this->_params._parameters.at(p)._x0);
		lbounds[p] = this->_params._parameters.at(p)._lb;
		ubounds[p] = this->_params._parameters.at(p)._ub;
		sigma_.push_back((ubounds[p] - lbounds[p])*sigma);
	}
	std::cout.setf(std::ios_base::scientific);
	std::cout.precision(5);
	int offsrping = 10; // lambda

	// libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, linScalingStrategy> gp(lbounds, ubounds, dim); // maybe this uses sigma somehow
	// libcmaes::CMAParameters< libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, linScalingStrategy>> cmaparams(dim, &x0.front(), sigma, -1, 0, gp);
	libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, libcmaes::linScalingStrategy> gp(lbounds, ubounds, dim); // maybe this uses sigma somehow
	libcmaes::CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, libcmaes::linScalingStrategy> > cmaparams(dim, &x0.front(), sigma, offsrping, 21, gp);
	cmaparams.set_algo(aCMAES);
	cmaparams.set_max_iter(_max_evals);
	// cmaparams.set_maximize(true);
	cmaparams.set_seed(21);
	cmaparams.set_ftolerance(this->_optConfig._ftolerance);
	cmaparams.set_xtolerance(this->_optConfig._xtolerance);
	std::cout << "ftol: " << this->_optConfig._ftolerance << std::endl;
	std::cout << "xtol: " << this->_optConfig._xtolerance << std::endl;
	
	// cmaparams.set_restarts(3);
	std::cout << "Starting a steerOpt Optimization" << std::endl;
	// GraphOptimizePrivate * cmaOP = new GraphOptimizePrivate(this->_graph, this->_params);
	SteerOpt::CMAOptimizePrivate * cmaOP = new SteerOpt::CMAOptimizePrivate(this->_world, this->_steerSuite);
	cmaOP->setOptimizationParameters(this->_params);
	cmaOP->setOptimiationConfiguration(this->_optConfig);
	cmaOP->setGraph(this->_graph);
	cmaOP->_gridBound1 = this->_gridBound1;
	cmaOP->_gridBound2 = this->_gridBound2;
	cmaOP->_gridNumX = this->_gridNumX;
	cmaOP->_gridNumZ = this->_gridNumZ;
	cmaOP->_height = this->_height;
	cmaOP->_meanDegree = this->_meanDegree;
	cmaOP->_steerSuite = this->_steerSuite;
	this->_steerSuite->init(this->_world);
	// this->_steerSuite->initRender(this->_world);
	//ESOptimizer<CMAStrategy<CovarianceUpdate>,CMAParameters<>> optim(fsphere,cmaparams);
	// RoundRobinCMA<libcmaes::GenoPheno<libcmaes::NoBoundStrategy, NoScalingStrategy>> optim(5, cmaOP->evalDegree, cmaparams);
	libcmaes::FitFunc func = cmaOP->evalManyObjective;
	if (this->_objectiveWeights.size() > 0)
	{
		cmaOP->_degree_threshold = this->_objectiveWeights[0];
		func = cmaOP->hierarchicalThreshold;
	}
	RoundRobinCMA<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy, libcmaes::linScalingStrategy>> optim(5, func, cmaparams);
	// RoundRobinCMA optim(5, cmaOP->evalDegree, lbounds, ubounds, dim, x0, _max_evals);

	optim._kdmin = this->_optConfig._kdmin;
	optim._diversity_weight = this->_optConfig._diversity_weight;
	// optim._minimize_or_maximize = this->_optConfig._minimize_or_maximize;

	while (!optim.stop())
	{
		// dMat candidates = optim.ask();

		size_t member = optim._current_member;
		dMat candidates = optim.ask();
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
		const dMat &phenocandidates = dMat(0, 0);
		optim.eval(candidates, phenocandidates);
		if (this->_logger && (optim._round > 1) )
		{
			std::cout << "logging" << std::endl;
			LogObject logObject;
			float f_val = optim.getSolution(member).best_candidate().get_fvalue();
// get_fvalue();
			float best_f = optim.getSolution(member).get_best_seen_candidate().get_fvalue();
			logObject.addLogData((int)optim._round);
			logObject.addLogData(f_val);
			logObject.addLogData(best_f);
			logObject.addLogData(optim._metric);
			logObject.addLogData(optim._diversity);
			logObject.addLogData(optim._diversityMin);

			// std::cout << "Best x: " << optim.getSolution(member).best_candidate().get_x_dvec().transpose() << std::endl;


			this->logData(logObject);
		}
		optim.tell();
		// optim.inc_iter(); // important step: signals next iteration.
	}
	this->_steerSuite->finish();
	_solutions = optim.get_solutions();
	this->_diversity.resize(_solutions.size());
	for (size_t i = 0; i < _solutions.size(); i++)
	{
		this->_diversity[i] = this->_optConfig._diversity_weight*(optim.getMemberDiversity(i));
	}

	// std::cout << "best solution: " << this->_cmasols << std::endl;
	// std::cout << "best parameters: " << this->_cmasols.best_candidate().get_x_dvec() << std::endl;
	// std::cout << "best function value: " << this->_cmasols.best_candidate().get_fvalue() << std::endl;
	// std::cout << "optimization took " << this->_cmasols.elapsed_time() / 1000.0 << " seconds\n";
	int k = 0;
	// std::cout << "best parameters: " << gp.pheno(this->_cmasols.best_candidate().get_x_dvec()) << std::endl;
	std::cout << "best solution: " << optim.getSolution(k) << std::endl;
	std::cout << "best parameters: " << gp.pheno(optim.getSolution(k).best_candidate().get_x_dvec()) << std::endl;
	std::cout << "best function value: " << optim.getSolution(k).best_candidate().get_fvalue() << std::endl;
	std::cout << "optimization took " << optim._time / 1000.0 << " seconds\n";
	delete lbounds;
	delete ubounds;
	delete cmaOP;
}


std::vector<double>  RoundRobinOptimization::getBestMember(size_t m)
{
	return _solutions[m];
}

size_t RoundRobinOptimization::getNumMembers()
{
	return _solutions.size();
}

double RoundRobinOptimization::getMemberDiversity(size_t m)
{
	return this->_diversity[m];
}

void RoundRobinOptimization::logOptimization(std::string fileName)
{
	_logger = LogManager::getInstance()->createLogger(fileName, LoggerType::BASIC_WRITE);
	_logger->addDataField("iteration", DataType::Integer);
	_logger->addDataField("f_val", DataType::Float);
	_logger->addDataField("best_f_so_far", DataType::Float);
	_logger->addDataField("metric", DataType::Float);
	_logger->addDataField("diversity", DataType::Float);
	_logger->addDataField("diversityMin", DataType::Float);
	_logger->addDataField("clearence", DataType::Float);
	_logger->addDataField("alignment", DataType::Float);

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

} /* namespace SteerOpt */
