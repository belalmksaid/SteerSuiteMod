/*
 * HierarchicalOptimization.cpp
 *
 *  Created on: Apr 24, 2016
 *      Author: Glen
 */

#include "HierarchicalOptimization.h"

namespace SteerOpt {

HierarchicalOptimization::HierarchicalOptimization(SteerSuite::SimWorld * world, SteerSuite::SteerSuite * steersuite)
: RoundRobinOptimization( world, steersuite)
{
	// TODO Auto-generated constructor stub

}

HierarchicalOptimization::~HierarchicalOptimization() {
	// TODO Auto-generated destructor stub
}


void HierarchicalOptimization::optimize()
{
	const int dim = this->_params.size(); // problem dimensions.
	std::vector<double> x0(dim, 0.0);
	_metric_thresholds.push_back(this->_optConfig._metric_threshold);


	this->_steerSuite->init(this->_world);
	this->preprocessEnvironment(&x0.front(), dim);
	double defaultValue = this->calcMultiObjective(&x0.front(), dim); // minimization
	this->postprocessEnvironment(&x0.front(), dim);
	this->_steerSuite->finish();
	std::stringstream ss;
	std::string fName = this->_optConfig._logFileName;
	ss << fName << "0.csv";
	this->_optConfig._logFileName = ss.str();
	if (!this->_optConfig._logFileName.empty())
	{
		std::cout << "Setting up optimizatin data logging" << std::endl;
		logOptimization(ss.str());
	}
	ss.str(std::string());

	CMAOptimize::optimize("MultiObjective");

	libcmaes::Candidate best_x = this->_cmasols.get_best_seen_candidate();
	double optimalValue = -best_x.get_fvalue(); // It was a minimization
	double boundaryValue = ((optimalValue - defaultValue)*_metric_thresholds[0]) + defaultValue;

	std::cout << "Done first objective optimization" << std::endl;
	std::cout << "default value: " << defaultValue << " optimal value: " << optimalValue << 
		" boundary value: " << boundaryValue << std::endl;

	this->_objectiveWeights.push_back(boundaryValue);
	
	ss << fName << "Diversity.csv";
	this->_optConfig._logFileName = ss.str();
	if (!this->_optConfig._logFileName.empty())
	{
		std::cout << "Setting up optimizatin data logging" << std::endl;
		logOptimization(ss.str());
	}
	RoundRobinOptimization::optimize();
}


std::vector<double>  HierarchicalOptimization::getBestMember(size_t m)
{
	return _solutions[m];
}

size_t HierarchicalOptimization::getNumMembers()
{
	return _solutions.size();
}

void HierarchicalOptimization::logOptimization(std::string fileName)
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


} /* namespace SteerOpt */
