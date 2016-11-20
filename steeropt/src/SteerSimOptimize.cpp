
#include "../include/SteerSimOptimize.h"
#include "testcaseio/Behaviour.h"

namespace SteerOpt {

	SteerSimOptimize::SteerSimOptimize(SteerSuite::SimWorld * world, SteerSuite::SteerSuite * steersuite)
	{
		this->_steerSuite = steersuite;
		this->_world = world;
		_max_evals = 10;
		_logger = NULL;
	}

	void SteerSimOptimize::setOptimizationParameters(Graphing::OptimizationParameters params)
	{
		this->_params = params;
	}

	double SteerSimOptimize::simulate(const double* x, const int N)
	{

		this->_steerSuite->init(this->_world);
		// std::cout << this->_simulationOptions.moduleOptionsDatabase.at(0).at(0) << std::endl;
		this->_steerSuite->simulate(x, N);
		LogData * outData = this->_steerSuite->getSimData();
		if (outData->size() > 0)
		{
			DataItem di = outData->getLogDataAt(0)->getLogData(7);
			std::cout << "Field name: " << outData->getLogger()->getFieldName(7) << std::endl;
			std::cout << "Maybe PLE energy? " << di.floatData << std::endl;
			this->_steerSuite->finish();
			return di.floatData;
		}
		else
		{
			std::cout << "Error getting Log data from siulation" << std::endl;
			this->_steerSuite->finish();
			return 0.0f;
		}
	}

	double SteerSimOptimize::simulateSteeringAlgorithm(const double* x, const int N)
	{

		this->_steerSuite->init(this->_world);
		std::cout << "Simulation has been initiated" << std::endl;
		// std::cout << this->_simulationOptions.moduleOptionsDatabase.at(0).at(0) << std::endl;
		// Set the parameters of the steering algorithm
		SteerLib::Behaviour behave;
		for (size_t p=0; p < this->_params.size(); p++)
		{
			std::stringstream ss;
			ss << x[p];
			SteerLib::BehaviourParameter bParam(this->_params.getParameter(p)._name, ss.str());
			behave.addParameter(bParam);
		}

		this->_steerSuite->simulateSteeringAlgorithm(x, N, behave);

		LogData * outData = this->_steerSuite->getSimData();
		if (outData->size() > 0)
		{
			DataItem di = outData->getLogDataAt(0)->getLogData(7);
			std::cout << "Field name: " << outData->getLogger()->getFieldName(7) << std::endl;
			std::cout << "Maybe PLE energy? " << di.floatData << std::endl;
			this->_steerSuite->finish();
			return di.floatData;
		}
		else
		{
			std::cout << "Error getting Log data from siulation" << std::endl;
			this->_steerSuite->finish();
			return 0.0f;
		}
	}

	void SteerSimOptimize::logOptimization(std::string fileName)
	{
		_logger = LogManager::getInstance()->createLogger(fileName, LoggerType::BASIC_WRITE);
		_logger->addDataField("iteration", DataType::Integer);
		_logger->addDataField("f_val", DataType::Float);
		_logger->addDataField("best_f_so_far", DataType::Float);
		// _logger->addDataField("collisions", DataType::Float);
		// _logger->addDataField("best_x", DataType::String);

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
	
	void SteerSimOptimize::logData(LogObject logObject)
	{
		// LogObject benchmarkLogObject;
		_logger->writeLogObject(logObject);
		// benchmarkLogObject.addLogData((int)_currentScenario + (int)scenarioSetInitId);
	}

	void SteerSimOptimize::logData(int iteration, float f_val, float best_f)
	{
		LogObject benchmarkLogObject;
		
		benchmarkLogObject.addLogData(iteration);
		benchmarkLogObject.addLogData(f_val);
		benchmarkLogObject.addLogData(best_f);

		_logger->writeLogObject(benchmarkLogObject);

	}
	
}
