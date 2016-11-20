//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#ifndef SteerSimOptimize_H
#define SteerSimOptimize_H

#include "simulation/SimulationOptions.h"
#include "SteerSuite.h"
#include "SimWorld.h"
#include "SteerOptPlugin.h"
#include "OptimizationParameters.h"
#include "OptimizationConfiguration.h"
#include "LogManager.h"

#include <sstream>
#include <string>

namespace SteerOpt {
	/**
	 * \brief Convex quadratic benchmark function.
	 */
	class STEEROPTPLUG_API SteerSimOptimize
	{
public:
		SteerSimOptimize(SteerSuite::SimWorld * world, SteerSuite::SteerSuite * steersuite);

		/// \brief From INameable: return the class name.
		virtual void optimize(std::string objective) =0;

		virtual double simulate(const double* x, const int N);
		virtual double simulateSteeringAlgorithm(const double* x, const int N);
		virtual void setOptimizationParameters(Graphing::OptimizationParameters _params);
		virtual void setMaxEvals( size_t max_evals)
		{
			_max_evals = max_evals;
		}
		virtual void setOptimiationConfiguration(SteerOpt::OptimizationConfiguration optConfig)
		{
			_optConfig = optConfig;
		}
		virtual void logOptimization(std::string logFileName);
		virtual void logData(LogObject logObject);
		virtual void logData(int iteration, float f_val, float best_f);

public:
	SteerSuite::SteerSuite * _steerSuite;
	SteerSuite::SimWorld * _world;
	Graphing::OptimizationParameters _params;
	SteerOpt::OptimizationConfiguration _optConfig;
	size_t _max_evals;
	Logger * _logger;
	
};

}

#endif
