//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

//
// Copyright (c) 2009 Shawn Singh, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

/// @file EgocentricAIModule.cpp
/// @brief Implements the EgocentricAIModule plugin.


#include "SteerLib.h"

#include "SimulationPlugin.h"

#include "EgocentricAIModule.h"
#include "EgocentricAgent.h"
#include "GridAStar.h"

#include "LogObject.h"
#include "LogManager.h"

// globally accessible to the simpleAI plugin

namespace EgocentricGlobals
{

	SteerLib::EngineInterface * gEngineInfo;
	SteerLib::EngineInterface * gEngine;
	SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
	GridEnvironment * gEnvironmentForAStar;

	bool gUseDynamicPhaseScheduling;
	bool gShowStats;
	bool gShowAllStats;


	PhaseProfilers * gPhaseProfilers;

	float pedestrian_static_object_comfort_zone ;
	float pedestrian_dynamic_object_comfort_zone ;
	float minimum_activation_threshold ;
	float right_activation_decay ;
	float left_activation_decay ;
	float forward_activation_decay ;
	float backward_activation_decay ;
	float ego_ped_reached_target_distance_threshold ;
	int waypoint_distance ;
	int furthest_local_target_distance ;
	int ego_next_waypoint_distance ;
	int ped_max_num_waypoints ;
	float time_threshold ;
	float ped_max_speed                ;
	float ped_typical_speed            ;
	float ped_max_force                ;
	float pedestrian_radius ;
	int max_sacount ;
	float activation_decay ;
	float ped_max_turning_rate;
	float ped_scoot_rate;
}

using namespace EgocentricGlobals;



PLUGIN_API SteerLib::ModuleInterface * createModule()
{
	return new EgocentricAIModule;
}

PLUGIN_API void destroyModule( SteerLib::ModuleInterface*  module )
{
	if (module) delete module;
}


void EgocentricAIModule::init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo )
{
	gEngine = engineInfo;
	gSpatialDatabase = engineInfo->getSpatialDatabase();
	logFilename = "egocentricAI.log";
	logStats=false;

	// pedestrian_static_object_comfort_zone|pedestrian_dynamic_object_comfort_zone|minimum_activation_threshold|right_activation_decay|left_activation_decay|forward_activation_decay|backward_activation_decay|ego_ped_reached_target_distance_threshold|waypoint_distance|furthest_local_target_distance|ego_next_waypoint_distance|ped_max_num_waypoints|time_threshold|ped_max_speed|ped_typical_speed|ped_max_force|pedestrian_radius|max_sacount|activation_decay


	pedestrian_static_object_comfort_zone = PEDESTRIAN_STATIC_OBJECT_COMFORT_ZONE;
	pedestrian_dynamic_object_comfort_zone = PEDESTRIAN_DYNAMIC_OBJECT_COMFORT_ZONE;
	minimum_activation_threshold = MINIMUM_ACTIVATION_THRESHOLD;
	right_activation_decay = RIGHT_ACTIVATION_DECAY;
	left_activation_decay = LEFT_ACTIVATION_DECAY;
	forward_activation_decay = FORWARD_ACTIVATION_DECAY;
	backward_activation_decay = BACKWARD_ACTIVATION_DECAY;
	ego_ped_reached_target_distance_threshold = EGO_PED_REACHED_TARGET_DISTANCE_THRESHOLD;
	waypoint_distance = WAYPOINT_DISTANCE;
	furthest_local_target_distance = FURTHEST_LOCAL_TARGET_DISTANCE;
	ego_next_waypoint_distance = EGO_NEXT_WAYPOINT_DISTANCE;
	ped_max_num_waypoints = PED_MAX_NUM_WAYPOINTS;
	time_threshold = TIME_THRESHOLD;
	ped_max_speed = PED_MAX_SPEED;
	ped_typical_speed = PED_TYPICAL_SPEED;
	ped_max_force = PED_MAX_FORCE;
	pedestrian_radius = PEDESTRIAN_RADIUS;
	max_sacount = MAX_SACOUNT;
	activation_decay = ACTIVATION_DECAY;
	ped_max_turning_rate = PED_MAX_TURNING_RATE;
	ped_scoot_rate = PED_SCOOT_RATE ;

	GridDatabase2D * grid = dynamic_cast<GridDatabase2D *>(gSpatialDatabase);
	if ( grid != NULL)
	{
		gEnvironmentForAStar = new GridEnvironment(grid);
	}
	else
	{
		throw Util::GenericException("Egocentric Agent only supports the GridDatabase");
	}

	SteerLib::OptionDictionary::const_iterator optionIter;
	for (optionIter = options.begin(); optionIter != options.end(); ++optionIter)
	{
		std::stringstream value((*optionIter).second);
		if ((*optionIter).first == "ailogFileName")
		{
			logFilename = value.str();
			logStats = true;
		}
		else if ((*optionIter).first == "logStats")
		{
			logStats = true;
		}
		else if ((*optionIter).first == "stats")
		{
			gShowStats = Util::getBoolFromString(value.str());
		}
		else if ((*optionIter).first == "allstats")
		{
			gShowAllStats = Util::getBoolFromString(value.str());
		}
		/*
		 * Used this to replace values
		 * else if ((*optionIter).first == "$1")\r\n\t\t{\r\n\t\t\tvalue >> $1;\r\n\t\t}\r\n
		 * matching ([a-z|_|0-9]*);\r\n
		 */

		else if ((*optionIter).first == "pedestrian_static_object_comfort_zone")
		{
			value >> pedestrian_static_object_comfort_zone;
		}
		else if ((*optionIter).first == "pedestrian_dynamic_object_comfort_zone")
		{
			value >> pedestrian_dynamic_object_comfort_zone;
		}
		else if ((*optionIter).first == "minimum_activation_threshold")
		{
			value >> minimum_activation_threshold;
		}
		else if ((*optionIter).first == "right_activation_decay")
		{
			value >> right_activation_decay;
		}
		else if ((*optionIter).first == "left_activation_decay")
		{
			value >> left_activation_decay;
		}
		else if ((*optionIter).first == "forward_activation_decay")
		{
			value >> forward_activation_decay;
		}
		else if ((*optionIter).first == "backward_activation_decay")
		{
			value >> backward_activation_decay;
		}
		else if ((*optionIter).first == "ego_ped_reached_target_distance_threshold")
		{
			value >> ego_ped_reached_target_distance_threshold;
		}
		else if ((*optionIter).first == "waypoint_distance")
		{
			value >> waypoint_distance;
		}
		else if ((*optionIter).first == "furthest_local_target_distance")
		{
			value >> furthest_local_target_distance;
		}
		else if ((*optionIter).first == "ego_next_waypoint_distance")
		{
			value >> ego_next_waypoint_distance;
		}
		else if ((*optionIter).first == "ped_max_num_waypoints")
		{
			value >> ped_max_num_waypoints;
		}
		else if ((*optionIter).first == "time_threshold")
		{
			value >> time_threshold;
		}
		else if ((*optionIter).first == "ped_max_speed")
		{
			value >> ped_max_speed;
		}
		else if ((*optionIter).first == "ped_typical_speed")
		{
			value >> ped_typical_speed;
		}
		else if ((*optionIter).first == "ped_max_force")
		{
			value >> ped_max_force;
		}
		else if ((*optionIter).first == "pedestrian_radius")
		{
			value >> pedestrian_radius;
		}
		else if ((*optionIter).first == "max_sacount")
		{
			value >> max_sacount;
		}
		else if ((*optionIter).first == "activation_decay")
		{
			value >> activation_decay;
		}

		else if ((*optionIter).first == "ped_max_turning_rate")
		{
			std::cout << "Setting max turn rate to " << value.str() << std::endl;
			value >> ped_max_turning_rate;
		}
		else if ((*optionIter).first == "ped_scoot_rate")
		{
			value >> ped_scoot_rate;
		}

		else
		{
			// throw Util::GenericException("unrecognized option \"" + Util::toString((*optionIter).first) + "\" given to PPR AI module.");
		}
	}


		_logger = LogManager::getInstance()->createLogger(logFilename,LoggerType::BASIC_WRITE);

		_logger->addDataField("number_of_times_executed",DataType::LongLong );
		_logger->addDataField("total_ticks_accumulated",DataType::LongLong );
		_logger->addDataField("shortest_execution",DataType::LongLong );
		_logger->addDataField("longest_execution",DataType::LongLong );
		_logger->addDataField("fastest_execution", DataType::Float);
		_logger->addDataField("slowest_execution", DataType::Float);
		_logger->addDataField("average_time_per_call", DataType::Float);
		_logger->addDataField("total_time_of_all_calls", DataType::Float);
		_logger->addDataField("tick_frequency", DataType::Float);

		// LETS TRY TO WRITE THE LABELS OF EACH FIELD
	if( logStats )
	{
		std::stringstream labelStream;
		unsigned int i;
		for (i=0; i < _logger->getNumberOfFields() - 1; i++)
			labelStream << _logger->getFieldName(i) << " ";
		labelStream << _logger->getFieldName(i);
		_data = labelStream.str() + "\n";

		_logger->writeData(labelStream.str());

	}
}

void EgocentricAIModule::initializeSimulation()
{
	//
	// initialize the performance profilers
	//
	gPhaseProfilers = new PhaseProfilers;
	gPhaseProfilers->aiProfiler.reset();
	gPhaseProfilers->longTermPhaseProfiler.reset();
	gPhaseProfilers->midTermPhaseProfiler.reset();
	gPhaseProfilers->shortTermPhaseProfiler.reset();
	gPhaseProfilers->perceptivePhaseProfiler.reset();
	gPhaseProfilers->predictivePhaseProfiler.reset();
	gPhaseProfilers->reactivePhaseProfiler.reset();
	gPhaseProfilers->steeringPhaseProfiler.reset();

}

void EgocentricAIModule::cleanupSimulation()
{


	LogObject egocentricLogObject;

	egocentricLogObject.addLogData(gPhaseProfilers->aiProfiler.getNumTimesExecuted());
	egocentricLogObject.addLogData(gPhaseProfilers->aiProfiler.getTotalTicksAccumulated());
	egocentricLogObject.addLogData(gPhaseProfilers->aiProfiler.getMinTicks());
	egocentricLogObject.addLogData(gPhaseProfilers->aiProfiler.getMaxTicks());
	egocentricLogObject.addLogData(gPhaseProfilers->aiProfiler.getMinExecutionTimeMills());
	egocentricLogObject.addLogData(gPhaseProfilers->aiProfiler.getMaxExecutionTimeMills());
	egocentricLogObject.addLogData(gPhaseProfilers->aiProfiler.getAverageExecutionTimeMills());
	egocentricLogObject.addLogData(gPhaseProfilers->aiProfiler.getTotalTime());
	egocentricLogObject.addLogData(gPhaseProfilers->aiProfiler.getTickFrequency());

	_logger->writeLogObject(egocentricLogObject);
	_data = _data + _logger->logObjectToString(egocentricLogObject);
	_logData.push_back(egocentricLogObject.copy());

	// cleanup profileing metrics for next simulation/scenario
	gPhaseProfilers->aiProfiler.reset();
	gPhaseProfilers->longTermPhaseProfiler.reset();
	gPhaseProfilers->midTermPhaseProfiler.reset();
	gPhaseProfilers->shortTermPhaseProfiler.reset();
	gPhaseProfilers->perceptivePhaseProfiler.reset();
	gPhaseProfilers->predictivePhaseProfiler.reset();
	gPhaseProfilers->reactivePhaseProfiler.reset();
	gPhaseProfilers->steeringPhaseProfiler.reset();

	if ( logStats )
	{
		_logger->writeLogObject(egocentricLogObject);
	}

}

void EgocentricAIModule::finish()
{
	// nothing to do here
}

SteerLib::AgentInterface * EgocentricAIModule::createAgent()
{
	return new EgocentricAgent; 
}

void EgocentricAIModule::destroyAgent( SteerLib::AgentInterface * agent )
{
	assert(agent!=NULL);
	delete agent;
	agent = NULL;
}
