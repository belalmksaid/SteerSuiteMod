//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#include "SteerLib.h"
#include "SteerSimPlugin.h"
#include "footstepAI/GridAStar.h"
#include "footstepAI/FootstepAIModule.h"
#include "footstepAI/FootstepAgent.h"

#include "footrec/FootRecIO.h"

#include "LogObject.h"
#include "LogManager.h"


#define LONG_TERM_PLANNING_INTERVAL    10000
#define MID_TERM_PLANNING_INTERVAL     10000
#define SHORT_TERM_PLANNING_INTERVAL   1
#define PERCEPTIVE_PHASE_INTERVAL      1
#define PREDICTIVE_PHASE_INTERVAL      1
#define REACTIVE_PHASE_INTERVAL        1
#define SPACE_TIME_DISTANCE            8


using namespace Util;
using namespace SteerLib;



// todo: make these static?
namespace FootstepGlobals {
	SteerLib::EngineInterface * gEngine;
	SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
	FootRecWriter* gFootRecWriter;
	
	GridEnvironment * gEnvironmentForAStar;
/*	float gSpatialIncrement;
	float gDesiredFPS; // TODO: how to best conceptually cleanly get this?
	unsigned int gLongTermPlanningPhaseInterval;
	unsigned int gMidTermPlanningPhaseInterval;
	unsigned int gShortTermPlanningPhaseInterval;
	unsigned int gPredictivePhaseInterval;
	unsigned int gReactivePhaseInterval;
	unsigned int gPerceptivePhaseInterval;
	unsigned int gNumFramesPerSpaceTimePathNode;

	bool gUseDynamicDecimation;*/
	bool gShowStats;
	bool gShowAllStats;
	bool gMakeFootstepRec;
	std::string gFootstepRecFilename;

	bool gMakeFootstepSTPRec;
	std::string gFootstepSTPRecFilename;
	
	PhaseProfilers * gPhaseProfilers;

	float preferred_step_angle;
	float default_com_height;
	float default_mass;
	float default_min_step_length;
	float default_max_step_length;
	float default_min_step_time;
	float default_max_step_time;
	float default_max_speed;
	float default_base_radius;
	float default_time_cost_weight;
	float default_trajectory_cost_weght;
	float shoulder_comfort_zone;
	float shoulder_comfort_zone_2;
	float ped_reached_target_distance_threshold;
	int furthest_local_target_distance;
	int next_waypoint_distance;
	int ped_max_num_waypoints;
	float ped_query_radius;
	float ped_torso_radius;
	int ped_num_steps_before_forced_plan;
	float ped_initial_step_variation;
	float ped_reached_footstep_goal_threshold;

	int max_nodes_to_expand;
}

using namespace FootstepGlobals;

//
// 
//
void FootstepAIModule::init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo )
{

	gSpatialDatabase = engineInfo->getSpatialDatabase();
	GridDatabase2D * grid = dynamic_cast<GridDatabase2D *>(gSpatialDatabase);
	// if ( grid != NULL)
	{
		gEnvironmentForAStar = new GridEnvironment(grid);
	}
	{
		// throw Util::GenericException("Footstep Agent only supports the GridDatabase");
	}
	gEngine = engineInfo;

	//
	// parse command-line options
	//
	CommandLineParser opts;

	//gLongTermPlanningPhaseInterval = LONG_TERM_PLANNING_INTERVAL;
	//gMidTermPlanningPhaseInterval = MID_TERM_PLANNING_INTERVAL;
	//gShortTermPlanningPhaseInterval = SHORT_TERM_PLANNING_INTERVAL;
	//gPerceptivePhaseInterval = PERCEPTIVE_PHASE_INTERVAL;
	//gPredictivePhaseInterval = PREDICTIVE_PHASE_INTERVAL;
	//gReactivePhaseInterval = REACTIVE_PHASE_INTERVAL;
	//gNumFramesPerSpaceTimePathNode = SPACE_TIME_DISTANCE;
	//gUseDynamicDecimation = false;
	gShowStats = false;
	gShowAllStats = false;
	gMakeFootstepRec = false;
	gFootstepRecFilename = "";
	gMakeFootstepSTPRec = false;
	gFootstepSTPRecFilename = "";
	logToFile = false;
	logFilename = "footstepAI.log";

	preferred_step_angle = PREFERRED_STEP_ANGLE;
	default_com_height = DEFAULT_COM_HEIGHT;
	default_mass = DEFAULT_MASS;
	default_min_step_length = DEFAULT_MIN_STEP_LENGTH;
	default_max_step_length = DEFAULT_MAX_STEP_LENGTH;
	default_min_step_time = DEFAULT_MIN_STEP_TIME;
	default_max_step_time = DEFAULT_MAX_STEP_TIME;
	default_max_speed = DEFAULT_MAX_SPEED;
	default_base_radius = DEFAULT_BASE_RADIUS;
	default_time_cost_weight = DEFAULT_TIME_COST_WEIGHT;
	default_trajectory_cost_weght = DEFAULT_TRAJECTORY_COST_WEGHT;
	shoulder_comfort_zone = SHOULDER_COMFORT_ZONE;
	shoulder_comfort_zone_2 = SHOULDER_COMFORT_ZONE_2;
	ped_reached_target_distance_threshold = PED_REACHED_TARGET_DISTANCE_THRESHOLD;
	furthest_local_target_distance = FURTHEST_LOCAL_TARGET_DISTANCE;
	next_waypoint_distance = NEXT_WAYPOINT_DISTANCE;
	ped_max_num_waypoints = PED_MAX_NUM_WAYPOINTS;
	ped_query_radius = PED_QUERY_RADIUS;
	ped_torso_radius = PED_TORSO_RADIUS;
	ped_num_steps_before_forced_plan = PED_NUM_STEPS_BEFORE_FORCED_PLAN;
	ped_initial_step_variation = PED_INITIAL_STEP_VARIATION;
	ped_reached_footstep_goal_threshold = PED_REACHED_FOOTSTEP_GOAL_THRESHOLD;

	max_nodes_to_expand = MAX_NODES_TO_EXPAND;

	SteerLib::OptionDictionary::const_iterator optionIter;
	for (optionIter = options.begin(); optionIter != options.end(); optionIter++)
	{
		std::stringstream value((*optionIter).second);
		//if ((*optionIter).first == "longplan") {
		//	value >> gLongTermPlanningPhaseInterval;
		//}
		//else if ((*optionIter).first == "midplan") {
		//	value >> gMidTermPlanningPhaseInterval;
		//}
		//else if ((*optionIter).first == "shortplan") {
		//	value >> gShortTermPlanningPhaseInterval;
		//}
		//else if ((*optionIter).first == "perceptive") {
		//	value >> gPerceptivePhaseInterval;
		//}
		//else if ((*optionIter).first == "predictive") {
		//	value >> gPredictivePhaseInterval;
		//}
		//else if ((*optionIter).first == "reactive") {
		//	value >> gReactivePhaseInterval;
		//}
		//else if ((*optionIter).first == "spacetimeframes") {
		//	value >> gNumFramesPerSpaceTimePathNode;
		//}
		//else if ((*optionIter).first == "dynamic") {
		//	value >> gUseDynamicDecimation;
		//}
		//else if ((*optionIter).first == "stats") {
		//	value >> gShowStats;
		//}
		//else 
		if ((*optionIter).first == "allstats") {
			value >> gShowAllStats;
		}
		else if ((*optionIter).first == "stats")
		{
			gShowStats = Util::getBoolFromString(value.str());
		}
		else if((*optionIter).first == "record") {
			gMakeFootstepRec = true;
			value >> gFootstepRecFilename;
		}
		else if((*optionIter).first == "recordSTP") {
			gMakeFootstepSTPRec = true;
			value >> gFootstepSTPRecFilename;
		}
		else if ((*optionIter).first == "ailogFileName")
		{
			logToFile = true;
			logFilename = value.str();
		}
		/*
		 * Used this to replace values
		 * else if ((*optionIter).first == "$1")\n\t\t{\n\t\t\tvalue >> $1;\n\t\t}\n
		 * matching ([a-z|_|0-9]*);\n
		 */
		else if ((*optionIter).first == "preferred_step_angle")
		{
			value >> preferred_step_angle;
		}
		else if ((*optionIter).first == "default_com_height")
		{
			value >> default_com_height;
		}
		else if ((*optionIter).first == "default_mass")
		{
			value >> default_mass;
		}
		else if ((*optionIter).first == "default_min_step_length")
		{
			value >> default_min_step_length;
		}
		else if ((*optionIter).first == "default_max_step_length")
		{
			value >> default_max_step_length;
		}
		else if ((*optionIter).first == "default_min_step_time")
		{
			value >> default_min_step_time;
		}
		else if ((*optionIter).first == "default_max_step_time")
		{
			value >> default_max_step_time;
		}
		else if ((*optionIter).first == "default_max_speed")
		{
			value >> default_max_speed;
		}
		else if ((*optionIter).first == "default_base_radius")
		{
			value >> default_base_radius;
		}
		else if ((*optionIter).first == "default_time_cost_weight")
		{
			value >> default_time_cost_weight;
		}
		else if ((*optionIter).first == "default_trajectory_cost_weght")
		{
			value >> default_trajectory_cost_weght;
		}
		else if ((*optionIter).first == "shoulder_comfort_zone")
		{
			value >> shoulder_comfort_zone;
		}
		else if ((*optionIter).first == "shoulder_comfort_zone_2")
		{
			value >> shoulder_comfort_zone_2;
		}
		else if ((*optionIter).first == "ped_reached_target_distance_threshold")
		{
			value >> ped_reached_target_distance_threshold;
		}
		else if ((*optionIter).first == "furthest_local_target_distance")
		{
			value >> furthest_local_target_distance;
		}
		else if ((*optionIter).first == "next_waypoint_distance")
		{
			value >> next_waypoint_distance;
		}
		else if ((*optionIter).first == "ped_max_num_waypoints")
		{
			value >> ped_max_num_waypoints;
		}
		else if ((*optionIter).first == "ped_query_radius")
		{
			std::cout << "Setting query radius to " << value.str() << std::endl;
			value >> ped_query_radius;
		}
		else if ((*optionIter).first == "ped_torso_radius")
		{
			value >> ped_torso_radius;
		}
		else if ((*optionIter).first == "ped_num_steps_before_forced_plan")
		{
			value >> ped_num_steps_before_forced_plan;
		}
		else if ((*optionIter).first == "ped_initial_step_variation")
		{
			value >> ped_initial_step_variation;
		}
		else if ((*optionIter).first == "ped_reached_footstep_goal_threshold")
		{
			value >> ped_reached_footstep_goal_threshold;
		}
		else if ((*optionIter).first == "max_nodes_to_expand")
		{
			value >> max_nodes_to_expand;
			std::cout << "Setting max_nodes_to_expand to " << max_nodes_to_expand << std::endl;
		}
		else
		{
			// throw Util::GenericException("unrecognized option \"" + Util::toString((*optionIter).first) + "\" given to footstepAI module.");
		}
	}


	//gDesiredFPS = engineInfo->getClock().getFixedFrameRate();
	//gSpatialIncrement = PED_TYPICAL_SPEED * 0.5f * gNumFramesPerSpaceTimePathNode / gDesiredFPS;  // in other words, distance-traveled-in-one-frame * number-frames-to-skip-for-each-path-node, when going half (0.5f) speed.


	//
	// initialize the performance profilers
	//
/*
	cout << endl;
	if (!gUseDynamicDecimation) {
		cout << " PHASE INTERVALS (in frames):\n";
		cout << " longplan:   "   << gLongTermPlanningPhaseInterval << "\n";
		cout << " midplan:    "    << gMidTermPlanningPhaseInterval << "\n";
		cout << " shortplan:  "  << gShortTermPlanningPhaseInterval << "\n";
		cout << " perceptive: " << gPerceptivePhaseInterval << "\n";
		cout << " predictive: " << gPredictivePhaseInterval << "\n";
		cout << " reactive:   "   << gReactivePhaseInterval << "\n";
		cout << " spacetimeframes: " << gNumFramesPerSpaceTimePathNode << "\n";
	}
	else {
		cout << " Using dynamic phase decimation.\n";
	}
	cout << endl;


	//
	// print a warning if we are using annotations with too many agents.
	//
#ifdef USE_ANNOTATIONS
	if (testCase.getNumAgents() > 30) {
		cerr << "WARNING: using annotations with a large number of agents will be extremely slow and use a lot of memory." << endl;
	}
#endif
*/


	if (logToFile)
	{
		// Added by Glen to log data on footsteps for steerstats
		_footstepLogger = LogManager::getInstance()->createLogger(logFilename,LoggerType::BASIC_WRITE);
		_footstepLogger->addDataField("planning_number_of_times_executed",DataType::LongLong );
		_footstepLogger->addDataField("planning_total_ticks_accumulated",DataType::LongLong );
		_footstepLogger->addDataField("planning_shortest_execution",DataType::LongLong );
		_footstepLogger->addDataField("planning_longest_execution",DataType::LongLong );
		_footstepLogger->addDataField("planning_tick_frequency", DataType::Float);
		_footstepLogger->addDataField("planning_fastest_execution", DataType::Float);
		_footstepLogger->addDataField("planning_slowest_execution", DataType::Float);
		_footstepLogger->addDataField("planning_average_time_per_call", DataType::Float);
		_footstepLogger->addDataField("planning_total_time_of_all_calls", DataType::Float);
		_footstepLogger->addDataField("number_of_times_executed",DataType::LongLong );
		_footstepLogger->addDataField("total_ticks_accumulated",DataType::LongLong );
		_footstepLogger->addDataField("shortest_execution",DataType::LongLong );
		_footstepLogger->addDataField("longest_execution",DataType::LongLong );
		_footstepLogger->addDataField("tick_frequency", DataType::Float);
		_footstepLogger->addDataField("fastest_execution", DataType::Float);
		_footstepLogger->addDataField("slowest_execution", DataType::Float);
		_footstepLogger->addDataField("average_time_per_call", DataType::Float);
		_footstepLogger->addDataField("total_time_of_all_calls", DataType::Float);

		// LETS TRY TO WRITE THE LABELS OF EACH FIELD
		std::stringstream labelStream;
		unsigned int i;
		for (i=0; i < _footstepLogger->getNumberOfFields() - 1; i++)
			labelStream << _footstepLogger->getFieldName(i) << " ";
		labelStream << _footstepLogger->getFieldName(i);
		_data = labelStream.str() + "\n";
		_footstepLogger->writeData(labelStream.str());
	}
}

void FootstepAIModule::postprocessSimulation()
{
	// UNCOMMENT THIS IF YOU WANT TO RECORD A FOOTSTEP REC FILE.
	if (gMakeFootstepRec || gMakeFootstepSTPRec) {
		size_t numAgents = gEngine->getAgents().size();
		size_t numObstacles = gEngine->getObstacles().size();
		FootRecWriter frw(numAgents, numObstacles);
		for (unsigned int i=0; i < numAgents; i++) {
			FootstepAgent * agent = dynamic_cast<FootstepAgent*>(gEngine->getAgents()[i]);
			for (unsigned int step=0; step < agent->_stepHistory.size(); step++) {
				frw.addFootstep(i, agent->_stepHistory[step], 0);
			}

			// add the final step
			frw.addFootstep(i, agent->_currentStep, 0);
		}
		
		unsigned int obstacleID = 0;
		for(set<SteerLib::ObstacleInterface*>::const_iterator iter = gEngine->getObstacles().begin(); iter != gEngine->getObstacles().end(); iter++)
		{ // updated cbegin to begin and cend to end.
			AxisAlignedBox currObstacle = (*iter)->getBounds();
			frw.addObstacleInfo(obstacleID, currObstacle.xmin, currObstacle.xmax, currObstacle.zmin, currObstacle.zmax);
			obstacleID++;
		}

		if (gMakeFootstepRec)
			frw.writeToFile(gFootstepRecFilename);
		if (gMakeFootstepSTPRec)
			frw.writeToStepFile(gFootstepSTPRecFilename);
	}
}


void FootstepAIModule::cleanupSimulation()
{

	if (gShowStats)
	{
		cout << "--- Footstep planning ---\n";
		gPhaseProfilers->footstepPlanningPhaseProfiler.displayStatistics(cout);
		cout << endl;

		cout << "--- TOTAL AI ---\n";
		gPhaseProfilers->aiProfiler.displayStatistics(cout);
		cout << endl;
	}

	// do stuff
	// print data out to a log file for the simulation.
	if (logToFile)
	{

		LogObject footstepLogObject;
		footstepLogObject.addLogData(gPhaseProfilers->footstepPlanningPhaseProfiler.getNumTimesExecuted());
		footstepLogObject.addLogData(gPhaseProfilers->footstepPlanningPhaseProfiler.getTotalTicksAccumulated());
		footstepLogObject.addLogData(gPhaseProfilers->footstepPlanningPhaseProfiler.getMinTicks());
		footstepLogObject.addLogData(gPhaseProfilers->footstepPlanningPhaseProfiler.getMaxTicks());
		footstepLogObject.addLogData(gPhaseProfilers->footstepPlanningPhaseProfiler.getTickFrequency());
		footstepLogObject.addLogData(gPhaseProfilers->footstepPlanningPhaseProfiler.getMinExecutionTimeMills());
		footstepLogObject.addLogData(gPhaseProfilers->footstepPlanningPhaseProfiler.getMaxExecutionTimeMills());
		footstepLogObject.addLogData(gPhaseProfilers->footstepPlanningPhaseProfiler.getAverageExecutionTimeMills());
		footstepLogObject.addLogData(gPhaseProfilers->footstepPlanningPhaseProfiler.getTotalTime());
		footstepLogObject.addLogData(gPhaseProfilers->aiProfiler.getNumTimesExecuted());
		footstepLogObject.addLogData(gPhaseProfilers->aiProfiler.getTotalTicksAccumulated());
		footstepLogObject.addLogData(gPhaseProfilers->aiProfiler.getMinTicks());
		footstepLogObject.addLogData(gPhaseProfilers->aiProfiler.getMaxTicks());
		footstepLogObject.addLogData(gPhaseProfilers->aiProfiler.getTickFrequency());
		footstepLogObject.addLogData(gPhaseProfilers->aiProfiler.getMinExecutionTimeMills());
		footstepLogObject.addLogData(gPhaseProfilers->aiProfiler.getMaxExecutionTimeMills());
		footstepLogObject.addLogData(gPhaseProfilers->aiProfiler.getAverageExecutionTimeMills());
		footstepLogObject.addLogData(gPhaseProfilers->aiProfiler.getTotalTime());
		_footstepLogger->writeLogObject(footstepLogObject);
		// _footstepLogger->writeData(_footstepLogger->logObjectToString(footstepLogObject));
		// std::cout << "Writing data to footstep logFile " << footstepLogObject << std::endl;
		_data = _data + _footstepLogger->logObjectToString(footstepLogObject);
		// std::cout << this->getData() << std::endl;
		_logData.push_back(footstepLogObject.copy());

	}

	gPhaseProfilers->aiProfiler.reset();
	gPhaseProfilers->drawProfiler.reset();
	gPhaseProfilers->footstepPlanningPhaseProfiler.reset();
	gPhaseProfilers->longTermPhaseProfiler.reset();
	gPhaseProfilers->midTermPhaseProfiler.reset();
	gPhaseProfilers->shortTermPhaseProfiler.reset();
	gPhaseProfilers->perceptivePhaseProfiler.reset();
	gPhaseProfilers->locomotionPhaseProfiler.reset();

}
//
// finish()
//
void FootstepAIModule::finish()
{


	// TODO: de-allocate agents here.
	if ( gEnvironmentForAStar )
	{
		delete gEnvironmentForAStar;
		gEnvironmentForAStar=NULL;
	}

	if (logToFile)
	{
		_footstepLogger->closeLog();
		// std::cout << "Datalog length: " << _logData.size() << std::endl;
	}
}


void FootstepAIModule::draw()
{
}

void FootstepAIModule::initializeSimulation()
{
	gPhaseProfilers = new PhaseProfilers;
	gPhaseProfilers->aiProfiler.reset();
	gPhaseProfilers->drawProfiler.reset();
	gPhaseProfilers->footstepPlanningPhaseProfiler.reset();
	gPhaseProfilers->longTermPhaseProfiler.reset();
	gPhaseProfilers->midTermPhaseProfiler.reset();
	gPhaseProfilers->shortTermPhaseProfiler.reset();
	gPhaseProfilers->perceptivePhaseProfiler.reset();
	//gPhaseProfilers->predictivePhaseProfiler.reset();
	//gPhaseProfilers->reactivePhaseProfiler.reset();
	gPhaseProfilers->locomotionPhaseProfiler.reset();

}



PLUGIN_API SteerLib::ModuleInterface * createModule() { return new FootstepAIModule; }

PLUGIN_API void destroyModule( SteerLib::ModuleInterface*  module ) { if (module) delete module; module = NULL; }

