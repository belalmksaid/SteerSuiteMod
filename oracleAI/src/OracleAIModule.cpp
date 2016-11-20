//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#include "SteerLib.h"
#include "SteerSimPlugin.h"
#include "oracleAI/GridAStar.h"
#include "oracleAI/OracleAIModule.h"
#include "oracleAI/OracleAgent.h"

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
namespace OracleGlobals {
	SteerLib::EngineInterface * gEngineInfo;
	SteerLib::GridDatabase2D * gSpatialDatabase;
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
	
	PhaseProfilers * gPhaseProfilers;
}

using namespace OracleGlobals;

//
// 
//
void OracleAIModule::init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo )
{

	gSpatialDatabase = engineInfo->getSpatialDatabase();
	gEnvironmentForAStar = new GridEnvironment(gSpatialDatabase);
	gEngineInfo = engineInfo;

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
	logToFile = false;
	logFilename = "oracleAI.log";

	SteerLib::OptionDictionary::const_iterator optionIter;
	for (optionIter = options.begin(); optionIter != options.end(); ++optionIter) {
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
		if ((*optionIter).first == "allstats")
		{
			value >> gShowAllStats;
		}
		else if ((*optionIter).first == "stats")
		{
			gShowStats = Util::getBoolFromString(value.str());
		}
		else if ((*optionIter).first == "ailogFileName")
		{
			logFilename = value.str();
		}
		else if ((*optionIter).first == "ailogFileName")
		{
			logToFile = true;
			logFilename = value.str();
		}
		else {
			// throw Util::GenericException("unrecognized option \"" + Util::toString((*optionIter).first) + "\" given to oracleAI module.");
		}
	}


	//gDesiredFPS = engineInfo->getClock().getFixedFrameRate();
	//gSpatialIncrement = PED_TYPICAL_SPEED * 0.5f * gNumFramesPerSpaceTimePathNode / gDesiredFPS;  // in other words, distance-traveled-in-one-frame * number-frames-to-skip-for-each-path-node, when going half (0.5f) speed.


	//
	// initialize the performance profilers
	//
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
#endif*/



	// Added by Glen to log data on oracles for steerstats
	if (logToFile)
	{
		_oracleLogger = LogManager::getInstance()->createLogger(logFilename,LoggerType::BASIC_WRITE);
		_oracleLogger->addDataField("planning_number_of_times_executed",DataType::LongLong );
		_oracleLogger->addDataField("planning_total_ticks_accumulated",DataType::LongLong );
		_oracleLogger->addDataField("planning_shortest_execution",DataType::LongLong );
		_oracleLogger->addDataField("planning_longest_execution",DataType::LongLong );
		_oracleLogger->addDataField("planning_tick_frequency", DataType::Float);
		_oracleLogger->addDataField("planning_fastest_execution", DataType::Float);
		_oracleLogger->addDataField("planning_slowest_execution", DataType::Float);
		_oracleLogger->addDataField("planning_average_time_per_call", DataType::Float);
		_oracleLogger->addDataField("planning_total_time_of_all_calls", DataType::Float);
		_oracleLogger->addDataField("number_of_times_executed",DataType::LongLong );
		_oracleLogger->addDataField("total_ticks_accumulated",DataType::LongLong );
		_oracleLogger->addDataField("shortest_execution",DataType::LongLong );
		_oracleLogger->addDataField("longest_execution",DataType::LongLong );
		_oracleLogger->addDataField("tick_frequency", DataType::Float);
		_oracleLogger->addDataField("fastest_execution", DataType::Float);
		_oracleLogger->addDataField("slowest_execution", DataType::Float);
		_oracleLogger->addDataField("average_time_per_call", DataType::Float);
		_oracleLogger->addDataField("total_time_of_all_calls", DataType::Float);

		// LETS TRY TO WRITE THE LABELS OF EACH FIELD
		std::stringstream labelStream;
		unsigned int i;
		for (i=0; i < _oracleLogger->getNumberOfFields() - 1; i++)
			labelStream << _oracleLogger->getFieldName(i) << " ";
		labelStream << _oracleLogger->getFieldName(i);

		_oracleLogger->writeData(labelStream.str());
	}

}

void OracleAIModule::postprocessSimulation()
{
	// UNCOMMENT THIS IF YOU WANT TO RECORD A FOOTSTEP REC FILE.
	if (gMakeFootstepRec) {
		size_t numAgents = gEngineInfo->getAgents().size();
		size_t numObstacles = gEngineInfo->getObstacles().size();
		FootRecWriter frw(numAgents, numObstacles);
		for (unsigned int i=0; i < numAgents; i++) {
			OracleAgent * agent = dynamic_cast<OracleAgent*>(gEngineInfo->getAgents()[i]);
			for (unsigned int step=0; step < agent->_stepHistory.size(); step++) {
				frw.addFootstep(i, agent->_stepHistory[step], 0);
			}
		}
		
		unsigned int obstacleID = 0;
		for(set<SteerLib::ObstacleInterface*>::const_iterator iter = gEngineInfo->getObstacles().cbegin(); iter != gEngineInfo->getObstacles().cend(); ++iter) {
			AxisAlignedBox currObstacle = (*iter)->getBounds();
			frw.addObstacleInfo(obstacleID, currObstacle.xmin, currObstacle.xmax, currObstacle.zmin, currObstacle.zmax);
			obstacleID++;
		}
		frw.writeToFile(gFootstepRecFilename);
	}
}


void OracleAIModule::cleanupSimulation()
{
	// do stuff
	// print data out to a log file for the simulation.
	if ( logToFile )
	{
		LogObject oracleLogObject;
		oracleLogObject.addLogData(gPhaseProfilers->footstepPlanningPhaseProfiler.getNumTimesExecuted());
		oracleLogObject.addLogData(gPhaseProfilers->footstepPlanningPhaseProfiler.getTotalTicksAccumulated());
		oracleLogObject.addLogData(gPhaseProfilers->footstepPlanningPhaseProfiler.getMinTicks());
		oracleLogObject.addLogData(gPhaseProfilers->footstepPlanningPhaseProfiler.getMaxTicks());
		oracleLogObject.addLogData(gPhaseProfilers->footstepPlanningPhaseProfiler.getTickFrequency());
		oracleLogObject.addLogData(gPhaseProfilers->footstepPlanningPhaseProfiler.getMinExecutionTimeMills());
		oracleLogObject.addLogData(gPhaseProfilers->footstepPlanningPhaseProfiler.getMaxExecutionTimeMills());
		oracleLogObject.addLogData(gPhaseProfilers->footstepPlanningPhaseProfiler.getAverageExecutionTimeMills());
		oracleLogObject.addLogData(gPhaseProfilers->footstepPlanningPhaseProfiler.getTotalTime());
		oracleLogObject.addLogData(gPhaseProfilers->aiProfiler.getNumTimesExecuted());
		oracleLogObject.addLogData(gPhaseProfilers->aiProfiler.getTotalTicksAccumulated());
		oracleLogObject.addLogData(gPhaseProfilers->aiProfiler.getMinTicks());
		oracleLogObject.addLogData(gPhaseProfilers->aiProfiler.getMaxTicks());
		oracleLogObject.addLogData(gPhaseProfilers->aiProfiler.getTickFrequency());
		oracleLogObject.addLogData(gPhaseProfilers->aiProfiler.getMinExecutionTimeMills());
		oracleLogObject.addLogData(gPhaseProfilers->aiProfiler.getMaxExecutionTimeMills());
		oracleLogObject.addLogData(gPhaseProfilers->aiProfiler.getAverageExecutionTimeMills());
		oracleLogObject.addLogData(gPhaseProfilers->aiProfiler.getTotalTime());
		_oracleLogger->writeLogObject(oracleLogObject);
	}


	if (gShowStats)
	{
		cout << "--- Footstep planning ---\n";
		gPhaseProfilers->footstepPlanningPhaseProfiler.displayStatistics(cout);
		cout << endl;

		cout << "--- TOTAL AI ---\n";
		gPhaseProfilers->aiProfiler.displayStatistics(cout);
		cout << endl;
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
void OracleAIModule::finish()
{

	// TODO: de-allocate agents here.
	delete gEnvironmentForAStar;

	_oracleLogger->closeLog();

}


void OracleAIModule::draw()
{
}

void OracleAIModule::initializeSimulation()
{
}

PLUGIN_API SteerLib::ModuleInterface * createModule() { return new OracleAIModule; }

PLUGIN_API void destroyModule( SteerLib::ModuleInterface*  module ) { if (module) delete module; module = NULL; }

