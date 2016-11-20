//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#include "SteerLib.h"
#include "SteerSimPlugin.h"
#include "shadowAI/GridAStar.h"
#include "shadowAI/ShadowAIModule.h"
#include "shadowAI/ShadowAgent.h"

#include "shadowRec/ShadowRecIO.h"


#define LONG_TERM_PLANNING_INTERVAL    10000
#define MID_TERM_PLANNING_INTERVAL     10000
#define SHORT_TERM_PLANNING_INTERVAL   1
#define PERCEPTIVE_PHASE_INTERVAL      1
#define PREDICTIVE_PHASE_INTERVAL      1
#define REACTIVE_PHASE_INTERVAL        1
#define SPACE_TIME_DISTANCE            8


using namespace Util;
using namespace SteerLib;

unsigned int ShadowAIModule::agentID = 0;

// todo: make these static?
namespace ShadowAIGlobals {
	SteerLib::EngineInterface * gEngineInfo;
	SteerLib::GridDatabase2D * gSpatialDatabase;
	ShadowRecWriter* gFootRecWriter;
	
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
	bool gMakeShadowRec;
	std::string gShadowRecFilename;
	std::string gDatabaseRoot;
	std::string gTrackReader;
	
	PhaseProfilers * gPhaseProfilers;
}

using namespace ShadowAIGlobals;

//
// 
//

SteerLib::AgentInterface* ShadowAIModule::createAgent() {
	SteerLib::AgentInterface* newGuy = new ShadowAgent(classifier, featureSets, track_reader, agentID++);
	agentLinks.push_back(static_cast<ShadowAgent*>(newGuy));	//I use static cast here because it's a little faster and i KNOW i just made a ShadowAgent
	return newGuy;
}

void ShadowAIModule::init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo )
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
	gShowStats = true;
	gShowAllStats = false;
	gMakeShadowRec = false;
	gShadowRecFilename = "";
	bool databaseRootSet = false;
	bool trackReader = false;
	bool featureSet = false;
	logToFile = false;
	classifier = 0;
	featureSets = 0;

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
		if ((*optionIter).first == "allstats") {
			value >> gShowAllStats;
		}
		else if((*optionIter).first == "record") {
			gMakeShadowRec = true;
			value >> gShadowRecFilename;
		}
		else if((*optionIter).first == "database") {
			databaseRootSet = true;
			value >> gDatabaseRoot;
		}
		else if((*optionIter).first == "trackreader") {
			trackReader = true;
			value >> gTrackReader;
		}
		else if((*optionIter).first == "features") {
			featureSet = true;
			string featureFileName;
			value >> featureFileName;
			featureSets = new StateConfig(featureFileName);
		}
		else if((*optionIter).first == "logFile") {
			logToFile = true;
			value >> logFilename;
		}
		else {
			throw Util::GenericException("unrecognized option \"" + Util::toString((*optionIter).first) + "\" given to footstepAI module.");
		}
	}

	if(!databaseRootSet) {
		throw GenericException("did not set the database filename, use \"database=filename\".");
	}

	if(!trackReader) {
		throw GenericException("did not select a file for the track reader, use \"trackreader=filename\".");
	}

	track_reader = new TrackReader(gTrackReader);

	classifier = new TreeInterface(gDatabaseRoot);
	
	bool actionsLoaded = classifier->loadActions();
	bool treesLoaded = classifier->loadModels();
	if(!actionsLoaded && !treesLoaded) {
		throw GenericException("Neither the trees or actions could be loaded, make sure the database files are okay.");
	}
	else if(!actionsLoaded) {
		throw GenericException("The actions could not be loaded, make sure the \".actions\" file is okay.");
	}
	else if(!treesLoaded) {
		throw GenericException("The trees could be loaded, make sure the \".names\" and \".tree\" files are okay.");
	}
	else {
		cout << "Trees Loaded" << endl;
	}

	if(!featureSet) {
		featureSets = new StateConfig();
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
	gPhaseProfilers->contextFeatureProfiler.reset();
	gPhaseProfilers->specializedFeatureProfiler.reset();

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
}

void ShadowAIModule::preprocessFrame(float timeStamp, float dt, unsigned int frameNumber) {
	//the module can iterate over all agents and wake up those who should run
	for(vector<ShadowAgent*>::iterator iter = agentLinks.begin(); iter != agentLinks.end(); ++iter) {
		ShadowAgent* deref = *iter;	//otherwise i'm fighting two-step indirection the whole time
		if(!deref->hasRun && timeStamp >= deref->wakeTime) {	//ShadowAgent and ShadowAIModule are friend classes, that means they can touch private parts
			deref->justWoke = true;
			deref->_enabled = true;
			deref->hasRun = true;
		}
	}
}

void ShadowAIModule::postprocessSimulation()
{
	// UNCOMMENT THIS IF YOU WANT TO RECORD A FOOTSTEP REC FILE.
	if (gMakeShadowRec) {
		size_t numAgents = gEngineInfo->getAgents().size();
		size_t numObstacles = gEngineInfo->getObstacles().size();
		ShadowRecWriter srw(static_cast<unsigned int>(numAgents), static_cast<unsigned int>(numObstacles));
		for (unsigned int i=0; i < numAgents; i++) {
			ShadowAgent * agent = dynamic_cast<ShadowAgent*>(gEngineInfo->getAgents()[i]);
			for (unsigned int step=0; step < agent->_stepHistory.size(); step++) {
				srw.addFootstep(i, agent->_stepHistory[step], 0);
			}
		}
		
		unsigned int obstacleID = 0;
		for(set<SteerLib::ObstacleInterface*>::const_iterator iter = gEngineInfo->getObstacles().cbegin(); iter != gEngineInfo->getObstacles().cend(); ++iter) {
			AxisAlignedBox currObstacle = (*iter)->getBounds();
			srw.addObstacleInfo(obstacleID, currObstacle.xmin, currObstacle.xmax, currObstacle.zmin, currObstacle.zmax);
			obstacleID++;
		}
		srw.writeToFile(gShadowRecFilename);
	}
}

//
// finish()
//
void ShadowAIModule::finish()
{
	/*
	StepData s;
	s.step.startTime = 1.201f;
	s.step.endTime = 3.4f;
	s.step.energyCost = 5.6f;
	s.step.isAGoalState = false;
	s.step.outputCOMState.x  = 1.0f;
	s.step.outputCOMState.z  = 2.0f;
	s.step.outputCOMState.dx = 3.0f;
	s.step.outputCOMState.dz = 4.0f;
	s.lookAtObject = 12345;

	//unsigned int numAgents = gEngineInfo->getAgents().size();
	//FootRecWriter frw(numAgents);
	//for (unsigned int i=0; i<numAgents; i++) {
	//	for (unsigned int 
	//}

	unsigned int numAgents = 2;
	FootRecWriter frw(numAgents);

	s.step.startTime = 1.201f;
	frw.addFootstep(0, s);
	s.step.startTime = 1.202f;
	frw.addFootstep(0, s);
	s.step.startTime = 1.203f;
	frw.addFootstep(0, s);

	s.step.startTime = 1.204f;
	frw.addFootstep(1, s);
	s.step.startTime = 1.205f;
	frw.addFootstep(1, s);
	s.step.startTime = 1.206f;
	frw.addFootstep(1, s);
	s.step.startTime = 1.207f;
	s.step.outputCOMState.x = 3.123f;
	frw.addFootstep(1, s);
	s.step.startTime = 1.208f;
	frw.addFootstep(1, s);

	frw.writeToFile("test.foot");

	FootRecReader frr("test.foot");
	std::cerr << "FOOT REC FILE:\n";
	std::cerr << " - num agents: " << frr.getNumAgents() << "\n";
	for (unsigned int i=0; i<frr.getNumAgents(); i++) {
		std::cerr << " - AGENT " << i << ":\n";
		for (unsigned int j=0; j<frr.getNumStepsForAgent(i); j++) {
			std::cerr << "     " << frr.getStepData(i,j).step << "     lookAtObject = " << frr.getStepData(i,j).lookAtObject << "\n";
		}
	}
	*/

	// TODO: de-allocate agents here.
	delete gEnvironmentForAStar;

	if (gShowStats || gShowAllStats) {
		cout << "\n";
		if (gShowAllStats) {
			cout << "===================================================\n";
			cout << "PROFILE RESULTS  " << endl;
			cout << "===================================================\n";

			cout << "--- Long-term planning ---\n";
			gPhaseProfilers->longTermPhaseProfiler.displayStatistics(cout);
			cout << endl;

			cout << "--- Mid-term planning ---\n";
			gPhaseProfilers->midTermPhaseProfiler.displayStatistics(cout);
			cout << endl;

			cout << "--- Short-term planning ---\n";
			gPhaseProfilers->shortTermPhaseProfiler.displayStatistics(cout);
			cout << endl;

			cout << "--- Perceptive phase ---\n";
			gPhaseProfilers->perceptivePhaseProfiler.displayStatistics(cout);
			cout << endl;

			//cout << "--- Predictive phase ---\n";
			//gPhaseProfilers->predictivePhaseProfiler.displayStatistics(cout);
			//cout << endl;

			//cout << "--- Reactive phase ---\n";
			//gPhaseProfilers->reactivePhaseProfiler.displayStatistics(cout);
			//cout << endl;

			cout << "--- Steering phase ---\n";
			gPhaseProfilers->locomotionPhaseProfiler.displayStatistics(cout);
			cout << endl;


			/*
			cout << "--- ESTIMATED REFERENCE (excluding long-term and space-time planning) ---\n";
			cout << "\n(NOTE: this is not rigorously valid to compare against amortized costs below,\n";
			cout << "         because it excludes space-time planning)\n\n";
			float totalAgentTime =
				//gSpaceTimePhaseProfiler.getAverageExecutionTime() +
				gPhaseProfilers->midTermPhaseProfiler.getAverageExecutionTime() + 
				gPhaseProfilers->shortTermPhaseProfiler.getAverageExecutionTime() +
				gPhaseProfilers->perceptivePhaseProfiler.getAverageExecutionTime() +
				gPhaseProfilers->predictivePhaseProfiler.getAverageExecutionTime() +
				gPhaseProfilers->reactivePhaseProfiler.getAverageExecutionTime() +
				gPhaseProfilers->steeringPhaseProfiler.getAverageExecutionTime();
			float totalAgentTime_5Hz_decimation =   // 5 Hz skips every 4 frames, so scale by 0.25
				//gSpaceTimePhaseProfiler.getAverageExecutionTime() * 0.25f +
				gPhaseProfilers->midTermPhaseProfiler.getAverageExecutionTime() * 0.25f + 
				gPhaseProfilers->shortTermPhaseProfiler.getAverageExecutionTime() * 0.25f +
				gPhaseProfilers->perceptivePhaseProfiler.getAverageExecutionTime() * 0.25f +
				gPhaseProfilers->predictivePhaseProfiler.getAverageExecutionTime() * 0.25f +
				gPhaseProfilers->reactivePhaseProfiler.getAverageExecutionTime() +  // reactive and steering phases still execute 20 Hz.
				gPhaseProfilers->steeringPhaseProfiler.getAverageExecutionTime();
			float totalAgentTime_4Hz_decimation =    // 4 Hz skips every 5 frames, so scale by 0.2
				//gPhaseProfilers->SpaceTimePhaseProfiler.getAverageExecutionTime() * 0.2f  +
				gPhaseProfilers->midTermPhaseProfiler.getAverageExecutionTime() * 0.2f + 
				gPhaseProfilers->shortTermPhaseProfiler.getAverageExecutionTime() * 0.2f +
				gPhaseProfilers->perceptivePhaseProfiler.getAverageExecutionTime() * 0.2f +
				gPhaseProfilers->predictivePhaseProfiler.getAverageExecutionTime() * 0.2f +
				gPhaseProfilers->reactivePhaseProfiler.getAverageExecutionTime() +  // reactive and steering phases still execute 20 Hz.
				gPhaseProfilers->steeringPhaseProfiler.getAverageExecutionTime();

			//cout << " percent space-time: " << gPhaseProfilers->SpaceTimePhaseProfiler.getAverageExecutionTime()/totalAgentTime * 100.0<< "\n";
			cout << " percent mid-term:   " << gPhaseProfilers->midTermPhaseProfiler.getAverageExecutionTime()/totalAgentTime * 100.0<< "\n";
			cout << " percent short-term: " << gPhaseProfilers->shortTermPhaseProfiler.getAverageExecutionTime()/totalAgentTime * 100.0<< "\n";
			cout << " percent perceptive: " << gPhaseProfilers->perceptivePhaseProfiler.getAverageExecutionTime()/totalAgentTime * 100.0 << "\n";
			cout << " percent predictive: " << gPhaseProfilers->predictivePhaseProfiler.getAverageExecutionTime()/totalAgentTime * 100.0 << "\n";
			cout << " percent reactive:   " << gPhaseProfilers->reactivePhaseProfiler.getAverageExecutionTime()/totalAgentTime * 100.0 << "\n";
			cout << " percent steering:   " << gPhaseProfilers->steeringPhaseProfiler.getAverageExecutionTime()/totalAgentTime * 100.0 << "\n";
			cout << "\n";
			cout << " Average per agent, no decimation: " << totalAgentTime * 1000.0 << " milliseconds\n";
			cout << " Average per agent, 5Hz (skip 4 frames): " << totalAgentTime_5Hz_decimation * 1000.0 << " milliseconds\n";
			cout << " Average per agent, 4Hz (skip 5 frames): " << totalAgentTime_4Hz_decimation * 1000.0 << " milliseconds\n";
			cout << "\n";
			*/
		}
		/*
		cout << "--- PROFILE RESULTS (excluding long-term planning) ---\n\n";
		float totalTimeForAllAgents =
			gPhaseProfilers->spaceTimePhaseProfiler.getTotalTime() +
			gPhaseProfilers->midTermPhaseProfiler.getTotalTime()+
			gPhaseProfilers->shortTermPhaseProfiler.getTotalTime() +
			gPhaseProfilers->perceptivePhaseProfiler.getTotalTime() +
			gPhaseProfilers->predictivePhaseProfiler.getTotalTime() +
			gPhaseProfilers->reactivePhaseProfiler.getTotalTime() +
			gPhaseProfilers->steeringPhaseProfiler.getTotalTime();

		// TODO: right now this is hacked, later on need to add an arg or access to the engine to get this value correctly:
		cerr << " TODO: 20 frames per second is a hard-coded assumption in the following calculations\n";
		float baseFrequency = 20.0f;
		float totalNumberOfFrames = (float)gPhaseProfilers->steeringPhaseProfiler.getNumTimesExecuted();
		cout << " average frequency space-time: " << gPhaseProfilers->spaceTimePhaseProfiler.getNumTimesExecuted()/totalNumberOfFrames * baseFrequency << " Hz (skipping " << totalNumberOfFrames/((float)gPhaseProfilers->spaceTimePhaseProfiler.getNumTimesExecuted()) << " frames)\n";
		cout << " average frequency mid-term:   " << gPhaseProfilers->midTermPhaseProfiler.getNumTimesExecuted()/totalNumberOfFrames * baseFrequency << " Hz (skipping " << totalNumberOfFrames/((float)gPhaseProfilers->midTermPhaseProfiler.getNumTimesExecuted()) << " frames)\n";
		cout << " average frequency short-term: " << gPhaseProfilers->shortTermPhaseProfiler.getNumTimesExecuted()/totalNumberOfFrames * baseFrequency << " Hz (skipping " << totalNumberOfFrames/((float)gPhaseProfilers->shortTermPhaseProfiler.getNumTimesExecuted()) << " frames)\n";
		cout << " average frequency perceptive: " << gPhaseProfilers->perceptivePhaseProfiler.getNumTimesExecuted()/totalNumberOfFrames * baseFrequency << " Hz (skipping " << totalNumberOfFrames/((float)gPhaseProfilers->perceptivePhaseProfiler.getNumTimesExecuted()) << " frames)\n";
		cout << " average frequency predictive: " << gPhaseProfilers->predictivePhaseProfiler.getNumTimesExecuted()/totalNumberOfFrames * baseFrequency << " Hz (skipping " << totalNumberOfFrames/((float)gPhaseProfilers->predictivePhaseProfiler.getNumTimesExecuted()) << " frames)\n";
		cout << " average frequency reactive:   " << gPhaseProfilers->reactivePhaseProfiler.getNumTimesExecuted()/totalNumberOfFrames * baseFrequency << " Hz (skipping " << totalNumberOfFrames/((float)gPhaseProfilers->reactivePhaseProfiler.getNumTimesExecuted()) << " frames)\n";
		cout << " average frequency steering:   " << gPhaseProfilers->steeringPhaseProfiler.getNumTimesExecuted()/totalNumberOfFrames * baseFrequency << " Hz (skipping " << totalNumberOfFrames/((float)gPhaseProfilers->steeringPhaseProfiler.getNumTimesExecuted()) << " frames)\n";
		cout << "\n";

		cout << " amortized percent space-time: " << gPhaseProfilers->spaceTimePhaseProfiler.getTotalTime()/totalTimeForAllAgents * 100.0 << "\n";
		cout << " amortized percent mid-term:   " << gPhaseProfilers->midTermPhaseProfiler.getTotalTime()/totalTimeForAllAgents * 100.0 << "\n";
		cout << " amortized percent short-term: " << gPhaseProfilers->shortTermPhaseProfiler.getTotalTime()/totalTimeForAllAgents * 100.0 << "\n";
		cout << " amortized percent perceptive: " << gPhaseProfilers->perceptivePhaseProfiler.getTotalTime()/totalTimeForAllAgents * 100.0 << "\n";
		cout << " amortized percent predictive: " << gPhaseProfilers->predictivePhaseProfiler.getTotalTime()/totalTimeForAllAgents * 100.0 << "\n";
		cout << " amortized percent reactive:   " << gPhaseProfilers->reactivePhaseProfiler.getTotalTime()/totalTimeForAllAgents * 100.0 << "\n";
		cout << " amortized percent steering:   " << gPhaseProfilers->steeringPhaseProfiler.getTotalTime()/totalTimeForAllAgents * 100.0 << "\n";
		cout << " AVERAGE PER AGENT WITH DECIMATION: " << totalTimeForAllAgents / ((float)gPhaseProfilers->steeringPhaseProfiler.getNumTimesExecuted()) * 1000.0 << " milliseconds\n";
		*/

		cout << "--- Footstep planning ---\n";
		gPhaseProfilers->footstepPlanningPhaseProfiler.displayStatistics(cout);
		cout << endl;

		cout << "--- TOTAL AI ---\n";
		gPhaseProfilers->aiProfiler.displayStatistics(cout);
		cout << endl;

		/*	
		cout << "--- Context Feature Vector Gen ---\n";
		gPhaseProfilers->contextFeatureProfiler.displayStatistics(cout);
		cout << endl;
		*/

		/*	
		cout << "--- Specialized Feature Vector Gen ---\n";
		gPhaseProfilers->specializedFeatureProfiler.displayStatistics(cout);
		cout << endl;
		*/

		if(logToFile) {
			ifstream logCheck(logFilename + ".xls");
			ofstream logOut;

			//check if the file already exists
			if(!logCheck.is_open()) {
				logCheck.close();
				logOut.open(logFilename + ".xls");
				
				//Write out the header, which consists of column labels
				logOut << "Source\t" << "Execution Count (#)\t" << "Fastest Time (ms)\t" << "Slowest Time (ms)\t" << "Average Time (ms)\t" << "Total Time (s)\t" << "Label\n";
			}
			else {
				logCheck.close();
				logOut.open(logFilename + ".xls", ios::app);

				//add some whitespace to differentiate runs in the .xls file, which can later be sorted/split in Excel
				logOut << '\n';
			}

			//Write out footstep planning stats
			logOut << "ShadowAI Decisions:" << '\t';
			logOut << gPhaseProfilers->footstepPlanningPhaseProfiler.getNumTimesExecuted() << '\t';
			logOut << gPhaseProfilers->footstepPlanningPhaseProfiler.getMinExecutionTime() * 1000.0f << '\t';
			logOut << gPhaseProfilers->footstepPlanningPhaseProfiler.getMaxExecutionTime() * 1000.0f << '\t';
			logOut << gPhaseProfilers->footstepPlanningPhaseProfiler.getAverageExecutionTime() * 1000.0f << '\t';
			logOut << gPhaseProfilers->footstepPlanningPhaseProfiler.getTotalTime() << '\t';
			logOut << gEngineInfo->getOptions().moduleOptionsDatabase.at("testCasePlayer").at("testcase") << '\n';

			//Write out the "totalAI" stats
			logOut << "ShadowAI Total AI:" << '\t';
			logOut << gPhaseProfilers->aiProfiler.getNumTimesExecuted() <<'\t';
			logOut << gPhaseProfilers->aiProfiler.getMinExecutionTime() * 1000.0f << '\t';
			logOut << gPhaseProfilers->aiProfiler.getMaxExecutionTime() * 1000.0f << '\t';
			logOut << gPhaseProfilers->aiProfiler.getAverageExecutionTime() * 1000.0f << '\t';
			logOut << gPhaseProfilers->aiProfiler.getTotalTime() << '\t';
			logOut << gEngineInfo->getOptions().moduleOptionsDatabase.at("testCasePlayer").at("testcase") << '\n';

			logOut.close();

			//see if the auxiliary datafile exists
			stringstream auxLogName;
			logCheck.open(logFilename + "_Aux.xls");
			
			//open for writing accordingly
			if(!logCheck.is_open()) {
				//create header
				logCheck.close();
				logOut.open(logFilename + "_Aux.xls");
			
				//write column tops for excel
				logOut << "Source\t" << "Execution Count (#)\t" << "Model Step Used (#)\t" << "A* Invoked (#)\t" << "Fastest Time (ms)\t" << "Slowest Time (ms)\t" << "Average Time (ms)\t" << "Total Time (s)\t" << "Label\n";
			}
			else {
				//just appending
				logCheck.close();
				logOut.open(logFilename + "_Aux.xls", ios::app);
				
				logOut << '\n';	//whitespace to separate runs
			}
			
			//output auxiliary performance data
			logOut << "Context Vector Generation:" << '\t';
			logOut << gPhaseProfilers->contextFeatureProfiler.getNumTimesExecuted() <<'\t';
			logOut << ShadowAgent::numData << '\t';
			logOut << ShadowAgent::numPlanner << '\t';
			logOut << gPhaseProfilers->contextFeatureProfiler.getMinExecutionTime() * 1000.0f << '\t';
			logOut << gPhaseProfilers->contextFeatureProfiler.getMaxExecutionTime() * 1000.0f << '\t';
			logOut << gPhaseProfilers->contextFeatureProfiler.getAverageExecutionTime() * 1000.0f << '\t';
			logOut << gPhaseProfilers->contextFeatureProfiler.getTotalTime() << '\t';
				
			logOut << gEngineInfo->getOptions().moduleOptionsDatabase.at("testCasePlayer").at("testcase") << "_" << gDatabaseRoot << "_ALL\n";
			
			logOut << "Specialized Vector Generation:" << '\t';
			logOut << gPhaseProfilers->specializedFeatureProfiler.getNumTimesExecuted() <<'\t';
			logOut << ShadowAgent::numData << '\t';
			logOut << ShadowAgent::numPlanner << '\t';
			logOut << gPhaseProfilers->specializedFeatureProfiler.getMinExecutionTime() * 1000.0f << '\t';
			logOut << gPhaseProfilers->specializedFeatureProfiler.getMaxExecutionTime() * 1000.0f << '\t';
			logOut << gPhaseProfilers->specializedFeatureProfiler.getAverageExecutionTime() * 1000.0f << '\t';
			logOut << gPhaseProfilers->specializedFeatureProfiler.getTotalTime() << '\t';
				
			logOut << gEngineInfo->getOptions().moduleOptionsDatabase.at("testCasePlayer").at("testcase") << "_" << gDatabaseRoot << "_ALL\n";
			
			logOut.close();
		}
	}
}


void ShadowAIModule::draw()
{
}

void ShadowAIModule::initializeSimulation()
{
}



PLUGIN_API SteerLib::ModuleInterface * createModule() { return new ShadowAIModule; }

PLUGIN_API void destroyModule( SteerLib::ModuleInterface*  module ) { if (module) delete module; module = NULL; }

ShadowAIModule::~ShadowAIModule() {
	if(classifier) {
		delete classifier;
	}

	if(featureSets) {
		delete featureSets;
	}

	if(track_reader) {
		delete track_reader;
	}
}