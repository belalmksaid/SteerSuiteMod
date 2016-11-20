//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


/// @file steergen/src/Main.cpp
/// @brief Entry point of SteerGen.
///

#include "SteerLib.h"
#include "StateConfig.h"
#include "GenEngine.h"
#include "footrec/FootRecIO.h"
#include <fstream>

using namespace std;
using namespace Util;

int main(int argc, char** argv)
{
	//performance logging variables, using try-block's scope for profiling
	PerformanceProfiler genProfiler;
	genProfiler.reset();
	bool logPerformance = false;
	string logFile;
	string databaseFilenameRoot;
	unsigned int contextNumber;

	//StateXMLReader stateReader;

	try {
		CommandLineParser* cp = new CommandLineParser();

		// options initialized with defaults, which can be overridden by command line arguments
		//bool validateRecFile = false;			//Validation not yet implemented, but if it is later can uncomment this flag
		//string recSearchPath = "";			//Does not appear to have a clear purpose in SteerBench
		//bool genSample = false;
		//string stateSpaceXML;					//TODO: find where they use tinyxml for parsing, and how to define an XML vocabulary
		//bool stateSpaceXMLSet = false;			//TODO, define proper vocabulary
		unsigned int recFileCount;
		bool restrictFileCount = false;
		string directory;
		bool directoryWasSet = false;
		bool validateFlagSet = false;
		bool cullFlagSet = false;
		
		//**required options**
		bool databaseFilenameSet = false;
		bool contextNumberSet = false;
		vector<char*> recFilesToExtract;	//Should be similar to SteerBench


		//cp->addOption("-validate", NULL, OPTION_DATA_TYPE_NO_DATA, 0, &validateRecFile, true);
		//cp->addOption("-recsearchpath", &recSearchPath, OPTION_DATA_TYPE_STRING);
		//cp->addOption("-recSearchPath", &recSearchPath, OPTION_DATA_TYPE_STRING);
		//cp->addOption("-statespace", &stateSpaceXML, OPTION_DATA_TYPE_STRING, 1, &stateSpaceXMLSet, true);
		//cp->addOption("-stateSpace", &stateSpaceXML, OPTION_DATA_TYPE_STRING, 1, &stateSpaceXMLSet, true);
		//cp->addOption("-sampleXML", NULL, OPTION_DATA_TYPE_NO_DATA, 0, &genSample, true);
		cp->addOption("-database", &databaseFilenameRoot, OPTION_DATA_TYPE_STRING, 1, &databaseFilenameSet, true);
		cp->addOption("-contextNumber", &contextNumber, OPTION_DATA_TYPE_UNSIGNED_INT, 1, &contextNumberSet, true);
		cp->addOption("-performance", &logFile, OPTION_DATA_TYPE_STRING, 1, &logPerformance, true);	//these variables are defined outside the try-block's scope
		cp->addOption("-fileCount", &recFileCount, OPTION_DATA_TYPE_UNSIGNED_INT, 1, &restrictFileCount, true);
		cp->addOption("-directory", &directory, OPTION_DATA_TYPE_STRING, 1, &directoryWasSet, true);
		cp->addOption("-validateMode", NULL, OPTION_DATA_TYPE_NO_DATA, 0, &validateFlagSet, true);
		cp->addOption("-cullActions", NULL, OPTION_DATA_TYPE_NO_DATA, 0, &cullFlagSet, true);

		//dump the rest of the command line arguments in recFilesToExtract, since they should just be filenames and aren't options
		cp->parse(argc, argv, true, recFilesToExtract);
		vector<string> recFiles;
		
		if(directoryWasSet && recFilesToExtract.size() == 1) {
			ifstream filenameReader(recFilesToExtract[0]);
			
			string line;
			while(filenameReader >> line) {
				recFiles.push_back(directory + line);
			}
			filenameReader.close();

			if(restrictFileCount && recFileCount < recFiles.size()) {
				recFiles.resize(recFileCount);
			}
		}
		else if(directoryWasSet) {
			throw new GenericException("You specified the \"-directory\" option but may have given a list of files, only a file listing the filenames in the directory should be provided.");
		}
		else {
			throw new GenericException("Didn't set a directory");
		}

		/*	Once state XML is done, uncomment this section
		if(genSample) {
			//check that the file doesn't already exist, unless the overwrite flag is set
			if(fileCanBeOpened(sampleFile) && !forceOver) {
				throw GenericException("A file with the name \"" + sampleFile + "\" already exists, use \"-overwrite\" flag to force the operation.");
			}
			else {
				stateReader.generateSample(sampleFile);
				return 0;
			}
		}
		
		//verify that all input files are valid
		if(stateSpaceXMLSet) {
			if(!(fileCanBeOpened(stateSpaceXML))) {
				throw GenericException("State Space XML file \"" + stateSpaceXML + "\" cannot be opened, check the path and filename for typos.");
			}
		}		
		*/

		//verify mandatory parameters set, including at least one rec file
		if(!contextNumberSet) {
			throw GenericException("Did not assign a context number to this data, expected \"-contextNumber NUMBER\".");
		}

		if(!databaseFilenameSet) {
			throw GenericException("No database filename for output was provided, expected \"-database FILENAME\".");
		}

		if(recFilesToExtract.size() == 0) {
			throw GenericException("No rec files were given to pull the training data from.");
		}

		//verify that the rec files can all be read, for now fail the whole operation if even one bad filename exists
		for(vector<string>::const_iterator citer = recFiles.cbegin(); citer != recFiles.cend(); ++citer) {
			if(!fileCanBeOpened(*citer)) {
				throw GenericException("Rec file with name \"" + *citer + "\" could not be opened, check that it exists.");
			}
		}

		//create instance of StateConfig class for all iterations, optionally use XML file provided to initialize the state spaces
		StateConfig* configuration = new StateConfig();
		GeneratorEngine* gen = new GeneratorEngine(configuration, contextNumber, databaseFilenameRoot, validateFlagSet);
		
		for(vector<string>::const_iterator citer = recFiles.cbegin(); citer != recFiles.cend(); ++citer) {
			AutomaticFunctionProfiler profileThisFunction(&genProfiler);
			cout << "Processing rec file " << *citer << "..." << endl;
			gen->loadRecFile(*citer);
			gen->extractSamples();
		}

		//need to do our output, of particular interest is how to save out the training data; multiple files may be preferable
		//gen->printStats();
		if(!validateFlagSet && cullFlagSet) {
			gen->shrinkage();
		}
		gen->saveSamples();

		delete gen;
		delete cp;
	}	
	catch (exception &e) {
		cerr << "\nERROR: exception caught in main:\n" << e.what() << "\n";
		cout.flush();
		cerr.flush();
		exit(1);
	}

	if(logPerformance) {
		ifstream logCheck(logFile + ".xls");
		ofstream logOut;

		//check if the file already exists
		if(logCheck.is_open()) {
			logCheck.close();
			logOut.open(logFile + ".xls", ios::app);	//want to append this data to the end of the file
				
			//add some whitespace to differentiate runs in the .xls file, which can later be sorted/split in Excel
			logOut << '\n';

			//Write out footstep planning stats
			logOut << "Example Extraction:" << '\t';
			logOut << genProfiler.getNumTimesExecuted() << '\t';
			logOut << genProfiler.getMinExecutionTime() * 1000.0f << '\t';
			logOut << genProfiler.getMaxExecutionTime() * 1000.0f << '\t';
			logOut << genProfiler.getAverageExecutionTime() * 1000.0f << '\t';
			logOut << genProfiler.getTotalTime() << '\t';
			logOut << databaseFilenameRoot << contextNumber << '\n';
		}
		else {
			logCheck.close();
			logOut.open(logFile + ".xls");

			//Write out the header, which consists of column labels
			logOut << "Source\t" << "Execution Count (#)\t" << "Fastest Time (ms)\t" << "Slowest Time (ms)\t" << "Average Time (ms)\t" << "Total Time (s)\t" << "Label\n";

			//Write out footstep planning stats
			logOut << "Example Extraction:" << '\t';
			logOut << genProfiler.getNumTimesExecuted() << '\t';
			logOut << genProfiler.getMinExecutionTime() * 1000.0f << '\t';
			logOut << genProfiler.getMaxExecutionTime() * 1000.0f << '\t';
			logOut << genProfiler.getAverageExecutionTime() * 1000.0f << '\t';
			logOut << genProfiler.getTotalTime() << '\t';
			logOut << databaseFilenameRoot << contextNumber << '\n';

		}
		logOut.close();
	}

	cerr.flush();
	cout.flush();

	return EXIT_SUCCESS;
}
