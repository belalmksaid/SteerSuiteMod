//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


/// @file steertrain/src/Main.cpp
/// @brief Entry point of SteerTrain.
///

#include "SteerLib.h"
#include "c5gpl.h"
#include <vector>
#include <fstream>

using namespace std;
using namespace Util;

int main(int argc, char** argv)
{
	//performance logging variables, using try-block's scope for profiling
	//PerformanceProfiler trainProfiler;
	//trainProfiler.reset();
	//bool logPerformance = false;
	//string logFile;
	//string databaseRoot;

	try {
		CommandLineParser* cp = new CommandLineParser();

		// options initialized with defaults, which can be overridden by command line arguments
		bool boosting = false;
		bool boostingN = false;
		bool useSubsets = false;
		bool softThresholds = false;
		bool winnowing = false;
		bool xValidate = false;
		unsigned int boostCount = 0;
		unsigned int folds = 0;
		bool noPruning = false;
		int verbosity = 0;

		//**required options**
		string databaseRoot;
		bool databaseRootSet = false;
		unsigned int maxContext; //this is pulled from a file, not the command line
		
		cp->addOption("-database", &databaseRoot, OPTION_DATA_TYPE_STRING, 1, &databaseRootSet, true);
		cp->addOption("-boost", NULL, OPTION_DATA_TYPE_NO_DATA, 0, &boosting, true);
		cp->addOption("-boostN", &boostCount, OPTION_DATA_TYPE_UNSIGNED_INT, 1, &boostingN, true);
		cp->addOption("-subsets", NULL, OPTION_DATA_TYPE_NO_DATA, 0, &useSubsets, true);
		cp->addOption("-soft", NULL, OPTION_DATA_TYPE_NO_DATA, 0, &softThresholds, true);
		cp->addOption("-winnow", NULL, OPTION_DATA_TYPE_NO_DATA, 0, &winnowing, true);
		cp->addOption("-crossVal", &folds, OPTION_DATA_TYPE_UNSIGNED_INT, 1, &xValidate, true);
		//cp->addOption("-performance", &logFile, OPTION_DATA_TYPE_STRING, 1, &logPerformance, true);	//these variables are defined outside the try-block's scope
		cp->addOption("-disablePruning", NULL, OPTION_DATA_TYPE_NO_DATA, 0, &noPruning, true);
		cp->addOption("-verbosity", &verbosity, OPTION_DATA_TYPE_SIGNED_INT, 1, NULL, true);

		//skip the first argument, and dump the rest of the command line arguments in recFilesToExtract, since they should just be filenames and aren't options
		cp->parse(argc, argv, true, true);

		//check the required parameters are set AND valid
		if(!databaseRootSet) {
			throw GenericException("Did not give a base name for the data to train on, expected \"-database NUMBER\".");
		}
		else {
			if(!fileCanBeOpened(databaseRoot + ".metaData")) {
				throw GenericException("The .metaData file for the database " + databaseRoot + " is missing, check your paths.");
			}
			else {
				//iterate from 0 to the maximal context number in order to check all the files are there
				ifstream metaReader(databaseRoot + ".metaData");
				metaReader >> maxContext;

				for(unsigned int i = 0; i <= maxContext; i++) {
					stringstream specializedRoot;
					specializedRoot << databaseRoot << "_" << i;
					string specializedLeftString = specializedRoot.str() + "l";
					string specializedRightString = specializedRoot.str() + "r";
					
					//check left .actions
					if(!fileCanBeOpened(specializedLeftString + ".actions")) {
						throw GenericException("The left .actions file for the context " + i + string(" is missing, check your paths."));
					}
					else if(!fileCanBeOpened(specializedLeftString + ".names")) {
						throw GenericException("The left .names file for the context " + i + string(" is missing, check your paths."));
					}
					else if(!fileCanBeOpened(specializedLeftString + ".data")) {
						throw GenericException("The left .data file for the context " + i + string(" is missing, check your paths."));
					}
					else if(!fileCanBeOpened(specializedRightString + ".actions")) {
						throw GenericException("The right .actions file for the context " + i + string(" is missing, check your paths."));
					}
					else if(!fileCanBeOpened(specializedRightString + ".names")) {
						throw GenericException("The right .names file for the context " + i + string(" is missing, check your paths."));
					}
					else if(!fileCanBeOpened(specializedRightString + ".data")) {
						throw GenericException("The right .data file for the context " + i + string(" is missing, check your paths."));
					}
				}

				//check context names
				if(!fileCanBeOpened(databaseRoot + "_C.names")) {
					throw GenericException("The .names file for the database " + databaseRoot + " is missing, check your paths.");
				}
				//check context .data
				else if(!fileCanBeOpened(databaseRoot + "_C.data")) {
					throw GenericException("The .data file for the database " + databaseRoot + " is missing, check your paths.");
				}
			}
		}

		vector<char*> c5arguments;
		c5arguments.push_back(argv[0]);

		if(boosting && boostingN) {
			throw GenericException("Both boosting options were given; pick one.");
		}

		if(boosting) {
			c5arguments.push_back("-b");
		}

		if(boostingN) {
			c5arguments.push_back("-t");
			char* buffer = new char[10];	//yep, this'll leak
			sprintf(buffer, "%i", boostCount);
			c5arguments.push_back(buffer);
		}

		if(winnowing) {
			c5arguments.push_back("-w");
		}

		if(softThresholds) {
			c5arguments.push_back("-p");
		}

		if(useSubsets) {
			c5arguments.push_back("-s");
		}

		if(noPruning) {
			c5arguments.push_back("-g");
		}

		if(xValidate) {
			if(folds == 0) {
				throw GenericException("Either did not give a fold-count for cross validation, or it was 0.");
			}
			c5arguments.push_back("-X");
			char* buffer = new char[10];	//yep, this'll leak
			sprintf(buffer, "%i", folds);
			c5arguments.push_back(buffer);
			//logPerformance = false;
			//cout << "Will not log cross-validation performance, ignoring \"-performance\" flag." << endl;
		}

		c5arguments.push_back("-f");
		C5Engine* trainer = new C5Engine[(maxContext+1) * 2];
		//should be .data and .names files for each i from 0 to maxContext
		#pragma omp parallel for firstprivate(c5arguments)
		for(int i = 0; i <= maxContext; i++) {
			//AutomaticFunctionProfiler profileThisFunction(&trainProfiler);

			stringstream specializedRoot;
			specializedRoot << databaseRoot << "_" << i;
			string specializedLeftString = specializedRoot.str() + "l";
			string specializedRightString = specializedRoot.str() + "r";
		
			c5arguments.push_back(const_cast<char*>(specializedLeftString.c_str()));

			trainer[i].c5train(static_cast<int>(c5arguments.size()), &c5arguments[0], verbosity);
			
			c5arguments.pop_back();

			c5arguments.push_back(const_cast<char*>(specializedRightString.c_str()));

			trainer[i+(maxContext+1)].c5train(static_cast<int>(c5arguments.size()), &c5arguments[0], verbosity);

			c5arguments.pop_back();
		}

		cout.flush();
		cerr.flush();

		cout << "----------------------------------------------------------------------------------------------" << endl;
		cout << "**********************************************************************************************" << endl;
		cout << "----------------------------------------------------------------------------------------------" << endl;

		for(int i = 0; i <= maxContext; i++) {
			cout << "\nContext " << i << " left side.\n\n";
			trainer[i].Evaluate(1);
			cout << "\nContext " << i << " right side.\n\n";
			trainer[i+(maxContext+1)].Evaluate(1);
		}

		cout.flush();

		//profiler run differently here so that deleting the CommandLineParser does not add overhead to the measurement
		//trainProfiler.start();

		C5Engine cTrainer;

		//now train the top-level classifier
		string contextClassifierName = databaseRoot + "_C";

		c5arguments.push_back(const_cast<char*>(contextClassifierName.c_str()));

		cTrainer.c5train(static_cast<int>(c5arguments.size()), &c5arguments[0], verbosity);

		//end profiler before deleting CommandLineParser
		//trainProfiler.stop();

		delete cp;
	}	
	catch (exception &e) {
		cerr << "\nERROR: exception caught in main:\n" << e.what() << "\n";
		exit(1);
	}

	/* Consider this "deprecated" and to be removed in the near future, after i formally decide we don't care about the performance of this program
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
			logOut << "Hierarchical Learning:" << '\t';
			logOut << trainProfiler.getNumTimesExecuted() << '\t';
			logOut << trainProfiler.getMinExecutionTime() * 1000.0f << '\t';
			logOut << trainProfiler.getMaxExecutionTime() * 1000.0f << '\t';
			logOut << trainProfiler.getAverageExecutionTime() * 1000.0f << '\t';
			logOut << trainProfiler.getTotalTime() << '\t';
			logOut << databaseRoot << '\n';
		}
		else {
			logCheck.close();
			logOut.open(logFile + ".xls");

			//Write out the header, which consists of column labels
			logOut << "Source\t" << "Execution Count (#)\t" << "Fastest Time (ms)\t" << "Slowest Time (ms)\t" << "Average Time (ms)\t" << "Total Time (s)\t" << "Label\n";

			//Write out footstep planning stats
			logOut << "Hierarchical Learning:" << '\t';
			logOut << trainProfiler.getNumTimesExecuted() << '\t';
			logOut << trainProfiler.getMinExecutionTime() * 1000.0f << '\t';
			logOut << trainProfiler.getMaxExecutionTime() * 1000.0f << '\t';
			logOut << trainProfiler.getAverageExecutionTime() * 1000.0f << '\t';
			logOut << trainProfiler.getTotalTime() << '\t';
			logOut << databaseRoot << '\n';
		}
		logOut.close();
	} */

	cerr.flush();
	cout.flush();

	return EXIT_SUCCESS;
}