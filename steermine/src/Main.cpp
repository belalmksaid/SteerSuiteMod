//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


/// @file steermine/src/Main.cpp
/// @brief Entry point of SteerMine.

//For now, this program will be used to generate various statistics and will be added on as other statistics become evident
//Starting with a a stepped-integer histogram of scores

#include "SteerLib.h"
#include "shadowrec/ShadowRecIO.h"
#include <fstream>
#include <cmath>

using namespace std;
using namespace Util;

int main(int argc, char** argv)
{
	try {
		ofstream fileOut;

		CommandLineParser* cp = new CommandLineParser();

		// options initialized with defaults, which can be overridden by command line arguments
		string directory;
		bool directoryWasSet = false;
		bool overwrite = false;		//default to append
		string runTitle;
		bool isRunTitleSet = false;
		
		//**required options**
		vector<char*> recFilesToMine;	//Should be similar to SteerBench
		string outputFile;
		bool outputIsSet = false;

		cp->addOption("-directory", &directory, OPTION_DATA_TYPE_STRING, 1, &directoryWasSet, true);
		cp->addOption("-outputFile", &outputFile, OPTION_DATA_TYPE_STRING, 1, &outputIsSet, true);
		cp->addOption("-outputfile", &outputFile, OPTION_DATA_TYPE_STRING, 1, &outputIsSet, true);
		cp->addOption("-overwrite", NULL, OPTION_DATA_TYPE_NO_DATA, 0, &overwrite, true);
		cp->addOption("-title", &runTitle, OPTION_DATA_TYPE_STRING, 1, &isRunTitleSet, true);

		//dump the rest of the command line arguments in recFilesToExtract, since they should just be filenames and aren't options
		cp->parse(argc, argv, true, recFilesToMine);
		vector<string> recFiles;

		if(outputIsSet) {
			if(overwrite) {
				fileOut.open(outputFile, ios::trunc);
			}
			else {
				fileOut.open(outputFile, ios::app);
			}
		}
		else {
			throw new GenericException("Did not set an output file with \"-outputFile\" flag.");
		}
		
		if(directoryWasSet && recFilesToMine.size() == 1) {
			ifstream filenameReader(recFilesToMine[0]);
			
			string line;
			while(filenameReader >> line) {
				recFiles.push_back(directory + line);
			}
			filenameReader.close();
		}
		else if(directoryWasSet) {
			throw new GenericException("You specified the \"-directory\" option but may have given a list of files, only a file listing the filenames in the directory should be provided.");
		}
		else {
			recFiles.push_back(recFilesToMine[0]);
			//throw new GenericException("Didn't set a directory");
		}

		//vector<unsigned int> cumulativeHistogram(0);
		//vector<unsigned int> instantHistogram(0);	//TODO: implement this, shouldn't be too hard just have to fiddle with the math a bit
		int anomCounter = 0;
		int population;

		//iterate over the files
		//for(vector<string>::const_iterator citer = recFiles.cbegin(); citer != recFiles.cend(); ++citer) {
		//	ShadowRecReader reader(*citer);
		//	//for each agent
		//	for(unsigned int i = 0; i < reader.getNumAgents(); i++) {
		//		//for each step
		//		for(unsigned int j = 0; j < reader.getNumStepsForAgent(i); j++) {
		//			//increment floor of score, expanding histogram if necessary to accomodate the extra counters
		//			int score = static_cast<int>(floorf(reader.getStepData(i, j).step.cumulativeScore) / 30.0f);
		//			if(score >= cumulativeHistogram.size()) {
		//				cumulativeHistogram.resize(score + 1, 0);
		//			}

		//			cumulativeHistogram[score]++;
		//		}
		//	}
		//}

		for(vector<string>::const_iterator citer = recFiles.cbegin(); citer != recFiles.cend(); ++citer) {
			ShadowRecReader reader(*citer);
			//for each agent
			population = reader.getNumAgents();
			for(unsigned int i = 0; i < reader.getNumAgents(); i++) {
				//for each step
				for(unsigned int j = 1; j < reader.getNumStepsForAgent(i); j++) {
					//increment floor of score, expanding histogram if necessary to accomodate the extra counters
					if(reader.getStepData(i, j).step.isAnomaly && !reader.getStepData(i, j - 1).step.isAnomaly) {
						anomCounter++;
					}
				}
			}
		}

		//print title if one was supplied, else print a blank line for spacing
		if(isRunTitleSet) {
			fileOut << runTitle << '\t';
		}
		else {
			//fileOut << '\n';
		}

		fileOut << anomCounter << "\t";

		fileOut << population << "\n";

		//print histogram bin info
		//for(int i = 0; i < cumulativeHistogram.size(); i++) {
		//	if(i == cumulativeHistogram.size() - 1) {
		//		fileOut << "[" << i * 30 << ", " << (i + 1) * 30 << ")" << '\n';
		//	}
		//	else {
		//		fileOut << "[" << i * 30 << ", " << (i + 1) * 30 << ")" << '\t';
		//	}
		//}

		////print the data
		//for(int i = 0; i < cumulativeHistogram.size(); i++) {
		//	if(i == cumulativeHistogram.size() - 1) {
		//		fileOut << cumulativeHistogram[i] << '\n';
		//	}
		//	else {
		//		fileOut << cumulativeHistogram[i] << '\t';
		//	}
		//}
		fileOut.close();
	}
	catch (GenericException ex) {
		cerr << "Exception thrown: " << ex.what() << endl;
		cerr.flush();
		cout.flush();
		return EXIT_FAILURE;
	}

	cerr.flush();
	cout.flush();

	return EXIT_SUCCESS;
}
