//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include "C5Reader/TreeInterface.h"

#include <fstream>
#include <float.h>
#include <sstream>
#include <iostream>
#include <Windows.h>

using namespace std;

TreeInterface::TreeInterface(string fileRoot) {
	filenameRoot = fileRoot;
	ifstream input(fileRoot + ".metaData");

	//as in SteerGen, this bit of code pulls out the highest context number used
	input >> maxContext;
	input.close();
	
	actionMaps[0].resize(++maxContext);
	actionMaps[1].resize(maxContext);
	
	specializedClassifiers[0].resize(maxContext);
	specializedClassifiers[1].resize(maxContext);

	for(int i = 0; i < specializedClassifiers[0].size(); i++) {
		specializedClassifiers[0][i] = new C5Engine();
	}

	for(int i = 0; i < specializedClassifiers[1].size(); i++) {
		specializedClassifiers[1][i] = new C5Engine();
	}

	contextClassifier = new C5Engine();
}

TreeInterface::~TreeInterface() {
	for(int i = 0; i < specializedClassifiers[0].size(); i++) {
		delete specializedClassifiers[0][i];
	}

	for(int i = 0; i < specializedClassifiers[1].size(); i++) {
		delete specializedClassifiers[1][i];
	}

	if(contextClassifier) {
		delete contextClassifier;
	}
}

bool TreeInterface::loadModels() {
	#pragma omp parallel
	{
		#pragma omp for nowait schedule(dynamic) //ordered //using fine-grained work division here because of the varied sizes of trees
		for(int i = 0; i < maxContext; i++)
		{
			stringstream fullFileName;
			string fullNameRoot;
			string fullNameRoot2;
			char* nameChar;
			char* nameChar2;
			
			fullFileName << filenameRoot << "_" << i;
			fullNameRoot = fullFileName.str() + "l";
			nameChar = const_cast<char*>(fullNameRoot.c_str());
			specializedClassifiers[0][i]->c5load(nameChar, false, false);

			fullNameRoot2 = fullFileName.str() + "r";
			nameChar2 = const_cast<char*>(fullNameRoot2.c_str());
			specializedClassifiers[1][i]->c5load(nameChar2, false, false);
		}

		#pragma omp single	//first thread to finish its loop iterations does this
		{
			stringstream fullFileName;
			fullFileName << filenameRoot << "_C";
			string fullNameRoot;
			fullNameRoot = fullFileName.str();
			contextClassifier->c5load(const_cast<char*>(fullNameRoot.c_str()), false, false);
		}
	}
	return true;
}

bool TreeInterface::loadActions() {
	//load the .actions file for each context
	#pragma omp parallel for schedule(static)	//coarse-grained work division since .actions files are roughly the same size
	for(int i = 0; i < maxContext; i++)
	{
		stringstream fullFileName;
		
		fullFileName << filenameRoot << "_" << i << "l.actions";
		ifstream actionGetter(fullFileName.str());

		//construct a stepinfo struct for each entry from the file data
		do {
			char rawLine[100];
			actionGetter.getline(rawLine, 100);
			stringstream tokenizer(rawLine, ios_base::in);
			int junk;

			stepInfo temp;
			tokenizer >> junk;
			tokenizer >> temp.phi;
			tokenizer >> temp.duration;
			tokenizer >> temp.desiredSpeed;
			tokenizer >> temp.footstepEnum;
			
			//store in the proper location of the 2D vector
			actionMaps[0][i].push_back(temp);
		} while(!actionGetter.eof());
		actionGetter.close();

		fullFileName.str("");
		fullFileName << filenameRoot << "_" << i << "r.actions";
		actionGetter.open(fullFileName.str());

		//construct a stepinfo struct for each entry from the file data
		do {
			char rawLine[100];
			actionGetter.getline(rawLine, 100);
			stringstream tokenizer(rawLine, ios_base::in);
			int junk;

			stepInfo temp;
			tokenizer >> junk;
			tokenizer >> temp.phi;
			tokenizer >> temp.duration;
			tokenizer >> temp.desiredSpeed;
			tokenizer >> temp.footstepEnum;
			
			//store in the proper location of the 2D vector
			actionMaps[1][i].push_back(temp);
		} while(!actionGetter.eof());
		actionGetter.close();
	}
	return true;
}

pair<unsigned int, float> TreeInterface::getContext(std::string featureVec) const {
	pair<unsigned int, float> result;
	
	//take the string and construct a DataRec for it
	char* cStyleInput = const_cast<char*>(featureVec.c_str());
	DataRec input = contextClassifier->StringToRec(cStyleInput);
	float confidenceRating;
	unsigned int classifiedValue = static_cast<unsigned int>(contextClassifier->Classify(input, confidenceRating)) - 1;

	result.first = classifiedValue;
	result.second = confidenceRating;

	return result;
}

pair<TreeInterface::stepInfo, float> TreeInterface::getAction(unsigned int context, std::string featureVec, bool whichFoot) const {
	C5Engine* chosenClassifier = specializedClassifiers[whichFoot][context];
	pair<stepInfo, float> result;
	
	//take the string and construct a DataRec for it
	char* cStyleInput = const_cast<char*>(featureVec.c_str());

	DataRec input = chosenClassifier->StringToRec(cStyleInput);
	float confidenceRating;
	unsigned int classifiedValue = static_cast<unsigned int>(chosenClassifier->Classify(input, confidenceRating)) - 1; //Holy Crap, the people that coded C5.0 didn't use
																													  //the 0th element of anything!!
	result.first = actionMaps[whichFoot][context][classifiedValue];
	result.second = confidenceRating;

	//return the corresponding stepInfo from the 2D vector
	return result;
}