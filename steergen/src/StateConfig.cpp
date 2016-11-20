//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

//
// Copyright notice is on the TODO list
//
//

/// @file steergen/src/StateConfig.cpp
/// @brief Implementation of the state space XML reader and configuration storage

#include "StateConfig.h"

using namespace std;
using namespace Util;

StateConfig::StateConfig() {
	//create a default state space for specialized classifiers
	vector<state*> tempVector;
	
	//create slice 0
	
	slice* wedge0 = new slice;
	wedge0->minDist = 0.0f;
	wedge0->maxDist = 10.0f;
	wedge0->firstBound = 0.0f;
	wedge0->secondBound = 22.5f;
	wedge0->agentOnly = true;
	tempVector.push_back(wedge0);

	//create slice 1
	
	slice* wedge1 = new slice;
	wedge1->minDist = 0.0f;
	wedge1->maxDist = 10.0f;
	wedge1->firstBound = 22.5f;
	wedge1->secondBound = 60.0f;
	wedge1->agentOnly = true;
	tempVector.push_back(wedge1);

	//create slice 2
	
	slice* wedge2 = new slice;
	wedge2->minDist = 0.0f;
	wedge2->maxDist = 10.0f;
	wedge2->firstBound = 60.0f;
	wedge2->secondBound = 105.0f;
	wedge2->agentOnly = true;
	tempVector.push_back(wedge2);

	//create slice 3
	
	slice* wedge3 = new slice;
	wedge3->minDist = 0.0f;
	wedge3->maxDist = 10.0f;
	wedge3->firstBound = 105.0f;
	wedge3->secondBound = 155.0f;
	wedge3->agentOnly = true;
	tempVector.push_back(wedge3);
	
	//create slice 4
	
	slice* wedge4 = new slice;
	wedge4->minDist = 0.0f;
	wedge4->maxDist = 10.0f;
	wedge4->firstBound = 155.0f;
	wedge4->secondBound = 360.0f - 155.0f;	//ultimately the bounds are compared to [0, 360)
	wedge4->agentOnly = true;
	tempVector.push_back(wedge4);
	
	//create slice 5
	
	slice* wedge5 = new slice;
	wedge5->minDist = 0.0f;
	wedge5->maxDist = 10.0f;
	wedge5->firstBound = 360.0f - 155.0f;
	wedge5->secondBound = 360.0f - 105.0f;
	wedge5->agentOnly = true;
	tempVector.push_back(wedge5);
	
	//create slice 6
	
	slice* wedge6 = new slice;
	wedge6->minDist = 0.0f;
	wedge6->maxDist = 10.0f;
	wedge6->firstBound = 360.0f - 105.0f;
	wedge6->secondBound = 360.0f - 60.0f;
	wedge6->agentOnly = true;
	tempVector.push_back(wedge6);
	
	//create slice 7

	slice* wedge7 = new slice;
	wedge7->minDist = 0.0f;
	wedge7->maxDist = 10.0f;
	wedge7->firstBound = 360.0f - 60.0f;
	wedge7->secondBound = 360.0f - 22.5f;
	wedge7->agentOnly = true;
	tempVector.push_back(wedge7);

	//create slice 7

	slice* wedge8 = new slice;
	wedge8->minDist = 0.0f;
	wedge8->maxDist = 10.0f;
	wedge8->firstBound = 360.0f - 22.5f;
	wedge8->secondBound = 360.0f;
	wedge8->agentOnly = true;
	tempVector.push_back(wedge8);

	spaces.push_back(tempVector);

	//create a default state space for the top-level classifier

	//north areas
	density* areaD = new density();
	flow* areaF = new flow();
	//obstaclesPresent* areaO = new obstaclesPresent();
	
	areaD->agentOnly = true;
	areaD->minDist = 1.0f;
	areaD->maxDist = 20.0f;
	areaD->firstBound = 360.0f - 135.0f;
	areaD->secondBound = 45.0f;
	
	areaF->minDist = 1.0f;
	areaF->maxDist = 20.0f;
	areaF->firstBound = 360.0f - 135.0f;
	areaF->secondBound = 45.0f;

	specialSpace.push_back(areaF);
	specialSpace.push_back(areaD);
	//specialSpace.push_back(areaO);
	
	//west areas
	areaD = new density();
	areaF = new flow();

	areaD->agentOnly = true;
	areaD->minDist = 1.0f;
	areaD->maxDist = 20.0f;
	areaD->firstBound = 45.0f;
	areaD->secondBound = 135.0f;
	
	areaF->minDist = 1.0f;
	areaF->maxDist = 20.0f;
	areaF->firstBound = 45.0f;
	areaF->secondBound = 135.0f;

	specialSpace.push_back(areaF);
	specialSpace.push_back(areaD);
	
	//south areas
	areaD = new density();
	areaF = new flow();

	areaD->agentOnly = true;
	areaD->minDist = 1.0f;
	areaD->maxDist = 20.0f;
	areaD->firstBound = 135.0f;
	areaD->secondBound = 360.0f - 135.0f;
	
	areaF->minDist = 1.0f;
	areaF->maxDist = 20.0f;
	areaF->firstBound = 135.0f;
	areaF->secondBound = 360.0f - 135.0f;

	specialSpace.push_back(areaF);
	specialSpace.push_back(areaD);

	//east areas
	areaD = new density();
	areaF = new flow();

	areaD->agentOnly = true;
	areaD->minDist = 1.0f;
	areaD->maxDist = 20.0f;
	areaD->firstBound = 360.0f - 135.0f;
	areaD->secondBound = 360.0f - 45.0f;
	
	areaF->minDist = 1.0f;
	areaF->maxDist = 20.0f;
	areaF->firstBound = 360.0f - 135.0f;
	areaF->secondBound = 360.0f - 45.0f;

	specialSpace.push_back(areaF);
	specialSpace.push_back(areaD);
}

StateConfig::StateConfig(string filename) {
	//currently unimplemented
}

StateConfig::~StateConfig() {
	//deallocate the spaces
	for(vector<vector<state*>>::iterator outerIter = spaces.begin(); outerIter != spaces.end(); ++outerIter) {
		for(vector<state*>::iterator innerIter = outerIter->begin(); innerIter != outerIter->end(); ++innerIter) {
			delete *innerIter;
		}

		outerIter->clear();
	}

	spaces.clear();

	for(vector<state*>::iterator iter = specialSpace.begin(); iter != specialSpace.end(); ++iter) {
		delete *iter;
	}

	specialSpace.clear();
}

vector<StateConfig::state*>::const_iterator StateConfig::iteratorBegin(int index) {
	if(index >= 0) {
		if(spaces.size() == 1) {	//the specialized state space is universal
			return spaces[0].cbegin();
		}
		else {
			return spaces[index].cbegin();
		}
	}
	else {
		return specialSpace.cbegin();
	}
}

vector<StateConfig::state*>::const_iterator StateConfig::iteratorEnd(int index) {
	if(index >= 0) {
		if(spaces.size() == 1) {
			return spaces[0].cend();
		}
		else {
			return spaces[index].cend();
		}
	}
	else {
		return specialSpace.cend();
	}
}

string StateConfig::toString(int index) {
	stringstream result;
	int counters[4];
	for(int i = 0; i < 4; i++) {
		counters[i] = 0;
	}

	vector<state*>::const_iterator start;
	vector<state*>::const_iterator finish;
	if(index >= 0) {
		if(spaces.size() == 1) {
			start = spaces[0].cbegin();
			finish = spaces[0].cend();
		}
		else {
			start = spaces[index].cbegin();
			finish = spaces[index].cend();
		}
	}
	else {	//this way we can pass in -1 as a special value to get to the context feature set
		start = specialSpace.cbegin();
		finish = specialSpace.cend();
	}

	for(vector<state*>::const_iterator citer = start; citer != finish; ++citer) {
		const state* temp = (*citer);

		switch(temp->type_id) {
		case STATE_SLICE:
			result << "Slice" << counters[0] << "_Distance:\t" << "-1,";
			for(unsigned int i = static_cast<int>(ceil(static_cast<const slice*>(temp)->minDist)); i <= static_cast<int>(ceil(static_cast<const slice*>(temp)->maxDist)); i++) {
				if(i == static_cast<int>(ceil(static_cast<const slice*>(temp)->maxDist))) {
					result << i << ".\n";
				}
				else {
					result << i << "," << i + 0.5f << ",";
				}
			}
			result << "Slice" << counters[0] << "_Speed:\t" << "continuous.\n";
			result << "Slice" << counters[0] << "_Theta:\t" << "0,1,2,3,4,5,6,7,8,9,10,11,-1,-2,-3,-4,-5,-6,-7,-8,-9,-10,-11,12.\n";
			counters[0]++;
			break;
		
		case STATE_DENSITY:
			result << "Density" << counters[1] << ":\t" << "continuous.\n";
			counters[1]++;
			break;
		
		case STATE_OBSTACLES:
			result << "Obstacles" << counters[1] << ":\t" << "n,y.\n";
			counters[2]++;
			break;
		
		case STATE_FLOW:
			result << "Flow" << counters[3] << "_Speed:\t" << "continuous.\n";
			result << "Flow" << counters[3] << "_Theta:\t" << "0,1,2,3,4,5,6,7,8,9,10,11,-1,-2,-3,-4,-5,-6,-7,-8,-9,-10,-11,12.\n";
			counters[3]++;
			break;
		}
	}

	return result.str();
}

/*----------------------- From here below is future work when the code is cleaned up ------------------------*/

//Class that contains callbacks to trigger when the XML is parsed
void StateParser::startElement(XMLTag* tag, const ticpp::Element* subroot) {
	
	if(tag->getTagName() == "BoundingSlice") {

	}

	else if(tag->getTagName() == "RegionDensity") {

	}
}

/*
void outputFormattedXML(std::ostream &out, const std::string & indentation) {

}
*/

StateXMLReader::StateXMLReader() {
	setupXMLStructure();
}

StateXMLReader::~StateXMLReader() {

}

void StateXMLReader::setupXMLStructure() {
	//TODO: need to link up callbacks

	//Do this breadth-first for clarity
	XMLTag* root = docReader.createRootTag("SteerSimStateSpace", "This file contains the geometric descriptions of a state space for machine learning from simulations.");

	//Top-level types of configurable state space data
	XMLTag* sliceTag = root->createChildTag("BoundingSlice", "Defines a pie-like wedge in space around the subject.", XML_DATA_TYPE_CONTAINER, NULL, &callbacks);
	XMLTag* densityTag = root->createChildTag("RegionDensity", "Defines the bounding box for this region, z-values will be ignored.", XML_DATA_TYPE_BOUNDING_BOX);

	//Data making up BoundingSlice entry, these might need to be deleted and handled in a callback form with lower-level function calls
	sliceTag->createChildTag("IsAgent", "Set true if this area cares about agents.", XML_DATA_TYPE_BOOLEAN, NULL);
	sliceTag->createChildTag("IsObstacle", "Set true if this area cares about environmental obstacles.", XML_DATA_TYPE_BOOLEAN, NULL);
	sliceTag->createChildTag("LeftBound", "Angle in degrees the left bound is rotated from the subject's right side.  90 would be a line from the front of the subject.", XML_DATA_TYPE_SIGNED_INT);
	sliceTag->createChildTag("RightBound", "Angle in degrees the right bound is rotated from the subject's right side.", XML_DATA_TYPE_SIGNED_INT);
	sliceTag->createChildTag("MaxDist", "Sets the far horizon with respect to the subject for entities to be counted in this region.", XML_DATA_TYPE_FLOAT);
	sliceTag->createChildTag("MinDist", "Sets the near horizon with respect to the subject for entities to be counted in this region.", XML_DATA_TYPE_FLOAT);
	
}

void StateXMLReader::generateSample(const std::string& filename) {
	docReader.writeXMLFile(filename);	//this currently doesn't work, either unimplemented functionality in the parser or bad XML format definition
}

void StateXMLReader::loadConfig(const std::string& filename) {

}

StateConfig* StateXMLReader::genDefault() {
	return new StateConfig;
}