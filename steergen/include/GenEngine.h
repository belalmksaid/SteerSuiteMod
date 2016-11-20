//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#ifndef GENENGINE_H
#define GENENGINE_H

#define _USE_MATH_DEFINES

#include "SteerLib.h"		//implicitly includes <math.h>
#include "footrec/FootRecIO.h"	//this references the header in the footstepAI project: the .cpp added to this project also references footstepAI directly
#include "StateConfig.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <limits>
#include <utility>
#include <stdio.h>
#include <stdlib.h>

#define EPSILON 0.0001f

class GeneratorEngine {
private:
	//private structs
	struct agentInfo {
		float COMx;
		float COMz;
		float velx;	//may end up using their footstep info instead of COM info
		float velz;
		unsigned int id;	//might allow for fast culling of agents but performance gains may be minimal; will integrate later
	};

	//slightly redundant as there is an identical data structure in FootRecIO, but we copy information into this and transform it
	struct obstacleInfo {
		float xmin;
		float xmax;
		float zmin;
		float zmax;
		float rayCorrectionAngle;
	};

	struct subjectInfo {
		Footstep step;
		float theta;
		float subjectPrevX;
		float subjectPrevZ;
		Util::Point subjectGoal;
	};

	struct stepInfo {
		float phi;
		float desiredSpeed;
		float duration;
		int footstepEnum;

		friend static bool operator==(const GeneratorEngine::stepInfo& lhs, const GeneratorEngine::stepInfo& rhs) {
			if(GeneratorEngine::epsTest(lhs.phi, rhs.phi) && GeneratorEngine::epsTest(lhs.desiredSpeed, rhs.desiredSpeed) && GeneratorEngine::epsTest(lhs.duration, rhs.duration) && (lhs.footstepEnum == rhs.footstepEnum)) {
				return true;
			}
			else {
				return false;
			}
		}
	};

	//private data
	FootRecReader* reader;
	StateConfig* stateSpace;
	unsigned int currentStep;
	std::string outFileRoot;
	subjectInfo subject;	//used for passing around subject information through the helper functions
	subjectInfo lastFrame;
	unsigned int contextNumber;
	std::pair<bool, std::vector<std::string>> metaLog; //first element is a "dirty bit"
	bool validation;
	std::vector<int> to_remove[2];
	
	//"table" to store the action space as it is revealed through the rec file
	std::vector<stepInfo> actionMap[2];
	std::vector<std::string> samples[2];
	std::vector<std::string> contextSamples;
	std::vector<unsigned int> actionUsage[2];

	//private functions
	void handleSlice(const StateConfig::slice, std::vector<agentInfo>&, std::vector<obstacleInfo>&, std::stringstream&);
	void handleDensity(const StateConfig::density, std::vector<agentInfo>&, std::vector<obstacleInfo>&, std::stringstream&);
	void handleFlow(const StateConfig::flow, std::vector<agentInfo>&, std::stringstream&);
	void handleObstacles(const StateConfig::obstaclesPresent, std::vector<obstacleInfo>&, std::stringstream&);
	void printRecFileContents(void) const;	//for debugging purposes
	void existingSampleLoader(const std::string&);
	
	//helper functions to keep code more readable
	void extractContextSamples(unsigned int, std::vector<agentInfo>&, std::vector<obstacleInfo>&);
	void extractSpecializedSamples(unsigned int, std::vector<agentInfo>&, std::vector<obstacleInfo>&);

public:
	GeneratorEngine(StateConfig*, unsigned int, const std::string&, bool);
	virtual ~GeneratorEngine(void);
	void loadRecFile(const std::string&);
	void extractSamples(void);	//should be able to tack on the context here
	void printStats(void) const;
	void saveSamples(void);
	void shrinkage(void);

	//helper function to make the code cleaner to read
	inline static bool epsTest(float first, float second) {
		if(first <= (second + EPSILON) && first >= (second - EPSILON)) {
			return true;
		}
		else if(first >= 3.0e30 && second >= 3.0e30) {
			return true;
		}
		else {
			return false;
		}
	}
};

#endif