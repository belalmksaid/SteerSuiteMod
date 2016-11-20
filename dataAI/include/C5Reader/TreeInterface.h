//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#ifndef TREE_INTERFACE
#define TREE_INTERFACE

#include "c5gpl.h"	//connects to a LIB or DLL, depending on project settings
#include <string>
#include <vector>
#include <utility>

class TreeInterface {

public:
	//pulled directly from GenEngine in SteerGen with equality testing dropped
	struct stepInfo {
		float phi;
		float desiredSpeed;
		float duration;
		int footstepEnum;
	};

	explicit TreeInterface(std::string);	//should be able to pull in everything with this one filename root
	TreeInterface(std::string, int);
	~TreeInterface(void);
	bool loadActions(void);
	bool loadModels(void);
	
	std::pair<unsigned int, float> getContext(std::string) const;
	std::pair<stepInfo, float> getAction(unsigned int, std::string, bool) const;
	bool forcedContext;
	int contextID;

private:
	//the data that makes up the classifiers
	std::vector<std::vector<stepInfo>> actionMaps[2];
	std::vector<C5Engine*> specializedClassifiers[2];
	C5Engine* contextClassifier;
	
	//helpful data
	std::string filenameRoot;
	int maxContext;
};

#endif