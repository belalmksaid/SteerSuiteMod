//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

//
// Copyright notice is on the TODO list, as well as the @ tags for documentation
//
//

/// @file steergen/src/StateConfig.h
/// @brief Header for the state space XML reader and configuration storage.

#ifndef __STATESPACEXML__
#define __STATESPACEXML__

#include "SteerLib.h"

//TODO: order these class declarations appropriately
//TODO: add these classes to the namespaces they belong in

//Class that stores the configured state space information
class StateConfig {

public:

	//used to predictably be able to dynamic_cast or static_cast
	//if a new feature-type is added, a child struct of state is needed, as well as adding it to toString
	enum StateEnum {
		STATE_SLICE,
		STATE_DENSITY,
		STATE_FLOW,
		STATE_OBSTACLES,
	};

	struct state {
		StateEnum type_id;
		virtual ~state(void) { }		//a virtual, empty destructor makes this a "polymorphic type" so dynamic_cast can work
	};

	//a slice of space around the agent as seen in Torrens' work
	struct slice : public state {
		float firstBound;	//these bounds are with regards to a counter-clockwise rotation about the agent and are in degrees
		float secondBound;
		float minDist;		//minimum distance allowed for consideration, supports multi-section slices
		float maxDist;		//maximum distance that agents should be considered close enough to count
		bool agentOnly;		//when true, only care about agents in this space rather than both agents and environmental obstacles

		//struct constructor to set the type_id
		slice(void) { type_id = STATE_SLICE; }
		~slice(void) { }
	};

	struct density : public state {
		float firstBound;
		float secondBound;
		float minDist;
		float maxDist;
		bool agentOnly;
		
		//struct constructor to set the type_id
		density(void) { type_id = STATE_DENSITY; }
		~density(void) { }
	};

	struct obstaclesPresent : public state {
		obstaclesPresent(void) { type_id = STATE_OBSTACLES; }
		~obstaclesPresent(void) { }
	};

	struct flow : public state {
		float firstBound;
		float secondBound;
		float minDist;
		float maxDist;

		flow(void) { type_id = STATE_FLOW; }
		~flow(void) { }
	};

	StateConfig(void);
	explicit StateConfig(std::string);
	~StateConfig(void);
	std::vector<StateConfig::state*>::const_iterator iteratorBegin(int);
	std::vector<StateConfig::state*>::const_iterator iteratorEnd(int);
	std::string toString(int);

//typically my private variables go at the beginning but the struct definitions made it cleaner to do so here
private:
	std::vector<std::vector<state*>> spaces;	//I really, really hate nested vectors

	std::vector<state*> specialSpace; //need a special space for the context classifier because it exists outside the number of contexts
};

//Class that contains callbacks to trigger when the XML is parsed
class StateParser : public Util::XMLParserCallbackInterface {
public:
	
	void init(StateConfig* states) {specs = states;}
	void startElement(Util::XMLTag*, const ticpp::Element*);
	//void outputFormattedXML(std::ostream&, const std::string&);
protected:
	StateConfig* specs;
};

//Class for reading the XML file itself
class StateXMLReader {
private:
	Util::XMLParser docReader; /////////////This is the utility that supposedly writes out an XML file.
	void setupXMLStructure(void);
	StateParser callbacks;

public:
	StateXMLReader(void);
	~StateXMLReader(void);
	void generateSample(const std::string&);	//only example of state space writing, dumps out the XML tag definitions in the proper format
	void loadConfig(const std::string&);
	StateConfig* genDefault(void);	//TODO: Define the "default" hardcoded state space
};

#endif