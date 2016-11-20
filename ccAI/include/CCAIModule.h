//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#ifndef __CC_AI_MODULE__
#define __CC_AI_MODULE__

/// @file CCAIModule.h
/// @brief Declares the CCAIModule plugin.


#include <map>
#include <set>
#include <utility>
#include <limits> 
#include "SteerLib.h"
#include "Logger.h"

class CCAgent;
typedef std::pair<int, int> pair_int;
using std::set;


// Max groups to create before issuing a warning
#define MAX_NUMBER_OF_GROUPS 5

// Density exponent/decay.
// Threshold is 1/2^decay.
#define DENSITY_DECAY 0.5f
#define DENSITY_THRESHOLD 0.7071067f
#define MIN_DENSITY 0.71f
#define MAX_DENSITY 0.74f


#define MIN_SLOPE 0.0f
#define MAX_SLOPE 10.0f

#define MIN_SPEED 0.001f
#define MAX_SPEED 3.0f

#define PATH_WEIGHT 0.03333f
#define DISCOMFORT_WEIGHT 2.0f
#define LARGE_POTENTIAL 9999.0f
#define LARGE_DISCOMFORT 9999.0f
#define TIME_WEIGHT 1.0f

// Padding for ccAI to reach goal, might be sensitive to radius.
#define GOAL_REACHED_DISTANCE 1.5f

// globally accessible to the CCAI plugin
extern SteerLib::EngineInterface * gEngine;
extern SteerLib::SpatialDataBaseInterface * gSpatialDatabase;


struct AniostropicFields {
	// Speed field out.
	float speed_out;
	// Cost field out.
	float cost_out;
	// Height gradient.
	Util::Vector height_gradient;
	// Potential gradient.
	Util::Vector potential_gradient;
	// Velocity.
	Util::Vector velocity;

	void reset() {
		speed_out = 0.0f;
		cost_out = std::numeric_limits<float>::infinity();
		height_gradient.zero();
		potential_gradient.zero();
		velocity.zero();
	}
	AniostropicFields() {
		reset();
	}
};

struct CellFields {
  // Cell index.
	pair_int indices;
	float discomfort;
	float potential;
	float density;
	float height;
	// X & Z, no Y.
	Util::Vector avg_velocity;

	// Aniostropic ============================
	AniostropicFields Nface;
	AniostropicFields Eface;
	AniostropicFields Wface;
	AniostropicFields Sface;

	void reset() {
		discomfort = 0.0f;
		potential = std::numeric_limits<float>::infinity();
		density = 0.0f;
		height = 0.0f;
		avg_velocity.zero();
		Nface.reset();
		Eface.reset();
		Wface.reset();
		Sface.reset();
	}
};

struct Groups {
	std::string name;
	Util::AxisAlignedBox target_region;
	Util::Point lower_right;
	Util::Point upper_left;

	std::set< size_t > agents;
};

// forward declaration
class PerformanceProfiler;

namespace CCGlobals {

        struct PhaseProfilers {
                Util::PerformanceProfiler aiProfiler;
                Util::PerformanceProfiler drawProfiler;
                Util::PerformanceProfiler createGroupingProfiler;
                Util::PerformanceProfiler computeDensityFieldProfiler;
                Util::PerformanceProfiler potentialFieldConstructionProfiler;
                Util::PerformanceProfiler steeringPhaseProfiler;
        };

        extern PhaseProfilers * gPhaseProfilers;

        static Vector North(0.0f, 0.0f, 1.0f);
		static Vector South(0.0f, 0.0f, -1.0f);
		static Vector West(1.0f, 0.0f, 0.0f);
		static Vector East(-1.0f, 0.0f, 0.0f);

}

/**
 * @brief An example plugin for the SimulationEngine that provides CC AI agents.
 *
 * This class is an example of a plug-in module (as opposed to a built-in module).
 * It compiles as part of a dynamic library which is loaded by a SimulationEngine at run-time.
 *
 * The ccAI plugin consists of three parts:
 *  - This class inherits from SteerLib::ModuleInterface, and implements only the desired functionality.  In this case
 *    the desired functionality is to be able to create/destroy CCAgent agents.
 *  - The two global functions createModule() and destroyModule() are implemented so that the engine can load the
 *    dynamic library and get an instance of our CCAIModule.
 *  - The CCAgent class inherits from SteerLib::AgentInterface, which is the agent steering AI used by the engine.
 *
 */
class CCAIModule : public SteerLib::ModuleInterface
{
public:
	CCAIModule() : _agent_counter(0), _comma_pressed(false), _period_pressed(false),
								 _forward_slash_pressed(false), _n_pressed(false), _m_pressed(false),
	               _b_pressed(false), _potential_draw(0.0f) { }
	
	//std::string getDependencies() { return "testCasePlayer"; }
	// MUBBASIR CHANGE removing dependencies to be used by scenario module
	std::string getDependencies() { return ""; }

	std::string getConflicts() { return ""; }
	std::string getData() { return ""; }
	LogData * getLogData() { return new LogData(); }
	void init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo );
	void finish();
	SteerLib::AgentInterface * createAgent();
	void destroyAgent( SteerLib::AgentInterface * agent );

	void initializeSimulation();
  void cleanupSimulation();
	/// This update function is called once before the simulation begins.
  void preprocessSimulation();
	/// This update function is called once per frame before all agents are updated.
	void preprocessFrame(float timeStamp, float dt, unsigned int frameNumber);
	void postprocessFrame(float timeStamp, float dt, unsigned int frameNumber);
	void processKeyboardInput(int key, int action );
	void draw();

private:
	// Check Bounds
	bool checkBounds(unsigned int x, unsigned int z) const;
	
	// Create groups
	void createGrouping();

	// Density
	void computeDensityField();

	// Unit Cost
	void computeUnitCost(const float dt);

	// Potential Field
	void potentialFieldConstruction(const Groups& group);
	float computePotential(const pair_int& cur, const set< pair_int >& known);
	void updatePotentialGradientVelocity();

	// Update agent locations based on groups.
	void updateLocations(const Groups& group);

	// 2D grid access.
	CellFields** _grid;
	unsigned int _rows;
	unsigned int _columns;

	std::map< std::string, Groups > _groups;

	int _agent_counter;

	bool _comma_pressed;
	bool _period_pressed;
	bool _forward_slash_pressed;
	bool _n_pressed;
	bool _m_pressed;
	bool _b_pressed;

	float _potential_scale;
	float _potential_draw;

	bool displayLogData;
	bool logStats;
	bool logToFile;
	std::string logFilename;
	Logger * _ccAILogger;
};

#endif
