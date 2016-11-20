//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


/// @file CCAIModule.cpp
/// @brief Implements the CCAIModule plugin.


#include "CCAIModule.h"


#include <cmath>

#include <map>
#include <queue>
#include <vector>


#include "CCAgent.h"
#include "SimulationPlugin.h"
#include "SteerLib.h"

#include "LogObject.h"
#include "LogManager.h"

#undef min // not sure where this is defined, but it's causing problems in VS2012

#include <algorithm>

using std::set;
using std::cout;
using std::endl;

// Used to define grid location indices.
typedef std::pair<int, int> pair_int;

using namespace Util;

// globally accessible to the CCAI plugin
SteerLib::EngineInterface * gEngine;
SteerLib::SpatialDataBaseInterface * gSpatialDatabase;

// TODO: Make static?
namespace CCGlobals
{
        PhaseProfilers * gPhaseProfilers;
}

using namespace CCGlobals;

PLUGIN_API SteerLib::ModuleInterface * createModule()
{
	return new CCAIModule;
}


PLUGIN_API void destroyModule( SteerLib::ModuleInterface*  module )
{
	delete module;
}


void CCAIModule::init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo )
{
	gEngine = engineInfo;
	gSpatialDatabase = engineInfo->getSpatialDatabase();

	logStats = true;
	logToFile = true;
	logFilename = "ccAI.log";

	SteerLib::OptionDictionary::const_iterator optionIter;
		for (optionIter = options.begin(); optionIter != options.end(); ++optionIter)
		{
			std::stringstream value((*optionIter).second);

			if ((*optionIter).first == "logStats")
			{
				logStats = Util::getBoolFromString(value.str());
			}
			else if ((*optionIter).first == "ailogFileName")
			{
				logFilename = value.str();
			}
			else if ((*optionIter).first == "displayStats")
			{
				displayLogData = Util::getBoolFromString(value.str());
			}
			else
			{
				// throw Util::GenericException("unrecognized option \"" + Util::toString((*optionIter).first) + "\" given to PPR AI module.");
			}
		}

        std::cout << "Initializing CCAIModule.\n";
        _rows = gSpatialDatabase->getNumCellsX();
        _columns = gSpatialDatabase->getNumCellsZ();
        _grid = new CellFields*[_rows];
        for (unsigned int i = 0; i < _rows; ++i)
        {
            _grid[i] = new CellFields[_columns];
            for (unsigned int j = 0; j < _columns; ++j)
            {
                _grid[i][j].indices = pair_int(i,j);
            }
        }

        if ( logToFile )
        {
        	_ccAILogger = LogManager::getInstance()->createLogger(logFilename,LoggerType::BASIC_WRITE);


        	_ccAILogger->addDataField("compute_density_field_number_of_times_executed",DataType::LongLong );
        	_ccAILogger->addDataField("compute_density_field_total_ticks_accumulated",DataType::LongLong );
        	_ccAILogger->addDataField("compute_density_field_shortest_execution",DataType::LongLong );
        	_ccAILogger->addDataField("compute_density_field_longest_execution",DataType::LongLong );
        	_ccAILogger->addDataField("compute_density_field_fastest_execution", DataType::Float);
        	_ccAILogger->addDataField("compute_density_field_slowest_execution", DataType::Float);
        	_ccAILogger->addDataField("compute_density_field_average_time_per_call", DataType::Float);
        	_ccAILogger->addDataField("compute_density_field_total_time_of_all_calls", DataType::Float);
        	_ccAILogger->addDataField("compute_density_field_tick_frequency", DataType::Float);

        	_ccAILogger->addDataField("create_grouping_number_of_times_executed",DataType::LongLong );
        	_ccAILogger->addDataField("create_grouping_total_ticks_accumulated",DataType::LongLong );
        	_ccAILogger->addDataField("create_grouping_shortest_execution",DataType::LongLong );
        	_ccAILogger->addDataField("create_grouping_longest_execution",DataType::LongLong );
        	_ccAILogger->addDataField("create_grouping_fastest_execution", DataType::Float);
        	_ccAILogger->addDataField("create_grouping_slowest_execution", DataType::Float);
        	_ccAILogger->addDataField("create_grouping_average_time_per_call", DataType::Float);
        	_ccAILogger->addDataField("create_grouping_total_time_of_all_calls", DataType::Float);
        	_ccAILogger->addDataField("create_grouping_tick_frequency", DataType::Float);

        	_ccAILogger->addDataField("draw_number_of_times_executed",DataType::LongLong );
        	_ccAILogger->addDataField("draw_total_ticks_accumulated",DataType::LongLong );
        	_ccAILogger->addDataField("draw_shortest_execution",DataType::LongLong );
        	_ccAILogger->addDataField("draw_longest_execution",DataType::LongLong );
        	_ccAILogger->addDataField("draw_fastest_execution", DataType::Float);
        	_ccAILogger->addDataField("draw_slowest_execution", DataType::Float);
        	_ccAILogger->addDataField("draw_average_time_per_call", DataType::Float);
        	_ccAILogger->addDataField("draw_total_time_of_all_calls", DataType::Float);
        	_ccAILogger->addDataField("draw_tick_frequency", DataType::Float);

        	_ccAILogger->addDataField("potential_field_construction_number_of_times_executed",DataType::LongLong );
        	_ccAILogger->addDataField("potential_field_construction_total_ticks_accumulated",DataType::LongLong );
        	_ccAILogger->addDataField("potential_field_construction_shortest_execution",DataType::LongLong );
        	_ccAILogger->addDataField("potential_field_construction_longest_execution",DataType::LongLong );
        	_ccAILogger->addDataField("potential_field_construction_fastest_execution", DataType::Float);
        	_ccAILogger->addDataField("potential_field_construction_slowest_execution", DataType::Float);
        	_ccAILogger->addDataField("potential_field_construction_average_time_per_call", DataType::Float);
        	_ccAILogger->addDataField("potential_field_construction_total_time_of_all_calls", DataType::Float);
        	_ccAILogger->addDataField("potential_field_construction_tick_frequency", DataType::Float);

        	_ccAILogger->addDataField("number_of_times_executed",DataType::LongLong );
        	_ccAILogger->addDataField("total_ticks_accumulated",DataType::LongLong );
        	_ccAILogger->addDataField("shortest_execution",DataType::LongLong );
        	_ccAILogger->addDataField("longest_execution",DataType::LongLong );
        	_ccAILogger->addDataField("fastest_execution", DataType::Float);
        	_ccAILogger->addDataField("slowest_execution", DataType::Float);
        	_ccAILogger->addDataField("average_time_per_call", DataType::Float);
        	_ccAILogger->addDataField("total_time_of_all_calls", DataType::Float);
        	_ccAILogger->addDataField("tick_frequency", DataType::Float);



        	// LETS TRY TO WRITE THE LABELS OF EACH FIELD
        	std::stringstream labelStream;
        	unsigned int i;
        	for (i=0; i < _ccAILogger->getNumberOfFields() - 1; i++)
        		labelStream << _ccAILogger->getFieldName(i) << " ";
        	labelStream << _ccAILogger->getFieldName(i);

        	_ccAILogger->writeData(labelStream.str());
        }
}

/*
 * This is where all deallocation of memory should be done?
 */
void CCAIModule::finish()
{
	for (unsigned int i = 0; i < _rows; ++i) {
	  delete [] _grid[i];
	}
	delete [] _grid;
}


SteerLib::AgentInterface * CCAIModule::createAgent()
{
  CCAgent* agent = new CCAgent;
  agent->_name = gEngine->getAgents().size();
  agent->_id = gEngine->getAgents().size();
  return agent;
}


void CCAIModule::destroyAgent( SteerLib::AgentInterface * agent )
{
	delete agent;
}


void CCAIModule::initializeSimulation() {
  /*
	std::cout << "Initializing CCAIModule.\n";
	_rows = gSpatialDatabase->getNumCellsX();
	_columns = gSpatialDatabase->getNumCellsZ();
 	_grid = new CellFields*[_rows];
	for (unsigned int i = 0; i < _rows; ++i)
	{
	    _grid[i] = new CellFields[_columns];
  	    for (unsigned int j = 0; j < _columns; ++j)
  	    {
  		_grid[i][j].indices = pair_int(i,j);
  	    }
	}
*/
        //
        // initialize the performance profilers
        //
        gPhaseProfilers = new PhaseProfilers;
        gPhaseProfilers->aiProfiler.reset();
        gPhaseProfilers->drawProfiler.reset();
        gPhaseProfilers->createGroupingProfiler.reset();
        gPhaseProfilers->computeDensityFieldProfiler.reset();
        gPhaseProfilers->steeringPhaseProfiler.reset();
        gPhaseProfilers->potentialFieldConstructionProfiler.reset();
}


void CCAIModule::cleanupSimulation()
{

	// TODO: This is where logging should be done.
	if (displayLogData)
	{
        gPhaseProfilers->computeDensityFieldProfiler.displayStatistics(std::cout);
        gPhaseProfilers->createGroupingProfiler.displayStatistics(std::cout);
        gPhaseProfilers->drawProfiler.displayStatistics(std::cout);
        std::cout << "potentialFieldConstructionProfiler: \n";
        gPhaseProfilers->potentialFieldConstructionProfiler.displayStatistics(std::cout);
        std::cout << "Total AI:\n";
        gPhaseProfilers->aiProfiler.displayStatistics(std::cout);
        std::cout << "Cleaned up CCAIModule.\n";
	}
	if ( logToFile )
	{
		LogObject ccLogObject;


		ccLogObject.addLogData(gPhaseProfilers->computeDensityFieldProfiler.getNumTimesExecuted());
		ccLogObject.addLogData(gPhaseProfilers->computeDensityFieldProfiler.getTotalTicksAccumulated());
		ccLogObject.addLogData(gPhaseProfilers->computeDensityFieldProfiler.getMinTicks());
		ccLogObject.addLogData(gPhaseProfilers->computeDensityFieldProfiler.getMaxTicks());
		ccLogObject.addLogData(gPhaseProfilers->computeDensityFieldProfiler.getMinExecutionTimeMills());
		ccLogObject.addLogData(gPhaseProfilers->computeDensityFieldProfiler.getMaxExecutionTimeMills());
		ccLogObject.addLogData(gPhaseProfilers->computeDensityFieldProfiler.getAverageExecutionTimeMills());
		ccLogObject.addLogData(gPhaseProfilers->computeDensityFieldProfiler.getTotalTime());
		ccLogObject.addLogData(gPhaseProfilers->computeDensityFieldProfiler.getTickFrequency());

		ccLogObject.addLogData(gPhaseProfilers->createGroupingProfiler.getNumTimesExecuted());
		ccLogObject.addLogData(gPhaseProfilers->createGroupingProfiler.getTotalTicksAccumulated());
		ccLogObject.addLogData(gPhaseProfilers->createGroupingProfiler.getMinTicks());
		ccLogObject.addLogData(gPhaseProfilers->createGroupingProfiler.getMaxTicks());
		ccLogObject.addLogData(gPhaseProfilers->createGroupingProfiler.getMinExecutionTimeMills());
		ccLogObject.addLogData(gPhaseProfilers->createGroupingProfiler.getMaxExecutionTimeMills());
		ccLogObject.addLogData(gPhaseProfilers->createGroupingProfiler.getAverageExecutionTimeMills());
		ccLogObject.addLogData(gPhaseProfilers->createGroupingProfiler.getTotalTime());
		ccLogObject.addLogData(gPhaseProfilers->createGroupingProfiler.getTickFrequency());

		ccLogObject.addLogData(gPhaseProfilers->drawProfiler.getNumTimesExecuted());
		ccLogObject.addLogData(gPhaseProfilers->drawProfiler.getTotalTicksAccumulated());
		ccLogObject.addLogData(gPhaseProfilers->drawProfiler.getMinTicks());
		ccLogObject.addLogData(gPhaseProfilers->drawProfiler.getMaxTicks());
		ccLogObject.addLogData(gPhaseProfilers->drawProfiler.getMinExecutionTimeMills());
		ccLogObject.addLogData(gPhaseProfilers->drawProfiler.getMaxExecutionTimeMills());
		ccLogObject.addLogData(gPhaseProfilers->drawProfiler.getAverageExecutionTimeMills());
		ccLogObject.addLogData(gPhaseProfilers->drawProfiler.getTotalTime());
		ccLogObject.addLogData(gPhaseProfilers->drawProfiler.getTickFrequency());

		ccLogObject.addLogData(gPhaseProfilers->potentialFieldConstructionProfiler.getNumTimesExecuted());
		ccLogObject.addLogData(gPhaseProfilers->potentialFieldConstructionProfiler.getTotalTicksAccumulated());
		ccLogObject.addLogData(gPhaseProfilers->potentialFieldConstructionProfiler.getMinTicks());
		ccLogObject.addLogData(gPhaseProfilers->potentialFieldConstructionProfiler.getMaxTicks());
		ccLogObject.addLogData(gPhaseProfilers->potentialFieldConstructionProfiler.getMinExecutionTimeMills());
		ccLogObject.addLogData(gPhaseProfilers->potentialFieldConstructionProfiler.getMaxExecutionTimeMills());
		ccLogObject.addLogData(gPhaseProfilers->potentialFieldConstructionProfiler.getAverageExecutionTimeMills());
		ccLogObject.addLogData(gPhaseProfilers->potentialFieldConstructionProfiler.getTotalTime());
		ccLogObject.addLogData(gPhaseProfilers->potentialFieldConstructionProfiler.getTickFrequency());

		ccLogObject.addLogData(gPhaseProfilers->aiProfiler.getNumTimesExecuted());
		ccLogObject.addLogData(gPhaseProfilers->aiProfiler.getTotalTicksAccumulated());
		ccLogObject.addLogData(gPhaseProfilers->aiProfiler.getMinTicks());
		ccLogObject.addLogData(gPhaseProfilers->aiProfiler.getMaxTicks());
		ccLogObject.addLogData(gPhaseProfilers->aiProfiler.getMinExecutionTimeMills());
		ccLogObject.addLogData(gPhaseProfilers->aiProfiler.getMaxExecutionTimeMills());
		ccLogObject.addLogData(gPhaseProfilers->aiProfiler.getAverageExecutionTimeMills());
		ccLogObject.addLogData(gPhaseProfilers->aiProfiler.getTotalTime());
		ccLogObject.addLogData(gPhaseProfilers->aiProfiler.getTickFrequency());


		_ccAILogger->writeLogObject(ccLogObject);
	}

    gPhaseProfilers->aiProfiler.reset();
    gPhaseProfilers->drawProfiler.reset();
    gPhaseProfilers->createGroupingProfiler.reset();
    gPhaseProfilers->computeDensityFieldProfiler.reset();
    gPhaseProfilers->steeringPhaseProfiler.reset();
    gPhaseProfilers->potentialFieldConstructionProfiler.reset();
}


/// This update function is called once before the simulation begins.
void CCAIModule::preprocessSimulation() {
	// Give agents names if they don't have them.
  std::vector<SteerLib::AgentInterface*> agents = gEngine->getAgents();
  std::vector<SteerLib::AgentInterface*>::iterator agent_iter;
  for (agent_iter = agents.begin(); agent_iter != agents.end(); ++agent_iter )
  {

		SteerLib::AgentInterface* agent = *agent_iter;
		/*
		CCAgent* cc_agent = dynamic_cast<CCAgent*>(agent);

		if (cc_agent->getName() == "") {
			std::stringstream make_name;
			make_name << "Agent_" << _agent_counter++;
		  cc_agent->setName(make_name.str());
		}
		*/
	}
}


float g_dt;
/// This update function is called once per frame before all agents are updated.
void CCAIModule::preprocessFrame(float timeStamp, float dt, unsigned int frameNumber) {
	// For each time step of the Continuum Crowds model:

	// Step 1
	// Clear out all previous grid continuum calculations.
	// Then add in any non-agent obstacles as locations of large discomfort.
	for (unsigned int i = 0; i < _rows; ++i) {
		for (unsigned int j = 0; j < _columns; ++j) {
			_grid[i][j].reset();
			_potential_scale = 1.0f;
			std::set<SteerLib::SpatialDatabaseItemPtr> objects;
			std::set<SteerLib::SpatialDatabaseItemPtr>::const_iterator iter;
			gSpatialDatabase->getItemsInRange(objects, i, i, j, j, NULL);

			for (iter = objects.begin(); iter != objects.end(); ++iter)
			{
				if ((*iter)->isAgent()) continue;
				_grid[i][j].discomfort = std::numeric_limits<float>::infinity();
			}
		}
	}

	// Step 2
	// Keep track of the current set of groups that share the same continuum solution.
	// Single location targets are converted to a single-point-group and run the same
	// way.
  createGrouping();

	// Step 3
	// Convert the crowd to a density field.
  computeDensityField();
	// Step 4
	// Compute speed field and unit cost.
  computeUnitCost(dt);

  for (std::map< std::string, Groups >::const_iterator iter = _groups.begin();
  		iter != _groups.end();
  		++iter) {
	  // Step 5
	  // Compute the potential field and update locations for each group.
	  potentialFieldConstruction(iter->second);
	  // Step 5
    // Lastly update each agent's location.
    updateLocations(iter->second);
	}
}


void CCAIModule::postprocessFrame(float timeStamp, float dt, unsigned int frameNumber) {
}


// Check that the grid location is within the grid carrying all values.
bool CCAIModule::checkBounds(unsigned int x, unsigned int z) const {
	if ( 0 <= x && x < _rows && 0 <= z && z < _columns) {
		return true;
	}
	return false;
}


//==============================================================================
// Grouping
//==============================================================================


// Groups are used to run the potential field and they are either given as input or converted
// from the standard target model.
//
// NOTE: due to calculation of entire potential fields too many groups will hamper performance.
void CCAIModule::createGrouping()
{
  gPhaseProfilers->createGroupingProfiler.start();
	// First clear last group set.
	_groups.clear();
	// Iterate through Agents creating group targets.
  std::vector<SteerLib::AgentInterface*> agents = gEngine->getAgents();
  std::vector<SteerLib::AgentInterface*>::iterator agent_iter;
  for (agent_iter = agents.begin(); agent_iter != agents.end(); ++agent_iter ) {
		// Skip diabled agent.
		if (!(*agent_iter)->enabled()) continue;

		SteerLib::AgentInterface* agent = *agent_iter;
		const size_t name = (agent)->id();
		// if (name == "") std::cerr << "Agent has no name. Correct this.\n";

		// Agent info.
    unsigned int agent_index = gSpatialDatabase->getCellIndexFromLocation(agent->position());
    unsigned int agent_x;
    unsigned int agent_z;
    gSpatialDatabase->getGridCoordinatesFromIndex(agent_index, agent_x, agent_z);
    // Goal info.
		const SteerLib::AgentGoalInfo& goal = agent->currentGoal();

		// If goals are of Group Goal (Target Region) process them.
    if (goal.goalType == SteerLib::GOAL_TYPE_GROUP_TARGET)
    {
    	if (_groups.find(goal.targetName) != _groups.end())
    	{
    		// Keep track of all Agents sharing this goal.
        _groups.find(goal.targetName)->second.agents.insert(name);
    		continue;
    	}

    	Groups group;
    	group.name = goal.targetName;
  		group.target_region = goal.targetRegion;
      // Grab the lower right and upper left coordinates of the goal region.
  		const Util::AxisAlignedBox& target_region = goal.targetRegion;
      Util::Point lower_right = Util::Point(target_region.xmin, 0.0f,  target_region.zmin);
      Util::Point upper_left = Util::Point(target_region.xmax, 0.0f, target_region.zmax);

			// Find the lower right and upper left grid coordinates of the goal region.
      unsigned int index = gSpatialDatabase->getCellIndexFromLocation(lower_right);
      unsigned int box_x;
      unsigned int box_z;
      gSpatialDatabase->getGridCoordinatesFromIndex(index, box_x, box_z);
      group.lower_right.x = box_x;
      group.lower_right.z = box_z;

      index = gSpatialDatabase->getCellIndexFromLocation(upper_left);
      gSpatialDatabase->getGridCoordinatesFromIndex(index, box_x, box_z);
      group.upper_left.x = box_x;
      group.upper_left.z = box_z;
      
      // Add this agent to this group goal.
      group.agents.insert(name);
      // Add this group goal.
      _groups[group.name] = group;
		} else {
			// If the goal is a single static location transform it to a single group goal location.
			// This allows detection of agents heading to the same goal.
      unsigned int goal_index = gSpatialDatabase->getCellIndexFromLocation(goal.targetLocation);
      unsigned int goal_x;
      unsigned int goal_z;
      gSpatialDatabase->getGridCoordinatesFromIndex(goal_index, goal_x, goal_z);
      // Use the name of the grid location as the name of the target region.
      std::stringstream stream;
      stream << goal_x << "," << goal_z;
    	if (_groups.find(stream.str()) != _groups.end()) {
        _groups.find(stream.str())->second.agents.insert(name);
    		continue;
			}

			// Convert single location to group goal.
			Groups group;
      group.name = stream.str();
      group.lower_right.x = goal_x;
      group.lower_right.z = goal_z;
      group.upper_left.x = goal_x;
      group.upper_left.z = goal_z;

      // Add this agent to this group goal.
      group.agents.insert(name);
      // Add this group goal.
      _groups[group.name] = group;
		}

		// We want at most 5 goal areas for speed of simulation so we will say that any goal
		// that is within 10 squares of another goal may be grouped into a goal area.
	}

  if (_groups.size() > MAX_NUMBER_OF_GROUPS) {
		std::cerr << "Number of Group goals: " << _groups.size() << " is larger than "
			<< MAX_NUMBER_OF_GROUPS << " and your simulation may be very slow." << endl;
	}
    gPhaseProfilers->createGroupingProfiler.stop();
}


//==============================================================================
// Density
//==============================================================================


// For each agent on the map compute their density and add it to the map.
// Density is discretized using the "splat" technique described in the Continuum
// Crowds paper.
void CCAIModule::computeDensityField()
{
  gPhaseProfilers->computeDensityFieldProfiler.start();
	// Keep track of cells that receive density.
	std::set< std::pair<int, int> > cell_history;

  std::vector<SteerLib::AgentInterface*> agents = gEngine->getAgents();
	std::vector<SteerLib::AgentInterface*>::iterator agent_iter;
  for (agent_iter = agents.begin(); agent_iter != agents.end(); ++agent_iter ) {
		// Skip diabled agent.
		if (!(*agent_iter)->enabled())
		  {
		    gPhaseProfilers->computeDensityFieldProfiler.stop();
		    return;
		  }

		SteerLib::AgentInterface* agent = *agent_iter;
		
		// Agent index.
    unsigned int agent_index = gSpatialDatabase->getCellIndexFromLocation(agent->position());

		// Grid Cell center point.
  	Util::Point center;
   	gSpatialDatabase->getLocationFromIndex(agent_index, center);
    unsigned int x_index;
    unsigned int z_index;
    gSpatialDatabase->getGridCoordinatesFromIndex(agent_index, x_index, z_index);

    // First the cell who's center is lower in both x and z is discovered. This
    // is the lower-right in this coordinate system.
  	if (agent->position().x > center.x) {
  		if (agent->position().z > center.z) {
    	  // case C
  			// do nothing.
  		} else {
  			// case B
  			z_index -= 1;
  		}
  	} else { // Now agent is less than center x valued.
  		if (agent->position().z > center.z) {
    	  // case D
  			x_index -= 1;
  		} else {
  			// case A
  			x_index -= 1;
  			z_index -= 1;
  		}
    }

		// Check lower-right grid cell and then neighboring 3.
		if (!checkBounds(x_index, z_index) || !checkBounds(x_index + 1, z_index)
		    || !checkBounds(x_index + 1, z_index + 1) || !checkBounds(x_index, z_index + 1)) {
		  // Corner or edge case so don't add density.
			continue;
		}
		// Grab lower-right cell and computer density splatting.
		unsigned int lower_right = gSpatialDatabase->getCellIndexFromGridCoords(x_index, z_index);
		Util::Point bl_center;
		gSpatialDatabase->getLocationFromIndex(lower_right, bl_center);
  
    float delta_x = fabs(agent->position().x - bl_center.x);
    float delta_z = fabs(agent->position().z - bl_center.z);
    if (delta_x == 1) delta_x = 0.999f;
    if (delta_z == 1) delta_z = 0.999f;
    // Compute per cell contribution.

	// MUBBASIR CHANGE 

    float density_A = pow(std::min(1 - delta_x, 1 - delta_z), DENSITY_DECAY);
    float density_B = pow(std::min(delta_x, 1 - delta_z), DENSITY_DECAY);
    float density_C = pow(std::min(delta_x, delta_z), DENSITY_DECAY);
    float density_D = pow(std::min(1 - delta_x, delta_z), DENSITY_DECAY);
    
    _grid[x_index][z_index].density += density_A;
    _grid[x_index + 1][z_index].density += density_B;
    _grid[x_index + 1][z_index + 1].density += density_C;
    _grid[x_index][z_index + 1].density += density_D;

		const Util::Vector& velocity = (agent)->velocity();

		// Sums all density scaled velocity, average it after.
    _grid[x_index][z_index].avg_velocity += velocity * density_A;
    _grid[x_index + 1][z_index].avg_velocity += velocity * density_B;
    _grid[x_index + 1][z_index + 1].avg_velocity += velocity * density_C;
    _grid[x_index][z_index + 1].avg_velocity += velocity * density_D;

		// Keep track of visited cells.
	  cell_history.insert(pair_int(x_index, z_index));
	  cell_history.insert(pair_int(x_index + 1, z_index));
	  cell_history.insert(pair_int(x_index + 1, z_index + 1));
	  cell_history.insert(pair_int(x_index, z_index + 1));
  }

	// Set average velocity per CELL.
	std::set< pair_int >::iterator iter;
	for (iter = cell_history.begin(); iter != cell_history.end(); ++iter) {
		if (_grid[iter->first][iter->second].density > 0.0f)
  		_grid[iter->first][iter->second].avg_velocity /= _grid[iter->first][iter->second].density;
	}
  gPhaseProfilers->computeDensityFieldProfiler.stop();
}


//==============================================================================
// Unit Cost
//==============================================================================


// Terrain speed.
float topologicalSpeed(float slope) {
	if (slope == std::numeric_limits<float>::infinity()) {
		return MIN_SPEED;
	}
	if (slope == -std::numeric_limits<float>::infinity()) {
		return MAX_SPEED;
	}
	return MAX_SPEED
		     + ((slope - MIN_SLOPE)/(MAX_SLOPE - MIN_SLOPE))
		     * (MIN_SPEED - MAX_SPEED);
}


// Speed field calculation based on density.
float speedField(float topological, float flow, float density) {
	if (density >= MAX_DENSITY) {
		return flow;
	} else if (density <= MIN_DENSITY) {
		return topological;
	}
 	return topological
       + ((density - MIN_DENSITY)/(MAX_DENSITY - MIN_DENSITY))
  		 * (flow - topological);
}


// Caculates the per face speed and unit cost from the current cell to the neighbor cell.
// This is called once per direction.
void faceUnitCost(CellFields& current, CellFields& neighbor, const Util::Vector& direction, float dt) {
 	// Save height gradient, speed out and cost.
 	AniostropicFields* cur_face = NULL;
 	if (direction == North) cur_face = &current.Nface;
	else if (direction == South) cur_face = &current.Sface;
	else if (direction == West) cur_face = &current.Wface;
	else if (direction == East) cur_face = &current.Eface;
	else std::cout << "ERROR: no such direction.\n";

	// Have slope take grid sizing for rise/run.
 	float slope = (neighbor.height - current.height) / ( 1.0f );
 	float topo_speed = topologicalSpeed(slope);
	float flow_speed = Util::dot(neighbor.avg_velocity, direction);
	// Clamp flow_speed.
	if (flow_speed < 0.0f) {
		flow_speed = 0.0f;
	}

	// Calculate speed field based on neighbor's density.
	float speed_field = speedField(topo_speed, flow_speed, neighbor.density);

	// Calculate unit cost.
	cur_face->height_gradient = slope * direction;
	cur_face->speed_out = speed_field;
	cur_face->cost_out = (PATH_WEIGHT * speed_field
 	                      + TIME_WEIGHT * dt
 	                      + DISCOMFORT_WEIGHT * neighbor.discomfort
 	                     )
	                      / speed_field;
}


// Compute  speed field and unit cost for the entire map grid.
void CCAIModule::computeUnitCost(const float dt) {
	for (unsigned int x = 0; x < _rows; ++x) {
		for (unsigned int z = 0; z < _columns; ++z) {
   		// Calculate topographical speed, flow speed and finally unit cost
   		// for each direction N S W E.  Slope/gradient corresponds to direction. 
  		CellFields& current = _grid[x][z]; 
  		// NORTH
  		if (checkBounds(x, z + 1)) {
  			CellFields& face = _grid[x][z + 1];
  			faceUnitCost(current, face, North, dt);
		  } else {
  			// Edge of grid, make it extreme?
  			current.Nface.speed_out = 0.0f;
  			current.Nface.cost_out = std::numeric_limits<float>::infinity();
  			current.Nface.potential_gradient = South;
  			current.Nface.velocity = South;
  		}
  		// SOUTH
  		if (checkBounds(x, z - 1)) {
  			CellFields& face = _grid[x][z - 1];
  			faceUnitCost(current, face, South, dt);
  		} else {
  			// Edge of grid, make it extreme?
  			current.Sface.speed_out = 0.0f;
  			current.Sface.cost_out = std::numeric_limits<float>::infinity();
  			current.Sface.potential_gradient = North;
  			current.Sface.velocity = North;
  		}
  		// WEST
  		if (checkBounds(x + 1, z)) {
  			CellFields& face = _grid[x + 1][z];
  			faceUnitCost(current, face, West, dt);
  		} else {
  			// Edge of grid, make it extreme?
  			current.Wface.speed_out = 0.0f;
  			current.Wface.cost_out = std::numeric_limits<float>::infinity();
  			current.Wface.potential_gradient = East;
  			current.Wface.velocity = East;
  		}
  		// EAST
  		if (checkBounds(x - 1, z)) {
  			CellFields& face = _grid[x - 1][z];
  			faceUnitCost(current, face, East, dt);
  		} else {
  			// Edge of grid, make it extreme?
  			current.Eface.speed_out = 0.0f;
  			current.Eface.cost_out = std::numeric_limits<float>::infinity();
  			current.Eface.potential_gradient = West;
  			current.Eface.velocity = West;
  		}
		}
	}
}


//==============================================================================
// Potential Field
//==============================================================================


// Simple quadratic equation solver.
// Returns infinity when un-solveable.
float quadraticFormula(double a, double b, double c) {
  if (a == 0.0) {
    return std::numeric_limits<float>::infinity();
  }

  double d_squared = (b*b) - (4*a*c);
  if (d_squared < 0.0) {
  	return std::numeric_limits<float>::infinity();
  }

  double d = sqrt( d_squared );
  double sol_one = (-b + d)/(2*a);
  double sol_two = (-b - d)/(2*a);

  if (sol_one > sol_two) {
  	return sol_one;
  } else {
	  cout << "Used 'negative' quardratic!?!: " << sol_two << " instead of: "<<sol_one<< "\n";
  	return sol_two;
	}
}


// Insert CellFields* in order of decreasing potentials.
// Used for Priority Queue ordering.
class FieldCompare {
	public:
		bool operator()(const CellFields* left, const CellFields* right) const {
			return (left->potential > right->potential);
		}
};


// Computes the potential of a single grid location as described in the Continuum Crowds
// paper.
float CCAIModule::computePotential(const pair_int& cur, const set< pair_int >& known) {
	// Current cell.
  unsigned ix = cur.first;
  unsigned iz = cur.second;

	// Find min potential North & South
	float north_adj = std::numeric_limits<float>::infinity();
	float south_adj = std::numeric_limits<float>::infinity();
	float cost_NS = std::numeric_limits<float>::infinity();
	float potential_NS = std::numeric_limits<float>::infinity();
	// North
  if (checkBounds(ix, iz + 1)) {
  	north_adj = _grid[ix][iz + 1].potential + _grid[ix][iz].Nface.cost_out;
  }
	// South
  if (checkBounds(ix, iz - 1)) {
  	south_adj = _grid[ix][iz - 1].potential + _grid[ix][iz].Sface.cost_out;
	}
  if (north_adj == std::numeric_limits<float>::infinity() &&
      south_adj == std::numeric_limits<float>::infinity()) {
		// Remove North-to-South term.
		potential_NS = std::numeric_limits<float>::infinity();
	} else if (north_adj <= south_adj) {
    // Use North values. Slight bias.
		potential_NS = _grid[ix][iz + 1].potential;
		cost_NS = _grid[ix][iz].Nface.cost_out;
	} else {
    // Use South values.
		potential_NS = _grid[ix][iz - 1].potential;
		cost_NS = _grid[ix][iz].Sface.cost_out;
	}

	// Find min potential East & West
	float west_adj = std::numeric_limits<float>::infinity();
	float east_adj = std::numeric_limits<float>::infinity();
	float cost_WE = std::numeric_limits<float>::infinity();
	float potential_WE = std::numeric_limits<float>::infinity();
	// West
  if (checkBounds(ix + 1, iz)) {
  	// West cell potential + Current cell cost going West.
  	west_adj = _grid[ix + 1][iz].potential + _grid[ix][iz].Wface.cost_out;
	}
	// East
  if (checkBounds(ix - 1, iz)) {
  	// East cell potential + Current cell cost going East.
  	east_adj = _grid[ix - 1][iz].potential + _grid[ix][iz].Eface.cost_out;
  }
  if (west_adj == std::numeric_limits<float>::infinity() &&
      east_adj == std::numeric_limits<float>::infinity()) {
		// Remove West-to-East term.
		potential_WE = std::numeric_limits<float>::infinity();
	} else if (west_adj <= east_adj) {
    // Use West values. Bias this direction.
		potential_WE = _grid[ix + 1][iz].potential;
  	cost_WE = _grid[ix][iz].Wface.cost_out;
	} else {
    // Use East values.
		potential_WE = _grid[ix - 1][iz].potential;
  	cost_WE = _grid[ix][iz].Eface.cost_out;
	} 

	if (potential_NS == std::numeric_limits<float>::infinity()
	   && potential_WE == std::numeric_limits<float>::infinity()) {
		return std::numeric_limits<float>::infinity();
	}

	// Compute upwind direction values via paper descirbed equations.
	double a, b, c;
	if (potential_NS == std::numeric_limits<float>::infinity()) {
		// Only use West-East term.
		a = 1 / (cost_WE * cost_WE);
		b = -2 * potential_WE * a;
		c = potential_WE * potential_WE * a - 1;
	} else if (potential_WE == std::numeric_limits<float>::infinity()) {
		// Only use Noth-South term.
		a = 1 / (cost_NS * cost_NS);
		b = -2 * potential_NS * a;
		c = potential_NS * potential_NS * a - 1;
	} else {
		double a_we = 1 / (cost_WE * cost_WE);
		double a_ns = 1 / (cost_NS * cost_NS);
		a = a_we + a_ns;
		b = -2 * potential_WE * a_we - 2 * potential_NS * a_ns;
		c = potential_WE * potential_WE * a_we  +  potential_NS * potential_NS * a_ns - 1;
	}

	return quadraticFormula(a, b, c);
}


// Calculate the potential field for the map grid. This is done for each group
// using the Fast Marching Algorithm.
void CCAIModule::potentialFieldConstruction(const Groups& group)
{
  // Clear out previous potentials.
	AutomaticFunctionProfiler profileThisFunction( &CCGlobals::gPhaseProfilers->potentialFieldConstructionProfiler );
	for (unsigned int i = 0; i < _rows; ++i)
	{
		for (unsigned int j = 0; j < _columns; ++j)
		{
  		CellFields& cell = _grid[i][j];
  		cell.potential = std::numeric_limits<float>::infinity();
		}
	}

  // Known cells whose potential (phi) value has been computed.
  std::set< pair_int > known;

  // Priority queue of CellField pointers. Ordered by lowest potiential.
  // As a standard priority queue does not re-heapify itself if the underlying
  // data is updated either make_heap or another approach is needed. Here the candidate
  // cell that is updated is re-added to the candidate cells. Since we are after the
  // smallest potential it is wise to only re-add candidate cells with lower potential.
  // Then when a candidate cell is at the top of the queue a simple set check will
  // determine if the candidate cell has already been processed at a cheaper potential.
  // This avoids calling heapify at a trivial memory cost.
	std::priority_queue< CellFields*, std::vector<CellFields*>, FieldCompare> candidate;

	// Include all squares comprising the goal in known and set phi to zero in them.
  for (int i = group.lower_right.x; i <= group.upper_left.x; ++i) {
  	for (int j = group.lower_right.z; j <= group.upper_left.z; ++j) {
  		_grid[i][j].potential = 0.0f;
      candidate.push(&_grid[i][j]);
		}
	}
 
 	// Process each candidate cell until all are completed and the entire map grid is
 	// marched over.
	while (candidate.size() != 0) {
    // Since some cells are re-approximated they may be in the queue more than once.
    // By definition a priority queue will keep th cheapest on top so any cell that
    // appears more than once is appearing at a higher cost and must be popped off.
    // The lowest potential cell goes first and is added to the set of known cells.
    if (known.find(candidate.top()->indices) != known.end()) {
      candidate.pop();
      continue;
  	}

  	// New known cell.
  	known.insert(candidate.top()->indices);
    unsigned int ix = candidate.top()->indices.first;
    unsigned int iz = candidate.top()->indices.second;
    unsigned int ax, az;
		
    // Keep track of a potential for range coloring.
    if (candidate.top()->potential > _potential_scale &&
        candidate.top()->potential != std::numeric_limits<float>::infinity()) {
      _potential_scale = candidate.top()->potential;
		}

  	candidate.pop();

  	// Compute phi of all UNKNOWN-adjacent-cells to last known cell.
    for (int i = 0; i < 4; ++i) {
    	// North
     	if (i == 0) {
   	  	ax = ix;
   	  	az = iz + 1;
     	// South
  	 	} else if (i == 1) {
        ax = ix;
        az = iz - 1;
      // West
		  } else if (i == 2) {
		  	ax = ix + 1;
		  	az = iz;
  		// East
  		} else {
  			ax = ix - 1;
  			az = iz;
  		}
      // Skip out of bounds. Skip known cells. Skip goal cells.
      if (checkBounds(ax, az) && known.find(pair_int(ax, az)) == known.end()
       		&& _grid[ax][az].potential != 0.0f) {
        CellFields& adj = _grid[ax][az];
        float phi = computePotential(adj.indices, known);
        if (phi < adj.potential) {
        	adj.potential = phi;
        	candidate.push(&adj);
	  	 	}
      }
		}
  }

  // Update the potential gradients.
  updatePotentialGradientVelocity();
}


// Now update potential gradient and velocity vectors for
// directions of lower potential.
void CCAIModule::updatePotentialGradientVelocity() {
	// Iterate through map grid and if current cell has a higher potential
	// than velocity is out of cell. Fix both cells out going and face cells
	// incoming gradient.
	for (unsigned int xi = 0; xi < _rows; ++xi) {
	  for (unsigned int zi = 0; zi < _columns; ++zi) {
	  	CellFields& cur = _grid[xi][zi];
			// North
      if (checkBounds(xi, zi + 1)) {
        CellFields& face = _grid[xi][zi + 1];
        if (cur.potential == 0.0f) {
        	cur.Nface.potential_gradient = South;
					cur.Nface.velocity = South * MAX_SPEED;
				} else if (cur.potential > face.potential) {
        	float grad = cur.potential - face.potential;
        	if (cur.potential == std::numeric_limits<float>::infinity()) {
        		grad = LARGE_POTENTIAL;
					}
        	cur.Nface.potential_gradient = North * grad;
					cur.Nface.velocity = Util::normalize(cur.Nface.potential_gradient) * cur.Nface.speed_out;
				}
				face.Sface.potential_gradient = cur.Nface.potential_gradient;
				face.Sface.velocity = cur.Nface.velocity;
			}
			// South
      if (checkBounds(xi, zi - 1)) {
        CellFields& face = _grid[xi][zi - 1];
        if (cur.potential == 0.0f) {
        	cur.Sface.potential_gradient = North;
					cur.Sface.velocity = North * MAX_SPEED;
				} else if (cur.potential > face.potential) {
        	float grad = cur.potential - face.potential;
        	if (cur.potential == std::numeric_limits<float>::infinity()) {
						grad = LARGE_POTENTIAL;
					}
        	cur.Sface.potential_gradient = South * grad;
					cur.Sface.velocity = Util::normalize(cur.Sface.potential_gradient) * cur.Sface.speed_out;

				}
				face.Nface.potential_gradient = cur.Sface.potential_gradient;
				face.Nface.velocity = cur.Sface.velocity;
			}
			// West
      if (checkBounds(xi + 1, zi)) {
        CellFields& face = _grid[xi + 1][zi];
        if (cur.potential == 0.0f) {
        	cur.Wface.potential_gradient = East;
					cur.Wface.velocity = East * MAX_SPEED;
				} else if (cur.potential > face.potential) {
        	float grad = cur.potential - face.potential;
        	if (cur.potential == std::numeric_limits<float>::infinity()) {
						grad = LARGE_POTENTIAL;
					}
        	cur.Wface.potential_gradient = West * grad;
					cur.Wface.velocity = Util::normalize(cur.Wface.potential_gradient) * cur.Wface.speed_out;
					if(cur.Wface.velocity.x != cur.Wface.velocity.x) {
						cout<<"vel: "<< cur.Wface.velocity<<" cur: "<<cur.potential<<" face: "<<face.potential;
						cout<<"donkey: "<<cur.Wface.potential_gradient<<" speed out: "<<cur.Wface.speed_out<<endl;
						cout<<"grad: "<<grad<<endl;
					}
				}

				face.Eface.potential_gradient = cur.Wface.potential_gradient;
				face.Eface.velocity = cur.Wface.velocity;
			}
			// East
      if (checkBounds(xi - 1, zi)) {
        CellFields& face = _grid[xi - 1][zi];
        if (cur.potential == 0.0f) {
        	cur.Eface.potential_gradient = West;
					cur.Eface.velocity = West * MAX_SPEED;
				} else if (cur.potential > face.potential) {
        	float grad = cur.potential - face.potential;
        	if (cur.potential == std::numeric_limits<float>::infinity()) {
						grad = LARGE_POTENTIAL;
					}
        	cur.Eface.potential_gradient = East * grad;
					cur.Eface.velocity = Util::normalize(cur.Eface.potential_gradient) * cur.Eface.speed_out;
				}

				face.Wface.potential_gradient = cur.Eface.potential_gradient;
				face.Wface.velocity = cur.Eface.velocity;
			}
		}
	}
}


// For each agent check if they are in the group that was run. If so update
// that agent's location.
void CCAIModule::updateLocations(const Groups& group) {
  const float x_offset = 0.5f * gSpatialDatabase->getGridSizeX() / ((float)_rows);
  const float z_offset = 0.5f * gSpatialDatabase->getGridSizeZ() / ((float)_columns);

  std::vector<SteerLib::AgentInterface*> agents = gEngine->getAgents();
  std::vector<SteerLib::AgentInterface*>::iterator agent_iter;
  for (agent_iter = agents.begin(); agent_iter != agents.end(); ++agent_iter )
  {
			// Skip diabled agent.
			if (!(*agent_iter)->enabled()) continue;

			SteerLib::AgentInterface* agent = *agent_iter;
			const int name = (agent)->id();

			// Check if this agent is part of this group.
			if (group.agents.find(name) == group.agents.end()) continue;

		unsigned int agent_index = gSpatialDatabase->getCellIndexFromLocation(agent->position());
		unsigned int agent_x;
		unsigned int agent_z;
		gSpatialDatabase->getGridCoordinatesFromIndex(agent_index, agent_x, agent_z);

		if (!checkBounds(agent_x, agent_z)) {
			cout << "Cannot updated agent location. X,Z: " << agent_x << "," << agent_z << endl;
			return;
		  }
		Util::Point center;
		gSpatialDatabase->getLocationFromIndex(agent_index, center);

			// Grab distance from agent to faces.
		  float nDist = Util::distanceBetween(Util::Point(center.x, 0.0f, center.z + z_offset), agent->position());
		float sDist = Util::distanceBetween(Util::Point(center.x, 0.0f, center.z - z_offset), agent->position());
		float wDist = Util::distanceBetween(Util::Point(center.x + x_offset, 0.0f, center.z), agent->position());
		float eDist = Util::distanceBetween(Util::Point(center.x - x_offset, 0.0f, center.z), agent->position());
		float total = nDist + sDist + wDist + eDist;

			// Grab velocities per face.
		Util::Vector& north = _grid[agent_x][agent_z].Nface.velocity;
		Util::Vector& south = _grid[agent_x][agent_z].Sface.velocity;
		Util::Vector& west = _grid[agent_x][agent_z].Wface.velocity;
		Util::Vector& east = _grid[agent_x][agent_z].Eface.velocity;

		if (total <= 0.0f) {
			total = 0.01f;
		}

			// Interpolate velocitites.
		Util::Vector vel = north * (1 - nDist/total) + south * (1 - sDist/total)
							   + west * (1 - wDist/total) + east * (1 - eDist/total);


		// TODO this is a problem for the HybridAI
		CCAgent * ccAgent = dynamic_cast<CCAgent*>(agent);
		if ( ccAgent != NULL )
		{
			// std::cout << "CC updating velocity" << std::endl;
			ccAgent->setPlanVelocity(vel);
		}
	}
}


void CCAIModule::processKeyboardInput(int key, int action ) {
	//cout << "Pressed: " << key << " action: " << action << "\n";

	// 44 comma key
	if (key == 44 && action == 1) _comma_pressed = !_comma_pressed;
	// 46 period key
	if (key == 46 && action == 1) _period_pressed = !_period_pressed;
	// forward slash key
	if (key == 47 && action == 1) _forward_slash_pressed = !_forward_slash_pressed;
	// m key
	if (key == 77 && action == 1) _m_pressed = !_m_pressed;
	// n key
	if (key == 78 && action == 1) {
		_n_pressed = !_n_pressed;
		_potential_draw += 3.0f;
	}
	// b key
	if (key == 66 && action == 1){
		_b_pressed = !_b_pressed;
		_potential_draw -= 3.0f;
	}
}


void CCAIModule::draw() {
#ifdef ENABLE_GUI
  const float x_offset = 0.5f * gSpatialDatabase->getGridSizeX() / ((float)_rows);
  const float z_offset = 0.5f * gSpatialDatabase->getGridSizeZ() / ((float)_columns);

	// Draw goals
  Util::Color color(0.4f, 0.9f, 0.4f);
  for (std::map< std::string, Groups >::const_iterator iter = _groups.begin();
	     iter != _groups.end();
       ++iter) {
    const Groups& group = iter->second;
    for (unsigned int i = group.lower_right.x; i <= group.upper_left.x; ++i) {
      for (unsigned int j = group.lower_right.z; j <= group.upper_left.z; ++j) {
        unsigned int cell_index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
       
		//Util::DrawLib::glColor(color);
	 		  Util::Point here;
 	   	  gSpatialDatabase->getLocationFromIndex(cell_index, here);
 	 		  Util::DrawLib::drawBox(here.x - x_offset, here.x + x_offset, 0.0f, 0.05f,
   	                           here.z - z_offset, here.z + z_offset);
 	 		  Util::DrawLib::drawFlag(here);
			}
		}
	}

  // NOTE all drawing uses the data from the last group to run through the algorithm.
	for (unsigned int x_ind = 0; x_ind < _rows; ++x_ind) {
	  for (unsigned int z_ind = 0; z_ind < _columns; ++z_ind) {
      unsigned int cell_index = gSpatialDatabase->getCellIndexFromGridCoords(x_ind, z_ind);
			Util::Point here;
 	  	gSpatialDatabase->getLocationFromIndex(cell_index, here);
 	  	CellFields& cur = _grid[x_ind][z_ind];

  	  // Draw density
  		if (!_comma_pressed && cur.density  >= 0.01 * DENSITY_THRESHOLD) {
		    Util::Color color(0.6f, cur.density, 0.7f);
  	    //Util::DrawLib::glColor(color);
  	    float space = 0.05f;
  			Util::DrawLib::drawBox(here.x - x_offset + space , here.x + x_offset - space , -0.05f, 0.02f,
  					                   here.z - z_offset + space , here.z + z_offset - space );
  		}

  		// Draw cost
  		if (_forward_slash_pressed) {
  			Util::Color color(0.3f, 0.7f, 0.8f);
   		  //Util::DrawLib::glColor(color);
        if (cur.Nface.cost_out == std::numeric_limits<float>::infinity())
    			Util::DrawLib::drawBox(here.x - .05f, here.x + .05f, 0.0f, 1.0f,
    					                   here.z + z_offset -.05f, here.z + z_offset);
        if (cur.Sface.cost_out == std::numeric_limits<float>::infinity())
  	  		Util::DrawLib::drawBox(here.x - .05f, here.x + .05f, 0.0f, 1.0f,
  	  				                   here.z - z_offset -.05f, here.z - z_offset);
        if (cur.Wface.cost_out == std::numeric_limits<float>::infinity())
  		  	Util::DrawLib::drawBox(here.x + x_offset, here.x + x_offset + .05f, 0.0f, 1.0f,
  		  			                   here.z -.05f, here.z + .05f);
        if (cur.Eface.cost_out == std::numeric_limits<float>::infinity()) 
    			Util::DrawLib::drawBox(here.x - x_offset, here.x - x_offset + .05f, 0.0f, 1.0f,
    					                   here.z - .05f, here.z + .05f);
			}
	
      // Draw potential
      if (_period_pressed) {
      	float pot = cur.potential / _potential_scale ;
				Util::Color color(0.3f, 0.4f, pot);
  		  //Util::DrawLib::glColor(color);
  	    float space = 0.2f;
  			Util::DrawLib::drawBox(here.x - x_offset + space, here.x + x_offset - space, 0.0f, 0.00f,
  					                   here.z - z_offset + space, here.z + z_offset - space);
			}

      // Draw discomfort
      if (_m_pressed && cur.discomfort == std::numeric_limits<float>::infinity()) {
				Util::Color color(0.4f, 0.3f, 0.6f);
  		  //Util::DrawLib::glColor(color);
  	    float space = 0.3f;
  			Util::DrawLib::drawBox(here.x - x_offset + space, here.x + x_offset - space, 0.8f, 1.4f,
  					                   here.z - z_offset + space, here.z + z_offset - space);
			}

      // Draw potential fast marching step
      if (cur.potential <= _potential_draw) {
				Util::Color color(0.35f, 0.31f, 0.62f);
  		  //Util::DrawLib::glColor(color);
  			Util::DrawLib::drawBox(here.x - x_offset, here.x + x_offset, -0.1f, 0.0f,
  					                   here.z - z_offset, here.z + z_offset);
			}
		}
	}
#endif
}
