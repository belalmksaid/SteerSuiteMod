//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

/*
 * HIDAC_Parameters.h
 *
 *  Created on: 2013-04-11
 *      Author: glenpb
 */

#ifndef HIDAC_PARAMETERS_H_
#define HIDAC_PARAMETERS_H_

// #include "testcaseio/Behaviour.h"
/*
#define ACCELERATION 0.4
#define PERSONAL_SPACE_THRESHOLD 0.3
#define AGENT_REPULSION_IMPORTANCE 0.7
#define QUERY_RADIUS 4.0f
#define BODY_FORCE 1.3f
#define AGENT_BODY_FORCE 8.0f
#define SLIDING_FRICTION_FORCE 0.6
#define AGENT_B 500.0f // Yep its just called B... inverse proximity force importance
#define AGENT_A 1.0f // Yep its just called A... inverse proximity force importance
#define WALL_B 500.0f // Yep its just called B... inverse proximity force importance
#define WALL_A 1.0f // Yep its just called A... inverse proximity force importance
*/
#define MAX_SPEED 2.6f
#define PERFERED_SPEED 1.33 // TODO not added to parameters yet.


#define ACCELERATION 2.0 // = v/A
#define PERSONAL_SPACE_THRESHOLD 0.3 // not defined in HiDAC papaer
#define AGENT_REPULSION_IMPORTANCE 0.3 // in HiDAC
#define QUERY_RADIUS 3.0f // not defined in paper
#define BODY_FORCE 1500.0f // K (big K) 120000 / 80
#define AGENT_BODY_FORCE 1500.0f
#define SLIDING_FRICTION_FORCE 3000.0f // k (small k) 240000 / 80 = 3000
#define AGENT_B 500.f // 0.08f // inverse proximity force importance
#define AGENT_A 25.0f // 2000 / 80 Yep its just called A... inverse proximity force importance
#define WALL_B 500.0f //  inverse proximity force importance
#define WALL_A 25.0f //  proximity force importance
#define MASS 1

#define USE_PLANNING 1
// #define DRAW_ANNOTATIONS 1

namespace HIDACGlobals {

	struct PhaseProfilers {
		Util::PerformanceProfiler aiProfiler;
		Util::PerformanceProfiler drawProfiler;
		Util::PerformanceProfiler longTermPhaseProfiler;
		Util::PerformanceProfiler midTermPhaseProfiler;
		Util::PerformanceProfiler shortTermPhaseProfiler;
		Util::PerformanceProfiler perceptivePhaseProfiler;
		Util::PerformanceProfiler predictivePhaseProfiler;
		Util::PerformanceProfiler reactivePhaseProfiler;
		Util::PerformanceProfiler steeringPhaseProfiler;
	};


	extern SteerLib::EngineInterface * gEngineInfo;
	extern SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
	extern unsigned int gLongTermPlanningPhaseInterval;
	extern unsigned int gMidTermPlanningPhaseInterval;
	extern unsigned int gShortTermPlanningPhaseInterval;
	extern unsigned int gPredictivePhaseInterval;
	extern unsigned int gReactivePhaseInterval;
	extern unsigned int gPerceptivePhaseInterval;
	extern bool gUseDynamicPhaseScheduling;
	extern bool gShowStats;
	extern bool gShowAllStats;


	// Adding a bunch of parameters so they can be changed via input
	extern float hidac_acceleration;
	extern float hidac_personal_space_threshold;
	extern float hidac_agent_repulsion_importance;
	extern float hidac_query_radius;
	extern float hidac_body_force;
	extern float hidac_agent_body_force;
	extern float hidac_sliding_friction_force;
	extern float hidac_agent_b;
	extern float hidac_agent_a;
	extern float hidac_wall_b;
	extern float hidac_wall_a;
	extern float hidac_max_speed;



	extern PhaseProfilers * gPhaseProfilers;
}


class HIDACParameters
{
public:
	// Adding a bunch of parameters so they can be changed via input
	float hidac_acceleration;
	float hidac_personal_space_threshold;
	float hidac_agent_repulsion_importance;
	float hidac_query_radius;
	float hidac_body_force;
	float hidac_agent_body_force;
	float hidac_sliding_friction_force;
	float hidac_agent_b;
	float hidac_agent_a;
	float hidac_wall_b;
	float hidac_wall_a;
	float hidac_max_speed;

	void setParameters(SteerLib::Behaviour behavior)
	{
		// std::cout << "Setting parameters from behaviour" << std::endl;
		int i;
		for ( i = 0; i < behavior.getParameters().size(); i++)
		{
			std::string p_key = behavior.getParameters().at(i).key;
			std::stringstream value(behavior.getParameters().at(i).value);
			// std::cout << "key: " << p_key << ", value: " << value << std::endl;
			if (p_key == "hidac_acceleration")
			{
				value >> hidac_acceleration;
				std::cout << "set hidac acceleration to " << hidac_acceleration << std::endl;
			}
			else if (p_key == "hidac_personal_space_threshold")
			{
				value >> hidac_personal_space_threshold;
			}
			else if (p_key == "hidac_agent_repulsion_importance")
			{
				value >> hidac_agent_repulsion_importance;
			}
			else if (p_key == "hidac_query_radius")
			{
				value >> hidac_query_radius;
			}
			else if (p_key == "hidac_body_force")
			{
				value >> hidac_body_force;
			}
			else if (p_key == "hidac_agent_body_force")
			{
				value >> hidac_agent_body_force;
			}
			else if (p_key == "hidac_sliding_friction_force")
			{
				value >> hidac_sliding_friction_force;
			}
			else if (p_key == "hidac_agent_b")
			{
				value >> hidac_agent_b;
			}
			else if (p_key == "hidac_agent_a")
			{
				value >> hidac_agent_a;
			}
			else if (p_key == "hidac_wall_b")
			{
				value >> hidac_wall_b;
			}
			else if (p_key == "hidac_wall_a")
			{
				value >> hidac_wall_a;
			}
			else if (p_key == "hidac_max_speed")
			{
				value >> hidac_max_speed;
			}
		}
		
	}
};

inline std::ostream &operator<<(std::ostream & out, const HIDACParameters & p)
{ // methods used here must be const
	out << "hidac_acceleration: " << p.hidac_acceleration << std::endl;
	out << "hidac_personal_space_threshold: " << p.hidac_personal_space_threshold << std::endl;
	out << "hidac_agent_repulsion_importance: " << p.hidac_agent_repulsion_importance << std::endl;
	out << "hidac_query_radius: " << p.hidac_query_radius << std::endl;
	out << "hidac_body_force: " << p.hidac_body_force << std::endl;
	out << "hidac_agent_body_force: " << p.hidac_agent_body_force << std::endl;
	out << "hidac_sliding_friction_force: " << p.hidac_sliding_friction_force << std::endl;
	out << "hidac_agent_b: " << p.hidac_agent_b << std::endl;
	out << "hidac_agent_a: " << p.hidac_agent_a << std::endl;
	out << "hidac_wall_b: " << p.hidac_wall_b << std::endl;
	out << "hidac_wall_a: " << p.hidac_wall_a << std::endl;
	out << "hidac_max_speed: " << p.hidac_max_speed;

	return out;
}



#endif /* HIDAC_PARAMETERS_H_ */
