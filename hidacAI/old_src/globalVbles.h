//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

//****************************************************************************//
// globalVbles.h                                                        //
// Copyright (C) 2007 Nuria Pelechano                                        //
//****************************************************************************//
// This code is covered by a disclosure agreement filed by the 
// University of Pennsylvania.  Do not distribute with out permission of
// the author.  
//****************************************************************************// 
#ifndef _GLOBALVBLES_H_
	#define _GLOBALVBLES_H_


#include <vector>	
#include <list>	
using namespace std;

#include <fstream>

#define SLEEP		50000000


#ifndef ABS
#define	ABS(a)	        ( (a) >= 0.0 ? (a) : (-1.0)*(a) )
#endif

#ifndef LENGTH
#define LENGTH(a,b)		(sqrt(a*a + b*b))
#endif

#define MAZE2	false
#define MAZE3	false
//#define MYDEBUG	//false
#define WTC		false


//#define REPS 5	// Number of iteration for each case
//int num_repet = 0;

#define BUILDING	1
#define STREET		2
//int environment = BUILDING; // OJO
#define environment BUILDING

#define NUM_DOORS 15
#define NUM_HAZARDS 4 // 8
#define MAZE_ROWS	10//10
#define MAZE_COLS	10//10
#define SMOOTH_MOVE  true
#define NUM_tVectorS	10   // Number of steps between cells

// The search type could be RANDOM or DFS
#define RANDOM		1
#define DFS			2
//const int searchType =	DFS;  // OJO
#define searchType DFS

// enable/disable DFS communcicaion
#define DFScom true


#define LINEAR_INTERP	1
#define HELBING			2
//const int local_motion = LINEAR_INTERP;
#define local_motion LINEAR_INTERP

#define LEADERSHIP	true // true	// enable or disable leadership roles:
#define TRAINED		true // true	//enables or disables trained agents
//#define COMMUNICATION	//false //true
#define PANIC	false //true
#define AVOID_SHAKING true
#define WAITING_RULES true

// models for the simulation
#define HIDAC			0
#define SOCIAL_FORCES	1
#define RULE_BASED		2
#define CELL_AUTOMATA	3

// Force Field model:
/*
float OFFSET_WALL		= 0.3; // 0.3; //0.5;777777 
float OFFSET_OBSTACLES	= 0.8;
float OFFSET_PORTALS	= 0.3;
float OFFSET_HUMS		= 0.5;
*/
#define OFFSET_WALL			0.3 //0.25 //0.3 // 0.3; //0.5;
#define OFFSET_OBSTACLES	0.3 //0.25 //0.6 // 0.8
#define OFFSET_PORTALS		0.3
#define OFFSET_HUMS			0.5 //0.57 //0.55 // 0.5
#define USE_TEXTURES 

///// DRAWING DETAILS //////
#define DRAWDOORS true
#define WIDTH_WALL 0.2 //0.1 //0.3
#define HEIGHT_WALL 4.0 // 2.5 //2 // 0.2  //5  //Also HEIGHT_CELL in building3D.h
#define DRAW3D true
////////////////////////////

// Glogal variables to know how long it takes in simulation steps to evacuate the entire building:
/*
ofstream out, out2, outWTC;
ofstream script,mazeFile;
int globalTimeSteps = 0;
long globalTimeMillisecs = 0;
long globalOldTimeMillisecs = 0;
list<int> lhumsEvac;
bool finished = false;
*/
/*
class globalVariables
{
public:
	globalVariables():globalTimeSteps(0),globalTimeMillisecs(0),globalOldTimeMillisecs(0),finished(false){}; //{globalTimeMillisecs:0};
	ofstream out, out2, outWTC;
	ofstream script,mazeFile;
	int globalTimeSteps; // = 0;
	long globalTimeMillisecs; // = 0;
	long globalOldTimeMillisecs; // = 0;
	list<int> lhumsEvac;
	bool finished; // = false;
};

globalVariables globalVbles;
*/
#define VR_USER	true
#define TEST_CROWDS

#define USE_VR 1
//funda #define CAL3D_RENDERING 
//#define REAL_TIME false // true
//int dif_millSecs = 40; //25;// 20;
#define dif_millSecs 20 //10 //20
#define TRANSPARENCY 1.0 //0.5

//////////////////////////////////////
// Variables needed to get statistics:
#define REPS			1//25		// Number of iteration for each case
#define INIT_N_HUMS		5 // 800//10 //6 //80 // 25//150 // 100 // 20
#define INCR_N_HUMS		0	//20
#define MAX_N_HUMS		500 // 25//150 //100// 200
#define INIT_PERC_EVAC	8
#define INCR_PERC_EVAC  5
#define MAX_PERC_LEADERS	50 //25 //100


// HISTOGRAMAS:
#define INIT_TIME	5
#define INCR_TIME	5

#define LOOP_DFS_REPS 1//15


// VALIDATION DETAILS. MAY 2007
#define DENSITY_RADIUS_MIN	0.75
#define DENSITY_LIMIT		3.5		// Used to visualize when different densities are detected
//#define DEBUG_VALIDATION
//#define DEBUG_DENSITY		// Draws point of intensity red on top of each agent, indicating the density
#define PRINT_EVACUATION_TIMES
#define VALIDATION_OUTPUT
//#define RESUTS_MAZE
#define SECS 5.0

// information for OCEAN model
#define MIN 0
#define MED 1
#define MAX 2


#define TASK_COUNT 1

/*
typedef struct materialStruct {
   GLfloat ambient[4];
   GLfloat diffuse[4];
   GLfloat specular[4];
   GLfloat shininess;
} materialStruct;
*/
/*
materialStruct brassMaterials = {
    {0.33F, 0.22F, 0.03F, 1.0F},
    {0.78F, 0.57F, 0.11F, 1.0F},
    {0.99F, 0.91F, 0.81F, 1.0F},
    27.8F
};
*/
/*
materialStruct redPlasticMaterials = {
    {0.8F, 0.1F, 0.1F, 1.0F},
    {0.5F, 0.0F, 0.2F, 1.0F},
    {0.6F, 0.0F, 0.0F, 1.0F},
    32.0F
};


materialStruct redPlasticMaterials = {
    {0.3F, 0.0F, 0.0F, 1.0F},
    {0.6F, 0.0F, 0.0F, 1.0F},
    {0.8F, 0.6F, 0.6F, 1.0F},
    32.0F
};

materialStruct colorCubeMaterials = {
    {0.4F, 0.4F, 0.4F, 1.0F },
    { 0.4F, 0.4F, 0.4F, 1.0F },
    { 0.8F, 0.8F, 0.8F, 1.0F },
    {100.0F}
};
*/
/*
materialStruct colorCubeMaterials = {
    {0.3F, 0.1F, 0.4F, 1.0F },
    { 0.4F, 0.1F, 0.8F, 1.0F },
    { 0.4F, 0.1F, 0.8F, 1.0F },
    {100.0F}
};
*/


// Rendering things for debugging purposes:
//#define DEBUG_CROSSING_DOORS

#endif
