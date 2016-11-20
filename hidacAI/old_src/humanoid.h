//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

//****************************************************************************//
// humanoid.h                                                        //
// Copyright (C) 2007 Nuria Pelechano                                        //
//****************************************************************************//
// This code is covered by a disclosure agreement filed by the 
// University of Pennsylvania.  Do not distribute with out permission of
// the author.  
//****************************************************************************// 
#ifndef _HUMANOID_H_
#define _HUMANOID_H_

#include <sys/timeb.h>
////#include "cell.h"
//#include "maze.h"
#include "building.h"

#include "Trait.h"

class Agent;
class Model;

//extern long globalTimeMillisecs;

class CCell;

// Global variables:
//extern list<int> lhumsEvac;
//extern bool RENDER2D;   // if false then the render will be in 3D	
//extern int globalNumSimulations;

// crossing states: A is the attractor point before crossing the door, and B is the attractor point once the door is crossed
#define NOT_CROSSING	-1
#define APPROACHING_A	 0
#define REACHING_A		 1
#define CLOSE_TO_A		 2
#define LEAVING_B		 3


// Will affect the behavior of the humanoid
struct sLeadingBehavior{
	int leadership;  // 100 always decides, 0 always follows
	bool trained; 
};

struct TASKS
{
	tVector		location;		// position where the agent needs to be located before starting the action.
	float		proximity;		// how close i need to be of that location to be able to perform the action
	tVector		orientation;	// orientation that the agent needs to be facing before starting the action.
	list<int>	laction;		// List of cal3D actions to perform during this task.
	float		aux_actionTimer;// Timer to keep track of the overall time that the action has been being executed
	float		duration;		// How long the action lasts
	float		timer;			// When the action starts, timer is set to duration, and then every simulation step it decreases depending on the increment of time
	int			roomID;			// id of the room where the action needs to be performed
};

/*
struct ltstr
{
  bool operator()(const char* s1, const char* s2) const
  {
    return strcmp(s1, s2) < 0;
  }
};
*/

#define VISITED		1
#define FINISHED	2



class Chumanoid : public SteerLib::AgentInterface
{
public:
	Chumanoid();
	Chumanoid(bool know, int currCell, int prevCell,sLeadingBehavior pers);
	//funda Agent *mAgent;  // Jan: Points back to the PAR agent (init'ed when PAR crowds are created
	CTrait	mTrait;
	void setSpeed(double speed);
	void	increaseSpeed();
	void updateHumanoid(int ID, bool know, int currCell, int prevCell,float color[3],sLeadingBehavior pers,int panic);
	int  currentCell();
	int  previousCell();
	bool recalculateNextCell();
	bool lookForAlternativeDoor();
	bool chooseNextCell();
	void addBlockedCell(int newBlockedCells);
//	void setEnvironment(Cmaze *maze);
	void setEnvironment(Cbuilding *building);
	void runSimulationStep();
	void draw();
	void drawVR();
	void updateBlockedList();
	void updateClosedPortals();
	int  blocked(list<pairIDs> path);
	bool doorsClosed(list<pairIDs> path);
	int  gethumID() {return m_humID;}
	float getSpeed() {return m_speed;}
	tVector getAttractor(){return m_attractor;}
	void setAttractor(tVector attractor){m_attractor = attractor;}
	
	//void getinfoFromCell();			// The cell in which i am currently will use this function everytime some humanoid 
	//								// enters the cell and wants to communicate with me
	void enterCell();
	void exitCell();

	void insertCoords(pair<int,int> newCoord);
	int getLevelKnown(pair<int,int> newCoord);
	void insertFinishedCells(pair<int,DFSdata> newdataDFS);

	// Functions needed when the agent is lost:
	bool findRandomCell();  // We need this to decide where to go when all the paths are blocked
	void findDFScell();		// Depth first search algorithm used when the humanoid is lost.
	void createDFSnode(int parent);	// To create a DFS node. It is used when the planning is done in a way that is not the DFS search.
	void resetTimeDFS();
	bool NodeInParentsPath(int finishedNode, int currentParent, int steps); // To find out whether that node is in my list of nodes towards the root of the tree
	bool updateNodesInParentsPath(int finishedNode,int currentParent,int steps);
	void forgetFINISHEDcells();
	tVector calculateSteps();
	tVector calculateStepsSocialForces();
	tVector calculateStepsHelbing();
	tVector calculateStepsRuleBased();
	tVector calculateStepsCellularAutomata();
	int num_step;	// Used to know for how long i've been trying to cross a portal without succeed

	sLeadingBehavior		getPersonality();
	void			setPersonality(sLeadingBehavior pers);

	void createNewObstacle();
	void updateHumanoidRendering(float res,float bodyW, float bodyH, float HeadH, float legsH, float legsW);
	inline tVector getPosition()				{return mPos;};
	inline void setPosition(tVector pos)		{mPos = pos;};
	inline void setInitialPosition(tVector pos){mInitialPosition = pos;};
	inline tVector getInitialPosition()		{return mInitialPosition;};
	inline tVector getPrevPosition()			{return m_prevpos;};
	inline void setPrevPosition(tVector pos)	{m_prevpos=pos;};
	inline tVector getVelocity()				{return m_desiredVel;};
	inline void setVelocity(tVector vel)		{m_orientation = vel;};
	inline float getDensityAhead()			{return m_densityAhead;};
	inline void  setDensityAhead(float d)	{m_densityAhead=d;};
	
	inline tVector getOrientation()			{return m_orientation;};
	tVector lookAhead4Agents(tVector pos);
	//void initiateRays();
	//void updateRays();		// Updates the points at angles 0,30,60.... based on the current desired dir
	inline int getCrossingState()			{return m_crossingState;};
	inline bool doorCrossedFR()				{if (m_doorCrossedFR){m_doorCrossedFR=false; return true;}else return false;}; // returns whether the door has been crossed to measure frame rates during validation, and sets the value to False in order to count each person only once
	inline tVector getForceAvoidPerson()		{return m_forceAvoidPerson;};
	inline void setForceAvoidPerson(tVector force){m_forceAvoidPerson = force;};
	inline void setCloserPerson(Chumanoid* phum){m_closerHum = phum;};
	inline void setLastUP(tVector up){m_lastUP=up;};
	inline tVector getLastUP(){return m_lastUP;};

	inline bool reachedGoal() {return m_reached_goal;};
	bool portalBottleNeck();  // Based on current bottleneck and my patiece level i check whether i should pick another portal

	// Get and Set for personality parameters:
	int getOthersFirst();
	void setOthersFirst(int othersFirst);
	
	// Cal3d code:
	bool initCal3d(int numhum);

// code for PARs 
	void setGoal(int id);
	void removeTasks();

	
	
	

	
	inline int getWaitingTimerCurr(){return m_waiting_timer_curr;};

	inline void startWaitingTimer(){m_waiting_timer_curr = mTrait.GetWaitingTimer();};
	inline void decreaseWaitingTimer(){m_waiting_timer_curr>0 ? m_waiting_timer_curr-- : 0;};
// END Functions for OCC model

	// VALIDATION:
	inline float getDensityR1(){return m_densityR1;};
	inline void  setDensityR1(float density) {m_densityR1=density;};
	inline float getDiffTime(){return m_dt;};

//accessor methods for setting colors
	void setShirtColor(float r, float g, float b, float a) {m_color[0] = r; m_color[1] = g; m_color[2] = b; m_color[3] = a;};
	void setTrouserColor(float r, float g, float b, float a) {m_colorTrousers[0] = r; m_colorTrousers[1] = g; m_colorTrousers[2] = b; m_colorTrousers[3] = a;};
	void setHairColorOriginal(float r, float g, float b, float a) {m_colorHairOriginal[0] = r; m_colorHairOriginal[1] = g; m_colorHairOriginal[2] = b; m_colorHairOriginal[3] = a;};
	void setLegColor(float r, float g, float b, float a) {m_colorLegs[0] = r; m_colorLegs[1] = g; m_colorLegs[2] = b; m_colorLegs[3] = a;};

	// VR_USER functions
	bool canUserMove(float *newPos);
	bool assignNewCell(float *newPos);

	list<TASKS> m_tasks;	// List of tasks to perform
	bool		m_doTask;		//true while a tast is being carried out
	bool		m_execute_idle; // execute animation cycle
	bool		m_nextTask;		// there's a task to perform in the current cell (m_prevCell)

private:
	void renderHuman2D(float posX, float posY, float posZ);
	void renderHuman3D(float posX, float posY, float posZ);
	void renderHumanCal3D(float posX, float posY, float posZ);
	void calculateLookAheadEnds();

private:
	int		m_humID;

	// I'll be using hash tables instead of lists
	//map<string,int>	m_blockedTable; // Hash table with blocked cells. The number indecates the level in which i know about that cell
											// 0 means i don't know, 1 means i know because i saw it and 2 means somebody told me

	map<int,int>	m_blockedTable; // Hash table with blocked cells. The second number indecates the level in which i know about that cell
									// 0 means i don't know, 1 means i know because i saw it and 2 means somebody told me
	map<int,int>    m_listClosedPortals;	// What portals i remember to be closed (second number meaning same thing as above)

	bool	m_knowledge;			// 1 Knows the building, 0 naive agent (in the future could be a percentage of knowledge)
	int		m_myCell;			// Cell towards which i'm walking	
	int 	m_prevCell;			// Cell in which i am currently
	int		m_nextPortal;		// ID of the next portal i'm planning to cross
	// For walking withing the cell:
	tVector	m_attractor;
	tVector	m_prevAttractor;   // will be the attractor before crossing the portal, so we can calculate distances to both

	// I need to know the last attractor to remove the agents from the crossing portal list
	tVector m_lastAttracPos;		// attractor of the last portal crossed, needed to update m_crossingState
	int  m_lastAttracID;
	int  m_portalIDcoll;

	//Cmaze	*m_maze;				// The humanoid needs to have access to the environment.
	Cbuilding	*m_building;				// The humanoid needs to have access to the environment.

	CCell	*m_pcell;		// cell towards which i'm going
	CCell	*m_pprevCell;	// cell in which i am currently
	CCell	*m_pLastCell;	// last cell i was in (i need it for collision detection with other agents around portals)
	int		m_levelKnowledge;	// How much i know about the environment TAMPOCO SE USA, Y CREO Q ESTA REPETIDO

	
//	list<coord> m_lastCells;		// We'll save the last n cells of my path to avoid going back to the cell from where we came.
//	int m_numLastCells;

	// When the humanoid is lost, he'll start exploring the maze using a DFS algorithm:
	//map<string, DFSdata>	m_visitedCells;
	map<int, DFSdata>	m_visitedCells;	// hash table that for each cell i know about, i can access it's info and it's adjacent cells
	int						timeDFS;

	//
	list<tVector> m_route;	// List of steps walked by the humanoid

	// Personality factor:
	sLeadingBehavior m_personality;

	// Elements needed to apply helbing's model:
	double  m_speed;
	double  m_maxSpeed;
	double  m_densityAhead;		//	Density ahead of the agent. Is calculated as the number of agents in the semicircle ahead of the 
								// agent (dist<R, and vec(agent,others)dotVel > 0) and divided by PIxRxR/2
	double m_dt;				// diferential of time between simulations
	//float  m_time;
	long m_time;

	
	// For drawing purposes only:
	float	m_color[4];		// color to render the humanoid
	float	m_colorTrousers[4];		// color to render the humanoid
	float   m_colorHairOriginal[4];		// color hair //original color of hair
	float   m_colorLegs[4];
	
	float	m_incrx, m_incrz;
public: //funda
	tVector	mInitialPosition;
	tVector	mPos;
	tVector	m_prevpos;
	list<tVector> m_listPrevPos;
private:
	tVector    m_desiredVel;	// Vector in the desired velocity
	tVector	m_orientation; // Orientation of the Agent, to avoid using the m_desiredVel to specify the orientation, since that one has a lot of shaking
	tVector	m_lookAheadA, m_lookAheadB;	// the 2 extremes of the cone in which i'm looking for collisions ahead.
	tVector	m_vecA, m_vecB;
	int		m_pointSize;
	float	humBodyWidth, humBodyHeight;
	float	humRes, humHeadHeight;
	float	humLegsHeight, humLegsWidth;
	int		m_step_legs;
	bool m_doorCrossed;	// to know whether a door has been crossed and therefore the next cell neds to be calculated
	bool m_crossingPortal;
	bool m_tryingToCross;
	bool m_reached_goal;
	int m_crossingState;
	tVector m_forceAvoidPerson;
	Chumanoid *m_closerHum;
	tVector m_lastUP;	// This is very usefull to mantain consistency when calculating tangential forces



// Personality properties:
	int m_letOthersFirst;
	int m_maxLetOthersFirst;	// Max num of timesteps that will wait for others to walk first
	
	bool male;		// Whether it's female or male
	int m_stopShaking;
	
	int m_bottleNeck;  // Last door where i've seen a bottleNeck
	

// For OCEAN model:
	
	int m_waiting_timer_curr;	// Waiting timer used during the simulation, when we start it it will be = m_waiting_timer
	





public:
	// Cal3d rendering:
//	cal3d_agent *m_cal3dAgent;
	Model*	m_pcal3dAgent;
	bool mDrawCal3d;

	// VALIDATION 
	float m_densityR1;
	bool m_doorCrossedFR;


//Selecting
public:
	bool	mSelected;
	
//Groups
private:
	
	
	
	

	bool	mWaitingRules;

	
public:
	
	
	float* getColorHairOriginal() {return m_colorHairOriginal;}
	

	
	void	setWaitingRules(bool val){mWaitingRules = val;}
	bool	getWaitingRules(){return mWaitingRules;}



	

//Personality
public:
	bool	mShowColorHairPersonality;
	bool	mCommunicate;
	void	updateBehavior();
	void	updateBehaviorColorHair(int ind);
	void	updateTasks();
	
	
};
 
#endif
