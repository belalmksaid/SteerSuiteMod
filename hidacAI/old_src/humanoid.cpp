//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

//****************************************************************************//
// humanoid.cpp                                                        //
// Copyright (C) 2007 Nuria Pelechano                                        //
//****************************************************************************//
// This code is covered by a disclosure agreement filed by the 
// University of Pennsylvania.  Do not distribute with out permission of
// the author.  
//****************************************************************************// 

#include "humanoid.h"
#include <algorithm>	   // STL algorithms class library
#include "cell.h"
#include "time.h"
#include "globalVbles.h"
#include <math.h>
// #include "cal3d/model.h"
#include "tga.h"
#include <iostream>
using namespace std;
// #include <FL/glut.h>
// #include "ogl.h"
// #include "gui.h"


extern guiController* myControl;

int frameRate = 0;
long m_timeFR = 0;
extern float USER_velocity;


float colorTrousersStruct[4][4] = {{0.01,0.01,0.01,1.0},
                                   {0.02,0.02,0.02,1.0},
								   {0.0,0.0,0.2,1.0},
								   {0.2,0.1,0.1,1.0}};

float colorHair[9][4] = {{0.05,0.00,0.00, 1.0},
                         {0.5,0.5,0.5, 1.0},
                         {0.6,0.2,0.00, 1.0},
                         {0.15,0.00,0.00, 1.0},
                         {0.32,0.21,0.00, 1.0},
                         {0.42,0.21,0.00, 1.0},
						 {0.30,0.14,0.00, 1.0},
                         {0.58,0.55,0.01, 1.0},
						 {0.60,0.00,0.00, 1.0}};

Chumanoid::Chumanoid()
{
	m_knowledge = 1;
	m_myCell = 1;
	m_prevCell = 1;
	timeDFS = 0;
	m_visitedCells.clear();
	num_step = 0;
	m_personality.leadership = 0;
	m_personality.trained = false;
	m_speed = 0;
	m_dt = dif_millSecs;
	m_maxSpeed = 0.00124; // 1m/s = 0.001 m/ms
	m_maxSpeed = 0.001; //1 m/s // 0.0001 // slow 1 m/s
	m_time = 0;
	m_densityAhead = 0;
	m_lookAheadA.x = 0.0;
	m_lookAheadA.y = 0.0;
	m_lookAheadA.z = 0.0;
	m_orientation.x = 0.0;
	m_orientation.y = 0.0;
	m_orientation.z = 0.0;
	m_desiredVel.x = 0.0;
	m_desiredVel.y = 0.0;
	m_desiredVel.z = 0.0;
	m_lookAheadB = m_lookAheadA;
	m_vecA = m_vecB = m_lookAheadA;
	m_step_legs = rand() % 4;	
//	m_pcell = m_building->getCell(m_myCell);
//	m_pprevCell = m_building->getCell(m_prevCell);
	m_reached_goal = false;
	m_letOthersFirst = 0;
	m_stopShaking = 0;
	if ((rand()%100) > 35)
		male = true;
	else male = false;
	

	m_crossingState = NOT_CROSSING;
	m_portalIDcoll = -1;
	m_bottleNeck = -2;
	m_lastUP.x = 0;
	m_lastUP.y = 1;
	m_lastUP.z = 0;
	m_doTask = false;
	m_execute_idle = false;
	m_nextTask = false;
	m_doorCrossedFR = false;	
	m_prevAttractor = tVector();
	// Cal3d models:
	//m_cal3dAgent = new cal3d_agent;
	#ifdef CAL3D_RENDERING
		m_pcal3dAgent = new Model;
	#else 
		m_pcal3dAgent = NULL;
	#endif

	//initCal3d();

	m_densityR1 = 0;
	mDrawCal3d = false;

	setWaitingRules(false);//don't wait for other agents initially

//PERSONALITY TRAITS
	//initialize all 5 factors with 0 mean and 0 std
	for(int i=0; i<5; i++)
		mTrait.SetFFM(i,0,0);
	


	

	mCommunicate = true;

	mSelected = false;

	mShowColorHairPersonality = false;
	
	mTrait.SetRightPreference(5); //initially all agents prefer right if not explicitly modified


	m_waiting_timer_curr=0;

	mTrait.SetTaskCount(5);  //normally, each agent performs 5 tasks

	m_personality.leadership = 0;

	

	
}


Chumanoid::Chumanoid(bool know, int currCell, int prevCell, sLeadingBehavior pers)
{
//	m_blocked = listBl;
//	m_blockedTable.clear(); // OJO con esto!!!! lo he puesto temporal!!!!!!!!!!!!
	m_knowledge = know;
	m_myCell = currCell;
	m_prevCell = prevCell;
	m_pcell = m_building->getCell(m_myCell);
	m_pprevCell = m_building->getCell(m_prevCell);
	m_pLastCell = m_pprevCell;
	
//	m_numLastCells = 0;
	timeDFS = 0;
	m_visitedCells.clear();
	num_step = 0;
	m_personality = pers;
	m_lookAheadA.x = 0.0;
	m_lookAheadA.y = 0.0;
	m_lookAheadA.z = 0.0;
	m_lookAheadB.x = 0.0;
	m_lookAheadB.y = 0.0;
	m_lookAheadB.z = 0.0;

	if ((rand()%100) > 35)
		male = true;
	else male = false;

	mTrait.SetPanicLevel(0);
}


//////////////////////////////////////////////////////////
//FFM PERSONALITY MODEL
//////////////////////////////////////////////////////////
//update agent's behavior according to its personality 
void Chumanoid::updateBehavior()
{
	float open, conscient, extro, agree, neuro;
	float w_el,w_al,w_cl, w_nl; //leadership weights for traits
	float w_ei,w_ai,w_ci; //impatience weights for traits
	float w_er,w_ar,w_cr; //right preference weights for traits
	float w_np,w_cp; //panic weights for traits
	float f_a, f_c, f_e;
	float empathy;
	
	open = mTrait.GetFFM(OPEN);
	conscient = mTrait.GetFFM(CONSCIENT);
	extro = mTrait.GetFFM(EXTRO);
	agree = mTrait.GetFFM(AGREE);
	neuro = mTrait.GetFFM(NEURO);

//COMUPUTE EMPATHY FOR EMOTIONAL CONTAGION
	//use normalized factors (in [0 1])

	
	empathy = (0.05*open + 0.5)*0.34 +(0.05*conscient + 0.5) *0.17 + (0.05*extro + 0.5)*0.13 + 
		       (0.05*agree + 0.5)*0.33 + (0.05*neuro + 0.5)*0.03;
	mTrait.SetEmpathy(empathy);

//LEADERSHIP [0 100]
	w_el = w_nl = 0.45;  
	w_cl =  w_al = 0.05; 
	f_a = agree < 0 ? -10 * agree : 0; 
	f_c = conscient > 0 ? 10 * conscient: 0; 
	m_personality.leadership = w_el *(5 * extro + 50) + w_al * f_a + w_cl * f_c + w_nl * (-5 * neuro + 50);


//TRAINED	[true false]
	int val = (rand()%5+1)/5.0;

	if(0.05*open+0.5 >= val)
		m_personality.trained = true;
	else
		m_personality.trained = false;

//COMMUNICATION [true false]
	if(extro < 0)
		mCommunicate = false;
	else
		mCommunicate = true;
//PANIC
	if(myControl->glWin->alarm)
	{
		w_np = 0.9;
		w_cp = 0.1;
		mTrait.SetPanicLevel(w_np *(0.5* neuro + 5) + w_cp * (10 - conscient));
		
	}

//IMPATIENCE [0 10]
	w_ai = 0.4;
	w_ei = 0.3;
	w_ci = 0.3;
	f_e = extro >= 0 ? 10* extro : 0;
	mTrait.SetImpatience(w_ei * f_e + w_ai * (-0.5 * agree + 5) + w_ci * (-0.5 * conscient + 5));

////IMBALANCE [0 6]
//	int prob = rand()%10;
//	if(prob<=7 || conscient >=0)
//		mTrait.SetImbalance(0);
//	else
//		mTrait.SetImbalance(-0.6* conscient);


//
////PUSHING [true false]
//	if(extro > 5 || agree < 0)
//		mPushing = true;
//	else
//		mPushing = false;



//RIGHT PREFERENCE
	w_ar = w_cr = 0.5;
	if(agree < 0 || conscient < 0)
		mTrait.SetRightPreference(0.5); //50% chance
	else
		mTrait.SetRightPreference(w_ar * (0.05 * agree + 0.5) + w_cr * (0.05 * conscient + 0.5)); //50% chance

//PERSONAL SPACE [MIN MED MAX]

	if(extro < (-10.0 / 3.0))		//[-10 -10/3)
		mTrait.SetPersonalSpace(MAX);
	else if(extro <= (10.0 / 3.0))	// [-10/3 10/3]
		mTrait.SetPersonalSpace(MED);
	else	
		mTrait.SetPersonalSpace(MIN);

	

//WAITING RADIUS [MIN MED MAX]
	//shows how kind a person is
	if(agree < (-10.0 / 3.0))		//[-10 -10/3)
		mTrait.SetWaitingRadius(MIN);
	else if(agree <= (10.0 / 3.0))	// [-10/3 10/3]
		mTrait.SetWaitingRadius(MED);
	else							// (10/3 10]
		mTrait.SetWaitingRadius(MAX);

////WAITING TIMER 
//
//	if(agree < (-10.0 / 3.0))		//[-10 -10/3)
//		mTrait.SetWaitingTimer(MIN);
//	else if(agree <= (10.0 / 3.0))	// [-10/3 10/3]
//		mTrait.SetWaitingTimer(MED);
//	else							// (10/3 10]
//		mTrait.SetWaitingTimer(MAX);

//EXPLORING [0 10]
	
	mTrait.SetTaskCount(open/2.0 + 5);
	updateTasks();

//SPEED [0.001 0.002]

	//setSpeed(3*extro/40.0+50.0/40.0); 
	setSpeed((extro + 10)/20000.0 + 0.001); 

}


void Chumanoid::updateBehaviorColorHair(int ind)
{
	float trait;
	
	trait = mTrait.GetFFM(ind);
/* //funda:: simdilik kapadim contagioni gormek icin
	if(trait > 0) //if trait is positive, hair color is red
		mTrait.SetHairColorPersonality(trait/10.0,0.0,0.0,1.0);
	else //else hair color goes to blue
		mTrait.SetHairColorPersonality(0,0,-trait/10.0,1.0);
*/

}
//////////////////////////////////////////////////////////
void Chumanoid::setSpeed(double speed)
{ 
	m_speed = 0;
	m_maxSpeed = speed;
};

void Chumanoid::increaseSpeed()
{
	m_maxSpeed += 0.00174+0.0001 * pow(-1.0,(rand()%2)+1);
}

void Chumanoid::updateTasks()
{

	int mod;
	TASKS task;
	task.proximity = 0.8;  //1.15; 
	//   if (mod==0)
	//{	
	//	task.location.x = 10; // + m_humID;
	//	task.location.y = 0;
	//	task.location.z = 8.5; // + m_humID;
	//}
	//else if (mod==1)
	//{	
	//	task.location.x = 15; // + m_humID;
	//	task.location.y = 0;
	//	task.location.z = 8.5; // + m_humID;
	//}
	//else
	//{	
	//	task.location.x = 20; // + m_humID;
	//	task.location.y = 0;
	//	task.location.z = 8.5; // + m_humID;
	//}
	/*task.roomID = 2;
	task.location.x = 10;
	task.location.y = 0;
	task.location.z = 23;
	task.orientation.y = 90;
	m_tasks.push_front(task);*/

//funda	for (int i = 0; i < TASK_COUNT; i++)  //funda

	removeTasks(); //to assign new tasks

	for (int i = 0; i < mTrait.GetTaskCount(); i++)  //funda: each agent gets tasks according to their personality
	{
	
		int numActions = rand()%3; //max of 5 actions per task
		for (int j = 0; j <numActions; j++)
		{
			int actionNum = rand()%8;  //there are 9 idle actions and start at 3
			task.laction.push_back(actionNum+3);
		}
		
		//task.laction.push_back(3);  //Combine actions
		//task.laction.push_back(4);
		//task.laction.push_back(5);
		//task.laction.push_back(6);  
		//task.laction.push_back(7);
		//task.laction.push_back(8);
		//task.laction.push_back(9);
		//task.laction.push_back(10);
		//task.laction.push_back(11);
	//	task.duration = 0;  //2800; //8000; //00; 5600;  calculated elsewhere
	
//funda: all agents move to the same place for the time being
		//for seeing the formation of rings
		//if(getTaskCount()==1)
		//	mod = 0;
		//else
	//mod = rand()%7; // 7 tables
		mod = rand()%8; // 9 paintings

		if(mTrait.GetFriendship())
			mod = mTrait.GetGroupId(); //tasks are assigned according to their group ids

////To test two groups crossing each other
//		if(mTrait.GetGroupId()%2==0)
//			mod = 0;
//		else
//			mod = 1;
//
//
//		if (mod==0){ //show cabaret_table positions
//		task.location.x = 8;
//		task.location.y = 0;
//		task.location.z = 5;
//		task.orientation.y = 0;
//		task.roomID = 1;
//		m_tasks.push_front(task);
//	}
//	else if (mod==1){
//		task.location.x = 20;
//		task.location.y = 0;
//		task.location.z = 5;
//		task.orientation.y = 0;
//		task.roomID = 1;
// 		m_tasks.push_front(task);
//	}


//For openness 
	if (mod==0)
	{ 
		task.location.x = 5;
		task.location.y = 0;
		task.location.z = 3;
		task.orientation.y = 0;
		task.roomID = 1;
		m_tasks.push_front(task);
	}
	else if (mod==1)
	{
		task.location.x = 20;
		task.location.y = 0;
		task.location.z = 3;
		task.orientation.y = 0;
		task.roomID = 1;
 		m_tasks.push_front(task);
	}
	else if (mod==2)
	{
		task.location.x = 15;
		task.location.y = 0;
		task.location.z = 3	;
		task.orientation.y = 0;
		task.roomID = 1;
 		m_tasks.push_front(task);
	}
	else if (mod==3) 
	{
		task.location.x = 10;
		task.location.y = 0;
		task.location.z = 3;
		task.orientation.y = 0;
		task.roomID = 1;
 		m_tasks.push_front(task);
	}
	else if (mod==4) 
	{
		task.location.x = 3;
		task.location.y = 0;
		task.location.z = 5;
		task.orientation.y = 0;
		task.roomID = 1;
 		m_tasks.push_front(task);
	}
	else if (mod==5) 
	{
		task.location.x = 3;
		task.location.y = 0;
		task.location.z = 10;
		task.orientation.y = 0;
		task.roomID = 1;
 		m_tasks.push_front(task);
	}
	else if (mod==6) 
	{
		task.location.x = 20;
		task.location.y = 0;
		task.location.z = 5;
		task.orientation.y = 0;
		task.roomID = 1;
 		m_tasks.push_front(task);
	}
	else if (mod==7) 
	{
		task.location.x = 20;
		task.location.y = 0;
		task.location.z = 10;
		task.orientation.y = 0;
		task.roomID = 1;
 		m_tasks.push_front(task);
	}




	//	
	//if (mod==0){ //show cabaret_table positions
	//	task.location.x = 8;
	//	task.location.y = 0;
	//	task.location.z = 5;
	//	task.orientation.y = 0;
	//	task.roomID = 1;
	//	m_tasks.push_front(task);
	//}
	//else if (mod==1){
	//	task.location.x = 13;
	//	task.location.y = 0;
	//	task.location.z = 5;
	//	task.orientation.y = 0;
	//	task.roomID = 1;
 //		m_tasks.push_front(task);
	//}
	//else if (mod==2){
	//	task.location.x = 18;
	//	task.location.y = 0;
	//	task.location.z = 5;
	//	task.orientation.y = 0;
	//	task.roomID = 1;
 //		m_tasks.push_front(task);
	//}
	//else if (mod==3) {
	//	task.location.x = 8;
	//	task.location.y = 0;
	//	task.location.z = 8;
	//	task.orientation.y = 0;
	//	task.roomID = 1;
 //		m_tasks.push_front(task);
	//}
	//else if (mod==4) {
	//	task.location.x = 13;
	//	task.location.y = 0;
	//	task.location.z = 8;
	//	task.orientation.y = 0;
	//	task.roomID = 1;
 //		m_tasks.push_front(task);
	//}
	//else if (mod==5) {
	//	task.location.x = 18;
	//	task.location.y = 0;
	//	task.location.z = 8;
	//	task.orientation.y = 0;
	//	task.roomID = 1;
 //		m_tasks.push_front(task);
	//}
	//else {
	//	task.location.x = 4;
	//	task.location.y = 0;
	//	task.location.z = 6.5;
	//	task.orientation.y = 0;
	//	task.roomID = 1;
 //		m_tasks.push_front(task);
	//}
	}
}

void Chumanoid::updateHumanoid(int ID, bool know, int currCell, int prevCell,float color[3], sLeadingBehavior pers,int panic)
{
	m_humID = ID;
//	m_blocked = listBl;
	m_knowledge = know;
	m_myCell = currCell;
	m_prevCell = prevCell;
	int NumTrousers = rand()%4;
	int NumHair = rand()%9;

	NumHair = rand()%8;

	
//	if (( currCell==1) && (m_humID < 30))
//		NumHair = 8;

//	if (currCell == 1)
//		NumHair = 0;
//	else
//		NumHair = 7;
	// Red heads panic
 	if (NumHair == 8) 
		mTrait.SetPanicLevel(10);
	// blondes are impatient
	//if (NumHair == 7) 
	//	mTrait.SetImpatience(10);
		
//	if (rand()%3==0)
		mTrait.SetImpatience(10);

//	m_panic = panic;
//	if (panic > 5)
//		NumHair = 8;

		//if groups are not specifically set for observation, assign random shirt colors
	if(!mTrait.GetFriendship())
	{
		for (int j=0; j<3; j++)	
			color[j] = rand() / (float) RAND_MAX;
	}

	for (int i=0; i<3; i++)
	{
		m_color[i] = color[i];
		m_colorTrousers[i] = colorTrousersStruct[NumTrousers][i];
		m_colorHairOriginal[i] = colorHair[NumHair][i];
	}
	
	//Jan mellowing colors
	m_colorLegs[0] = 0.78;
	m_colorLegs[1] = 0.50;
	m_colorLegs[2] = 0.18;
	m_colorLegs[3] = 1.0;

	m_color[3] = 1.0;
	m_colorTrousers[3] = 1.0;
	m_colorHairOriginal[3] = 1.0;
	
	
	//m_colorLegs[0] = 0.98;
	//m_colorLegs[1] = 0.70;
	//m_colorLegs[2] = 0.38;
	//m_colorLegs[3] = 1.0;

	//m_color[3] = 1.0;
	//m_colorTrousers[3] = 1.0;
	//m_colorHairOriginal[3] = 1.0;

	timeDFS = 0;
	map<int,sPORTAL> portals;
	map<int,sPORTAL>::iterator iter;
//	float size = 8.0;

	//srand(time(NULL));
	m_incrx = (float)(rand() % (int)SIZE_CELL*10) / 12.0 + 0.5;
	m_incrz = (float)(rand() % (int)SIZE_CELL*10) / 12.0 + 0.5;
	

	
	// If m_myCell is not an exit of the building, then i'll start creating my DFS:
//	CCell *cell = m_maze->getCell(m_myCell);
//	if (m_maze->cellInExit(cell))

	createDFSnode(NO_PARENT);	// -1 means there's no parent
	
	num_step = 0;

	m_personality = pers;

	// For debugg purposes:
	if (m_personality.leadership > 80)
		m_pointSize = 6;
	else if (m_personality.leadership > 20)
		m_pointSize = 5;
	else
		m_pointSize = 4;
	//// End debug leadership


	m_pcell = m_building->getCell(m_myCell);
	m_pprevCell = m_building->getCell(m_prevCell);
	m_pLastCell = m_pprevCell;

	

	mPos = m_building->getRandomPosInCell(m_myCell);

	//// To test two groups crossing each other
	//if(mTrait.GetGroupId()% 2 == 0)
	//{
	//	mPos.x = 16;
	//	mPos.y = 0;
	//	mPos.z = 5;
	//}
	//else
	//{
	//	mPos.x = 10;
	//	mPos.y = 0;
	//	mPos.z = 5;
	//}


	////To test people in a museum

	//mPos.x = 16;
	//mPos.y = 0;
	//mPos.z = 9;

	//To test friend groups
	if(mTrait.GetFriendship())
	{		
		mPos.x = (mTrait.GetGroupId()+1)*2 % 20 + 2;
		mPos.y = 0;
		mPos.z = (mTrait.GetGroupId()+1)*3 % 10 + 2;
	}

	m_prevpos = mPos;
	mInitialPosition = mPos;

	// Make m_listPrevPos of length 3:
	for (int n=0; n<3; n++)
		m_listPrevPos.push_front(mPos);

	// I THE FIRST PORTAL IN MY HASH TABLE!!!
	//m_attractor.x = (*(m_pcell->getTablePortals().begin())).second.pos[0];
	//m_attractor.y = (*(m_pcell->getTablePortals().begin())).second.pos[1];
	//m_attractor.z = (*(m_pcell->getTablePortals().begin())).second.pos[2];

	m_attractor = mPos;

	// I need to calculate the next cell to walk to...
	chooseNextCell();

	// Parameters: Resolution, bodyWidth, bodyHeight, headHeight, legsHeight, legsWidth
	//updateHumanoidRendering(6,0.7,1.5,0.4,1.5,0.2);
	updateHumanoidRendering(6,0.5,0.7,0.3,0.8,0.1);

	m_doorCrossed = false;	// I set it to true so that the next cell is calculated at the very beginning!
	m_crossingPortal = false;

	#ifdef REAL_TIME
		_timeb tm;
		_ftime(&tm);
		m_time = tm.time*1000 + tm.millitm; 
	#endif	

	
	
	updateTasks();
	

	////funda
updateBehavior();
	
}



// when using hash tables:
void Chumanoid::addBlockedCell(int newBlockedCell)
{
	m_blockedTable[newBlockedCell] = 1;
}

int Chumanoid::currentCell()
{	
	return m_myCell;
}


int Chumanoid::previousCell()
{
	return m_prevCell;
}

// This function is called when there is some major change in the environment that requires to recompute the next cell due to 
// appearance of fire in m_myCell, or the door between m_prevCell and m_myCell being just closed
bool Chumanoid::recalculateNextCell()
{
//	int portalID;
	sPORTAL portal;
	PORTAL_INFO portalInfo;
	
//	portal = m_pprevCell->getPortal(m_nextPortal);	
	portalInfo = m_building->getPortalInfo(m_nextPortal);

	// If the door is closed of there's fire in the cell towards which i'm walking, then i need to change the next cell
	// in my way-finding
	if ((!portalInfo.open) || (m_pcell->getHazard()->getType() == FIRE))
	{
		m_listClosedPortals[portal.id] = 1;
		m_myCell    = m_prevCell;
		m_pcell     = m_pprevCell;
		m_prevCell  = m_pLastCell->getName();
		m_pprevCell =  m_pLastCell;
		return chooseNextCell();
	}
	return true;
}

bool Chumanoid::lookForAlternativeDoor()
{
	map<int,sPORTAL> tablePortals;
	map<int,sPORTAL>::iterator iter_portal;
	double *pos;

/*
	m_myCell    = m_prevCell;
	m_pcell     = m_pprevCell;
	m_prevCell  = m_pLastCell->getName();
	m_pprevCell =  m_pLastCell;
	return chooseNextCell();
*/

	// So far we just check whether there's another door within this room to get to the next room
	tablePortals = m_pprevCell->getTablePortals();
	for (iter_portal = tablePortals.begin(); iter_portal != tablePortals.end(); iter_portal++)
	{
		if ((*iter_portal).first != m_nextPortal)
		{
			if ((*iter_portal).second.nextcellID == m_myCell)
			{
				// there's another portal leading to the same room, and therefore we can just change the m_attractor
				m_building->removeHum(m_nextPortal,this);
				m_nextPortal = (*iter_portal).first;
				pos = m_pprevCell->getPortal(m_nextPortal).pos;
				m_attractor.x = pos[0];
				m_attractor.y = pos[1];
				m_attractor.z = pos[2];
				m_crossingState = NOT_CROSSING;
				return true;
			}
		}
	}
	return false;
}

// Choose the next cell to go
// Returns false when it cannot choose any cell because you are either stack or in the exit
bool Chumanoid::chooseNextCell()
{
	//CCell *cell;
	//CCell *prevCell;
	int numExits, closestExit, closestExitDist = 100000;
	float dist, distMin=1000000;
	int aux, parent;
	bool path_found;
//	float pos[3];
//	int n_extraPaths;
	list<path2exit> extraPaths;
	list<path2exit>::iterator  iter_path;
	int next_portal;
//	tVector distPortal;
	sPORTAL portal;

	updateBlockedList();
	updateClosedPortals();

	// Basic case: see if the next cell in the sortest path it's in a non blocked path
	//cell = m_building->getCell(m_myCell);
	numExits = m_building->getNumExits();

	for (int i=0; i<numExits; i++)
	{
		if (m_pcell->getPath(i).n != 10000 )
		{
			portal = m_pcell->getPortal(m_pcell->getPath(i).path.front().idNextPortal);
			dist = m_pcell->getPath(i).length + LENGTH((mPos.x-portal.pos[0]),(mPos.z-portal.pos[2]));
						
			//if (m_pcell->getPath(i).n < closestExitDist)
			if ((dist < distMin)) 
			{
				if ((!blocked(m_pcell->getPath(i).path)) && ( portal.id != m_bottleNeck))
				{
					// make sure that the door is opened
					//if (m_pcell->getPortal(m_pcell->getPath(i).path.front().idNextPortal).open)
					if (!doorsClosed(m_pcell->getPath(i).path))
					{
						distMin = dist;
						closestExitDist = m_pcell->getPath(i).n; 
						closestExit = i;
					}
				}
			}
		}
	}


	// Once i know which path i'm going to follow, check in the path of the cell which will be my next position
	
	//prevCell = m_building->getCell(m_prevCell);
	parent = m_pprevCell->getName(); 

	if (closestExitDist != 100000) //tendremos que buscar otro algoritmo para salir del maze
	{	
		//m_pointSize = 4;
		if (closestExitDist == 0)
		{
			// do nothing so far
			// Increase the number of humanoids that managed to evacuate the building
			if (!myControl->glWin->lhumsEvac.empty())
				myControl->glWin->lhumsEvac.remove(m_humID);
			myControl->glWin->lhumsEvac.push_front(m_humID);

			m_building->removeHum(m_nextPortal,this);

			exitCell(); // delete info from current m_myCell
			m_prevCell = m_myCell;
			m_pcell = m_building->getCell(m_myCell);
			m_pLastCell = m_pprevCell;
			m_pprevCell = m_pcell;
			next_portal = m_pcell->getPath(closestExit).path.front().idNextPortal;
			enterCell();

			tVector prev_attractor = m_attractor; // I DO THIS TEMPORALY, I'LL PROBABLY DELETE THIS HUMANS IN THE FUTURE....
//			next_portal = m_building->findPortal(m_myCell, -1); // -1 means EXIT
			// Set the next attractor point which is in the portal:
			/*
			m_attractor.x = m_pcell->getPortal(next_portal).pos[0];
			m_attractor.y = m_pcell->getPortal(next_portal).pos[1];
			m_attractor.z = m_pcell->getPortal(next_portal).pos[2];
			*/
			m_attractor.x = m_pprevCell->getPortal(next_portal).pos[0];
			m_attractor.y = m_pprevCell->getPortal(next_portal).pos[1];
			m_attractor.z = m_pprevCell->getPortal(next_portal).pos[2];

			
			m_nextPortal = next_portal;

			if ((m_attractor.x == prev_attractor.x) && (m_attractor.z == prev_attractor.z))
			{
				m_prevpos = mPos;
				// I want to remove them from the environment!
				m_reached_goal = true;
				exitCell();
			}

			/*
			if ((myControl->glWin->lhumsEvac.size() == NUM_HUMS) && !(finished))
			{
				//cout << "TIME = " << globalTimeSteps;
	//			reset = true;
	//			finished = true;
			}
			*/

			return false;
		}
		else
		{

			exitCell(); // delete info from current m_myCell
			m_prevCell = m_myCell; // CAMBIADO DE SITIO, ESTABA ARRIBA!!!
			m_myCell = m_pcell->getPath(closestExit).path.front().idCell;
			next_portal = m_pcell->getPath(closestExit).path.front().idNextPortal;
			enterCell(); // add info to new cell
		}
		
		if (searchType == DFS)
		{
			// We will always create an entry in our DFStree, so the root of the DFStree should be the first node where the
			// agent is located.
			createDFSnode(parent);
			resetTimeDFS();
		}
	}
	else // If the paths known are blocked then I need to find decide where to go
	{
		path_found = false;

		if ( TRAINED && (m_personality.trained))
		{
			extraPaths = m_pcell->getExtraPaths();
			iter_path = extraPaths.begin();
			while (!path_found && (iter_path != extraPaths.end()))
			{
				if (!blocked((*iter_path).path))
				{
					// make sure that the door is not closed
				//	if (m_pcell->getPortal(((*iter_path).path).front().idNextPortal).open)
					if (!doorsClosed((*iter_path).path)) 
					{
						path_found = true;

						exitCell(); // delete info from current m_myCell
						m_prevCell = m_myCell; // CAMBIADO DE SITIO, ESTABA ARRIBA!!!
						m_myCell = ((*iter_path).path).front().idCell; // cell->getPath(closestExit).path.front();
						next_portal = ((*iter_path).path).front().idNextPortal;
						enterCell(); // add info to new cell
					
						if (searchType == DFS)
						{
							// We will always create an entry in our DFStree, so the root of the DFStree should be the first node where the
							// agent is located.
							createDFSnode(parent);
							resetTimeDFS();
						}
					}
				}
				iter_path++;
			}
		}

		// OJO PARA RANDOM NO ESTOY MIRANDO SI LAS PUERTAS ESTAN CERRADAS O NO
		if ( !path_found && (searchType == RANDOM))
		{
			//m_pointSize = 2;
			exitCell(); // delete info from the current m_myCell cell

			// if there is only one door in the cell, then take that door:
			if (m_pcell->getNumDoors() == 1)
			{
				aux = m_myCell;
				m_myCell = m_prevCell;
				m_prevCell = aux;
				enterCell(); // add info to new cell
			}

			// This part searched for a random door, if it returns false it means it hasn't found a door
			// in this step, so it will stay without moving for this one step of the simulation

			//else if (findRandomCell())
			{
			//	exitCell(); // delete info from previous cell
			//	enterCell(); // add info to new cell
			}
			
		}
		else if (!path_found && (searchType == DFS))
		// New exploration alg based on DFS
		{
			//m_pointSize = 6;
			findDFScell();
			next_portal = m_building->findPortal(m_myCell, m_prevCell);
		}
	}

	// save myself from getting stuck:
	if (m_myCell == m_prevCell)
	{
		// I need this when i'm not forgeting cells!
		forgetFINISHEDcells();	
		
		return false;
		
	}


	m_pcell = m_building->getCell(m_myCell);
	m_pLastCell = m_pprevCell;
	m_pprevCell = m_building->getCell(m_prevCell);
	
	// Set the next attractor point which is in the portal:
	/*
	m_attractor.x = m_pcell->getPortal(next_portal).pos[0];
	m_attractor.y = m_pcell->getPortal(next_portal).pos[1];
	m_attractor.z = m_pcell->getPortal(next_portal).pos[2];
	*/
	
	m_attractor.x = m_pprevCell->getPortal(next_portal).pos[0];
	m_attractor.y = m_pprevCell->getPortal(next_portal).pos[1];
	m_attractor.z = m_pprevCell->getPortal(next_portal).pos[2];			

	m_nextPortal = next_portal;

	// if the next attractor is too close, then update the lists of portals to which i belong:
	if (length(m_attractor-mPos) < 0.2)
	{
		m_crossingState = NOT_CROSSING;
		m_building->removeHum(m_nextPortal,this);
	}

	return true;
}


bool Chumanoid::doorsClosed(list<pairIDs> path)
{
	// Check wether one of the doors i need to cross in my path is closed. I can only check the doors within the rooms where i've 
	// already been. That information is in my memory (DFS)
//		int val;
	// I need to check whether any of the cells in the exit path is in my hash table of closed doors
	// with a value higher than 0

	list<pairIDs>::iterator path_iter;
	map<int,int>::iterator map_iter;
	

	if (!m_listClosedPortals.empty())
	{
		for (path_iter = path.begin(); path_iter != path.end(); path_iter++)
		{
			map_iter = m_listClosedPortals.find((*path_iter).idNextPortal);
			if (map_iter != m_listClosedPortals.end())
			{
				return 1;
				/*
				val = m_listClosedPortals[(*path_iter).idNextPortal];
				if (val > 0)
				{
					return 1; // as soon as one of the cells in the path is bloked, the whole path is discarded
				}
				*/
			}
			
		}
	}
	return 0;

}

/*
bool Chumanoid::findRandomCell()
{
	int wall;
	int possExit = -1;
	CCell *cell;
	CCell *newPcell;
	coord newCell;


	cell = m_maze->getCell(m_myCell);
	// In each cell there can be 4 possible doors
	wall = rand()%4; // I'll consider 0=N, 1=S, 2=E, 3=W


	switch (wall)
	{
	case 0: // 'N'
		if ((!cell->getWall('N')) && m_myCell.row > 0)
		{
			if (m_prevCell.row >= m_myCell.row)
			{
				
				//newPcell = m_maze[(m_myCell.row - 1)* + m_myCell.col]
				newCell.row = m_myCell.row - 1;
				newCell.col = m_myCell.col;
				newPcell = m_maze->getCell(newCell);
				if (newPcell->getHazard()->getType() == 0)
				{
					exitCell(); // delete info from the current m_myCell cell
					m_prevCell = m_myCell;
					newCell = newPcell->getName();
					m_myCell = newCell;
					enterCell(); // add info to new cell
					m_pcell = m_maze->getCell(m_myCell);
					m_pprevCell = m_maze->getCell(m_prevCell);
					return true;
				}
			}

		}
		break;

	case 1: // 'S'
		if ((!cell->getWall('S')) && (m_myCell.row < m_maze->getRows()-1)) 
		{
			if (m_prevCell.row <= m_myCell.row)
			{
				newCell.row = m_myCell.row + 1;
				newCell.col = m_myCell.col;
				newPcell = m_maze->getCell(newCell);
				if (newPcell->getHazard()->getType() == 0)
				{
					exitCell(); // delete info from the current m_myCell cell
					m_prevCell = m_myCell;
					newCell = newPcell->getName();
					m_myCell = newCell;
					enterCell(); // add info to new cell
					m_pcell = m_maze->getCell(m_myCell);
					m_pprevCell = m_maze->getCell(m_prevCell);
					return true;
				}
			}

		}
		break;
	case 2: // 'E'
		if ((!cell->getWall('E')) && (m_myCell.col < m_maze->getCols()-1)) 
		{
			if (m_prevCell.col <= m_myCell.col)
			{
				newCell.row = m_myCell.row;
				newCell.col = m_myCell.col + 1;
				newPcell = m_maze->getCell(newCell);
				if (newPcell->getHazard()->getType() == 0)
				{
					exitCell(); // delete info from the current m_myCell cell
					m_prevCell = m_myCell;
					newCell = newPcell->getName();
					m_myCell = newCell;
					enterCell(); // add info to new cell
					m_pcell = m_maze->getCell(m_myCell);
					m_pprevCell = m_maze->getCell(m_prevCell);
					return true;
				}
			}

		}
		break;
	case 3: // 'W'
		if ((!cell->getWall('W')) && (m_myCell.col > 0))
		{
			if (m_prevCell.col >= m_myCell.col)
			{
				newCell.row = m_myCell.row;
				newCell.col = m_myCell.col - 1;
				newPcell = m_maze->getCell(newCell);
				if (newPcell->getHazard()->getType() == 0)
				{
					exitCell(); // delete info from the current m_myCell cell
					m_prevCell = m_myCell;
					newCell = newPcell->getName();
					m_myCell = newCell;
					enterCell(); // add info to new cell
					m_pcell = m_maze->getCell(m_myCell);
					m_pprevCell = m_maze->getCell(m_prevCell);
					return true;
				}
			}
		}
		break;
	
	}

	return false;

}
*/

void Chumanoid::setEnvironment(Cbuilding *building)
{
	m_building = building;
}



/* OLD SIMULATION tVector!!!!!!!!!!!!1 THE GOOD ONE NOW IS THE NEXT FUNCTION!!!!!!!!!!!!!!
void Chumanoid::runSimulationStep()
{
	//updated blocked cells
	CCell *cell;
	DECISION decision;
	int prob;

	CCell *prevCell; 
	coord parent;

	cell = m_maze->getCell(m_myCell);
	decision = cell->getDecision();
		

	/// DEBUG /////
	int prueba=0;
	if (m_humID == 64)
	{
		prueba++;
	}

	//if ((m_humID == 19) || (m_humID==20))
	//{
	//	map<string, DFSdata>::iterator iter_prueba;
	//	cout<< endl;
	//	cout<< m_humID<<"\t";
	//	for (iter_prueba = m_visitedCells.begin();iter_prueba != m_visitedCells.end(); iter_prueba++)
	//	{
	//		cout << (*iter_prueba).first << "("<< (*iter_prueba).second.state<<") ";
	//	}
	//}	
	/// END DEBUG /////


	// Chose next cell to go
	// We need to find out whether this humanoid is a leader or not:
	if (LEADERSHIP)
	{
		if (decision.decided)
		{
		
			prob = rand()%100;
			if (prob < m_personality.leadership)	// If it's a leader it will decide, otherwise it will follow whoever decide before
			{
				chooseNextCell();
				//cout << "hum: " << m_humID << "chose: " << m_myCell.row<<","<<m_myCell.col<< "NOT" <<endl;
			}
			else

			{
				prevCell = m_maze->getCell(m_prevCell);
				parent = prevCell->getName(); 

				exitCell();
				m_prevCell = m_myCell;
				m_myCell = decision.cell;
				enterCell(); // add info to new cell
				m_pcell = m_maze->getCell(m_myCell);
				m_pprevCell = m_maze->getCell(m_prevCell);
		
				if (searchType == DFS)
				{
					// We will always create an entry in our DFStree, so the root of the DFStree should be the first node where the
					// agent is located.
					createDFSnode(parent.name);
					resetTimeDFS();
				}

				//cout << "hum: " << m_humID << "chose: " << decision.cell.row<<","<<decision.cell.col<< "FOLLOWS" <<endl;
			}
			
		}
		else
		{
			//chooseNextCell();
			decision.decided = chooseNextCell(); //true;
			decision.cell = m_myCell;
			cell->setDecision(decision);
			//cout << "hum: " << m_humID << "chose: " << decision.cell.row<<","<<decision.cell.col<< endl;
		}
	}
	else
	{
		chooseNextCell();
	}
 }
*/


void Chumanoid::runSimulationStep()
{
	//updated blocked cells
	CCell *cell;
	DECISION decision;
	int prob;
	CCell *prevCell; 
//	coord parent;
	int parent;
	tVector nextStep;
//	tVector dist;

	if ((myControl->glWin->levanta) && (m_humID==1))
	{
		mTrait.SetImbalance(0);
	}

	// Check whether my m_nextPortal has been closed or my current one has a better exit...
	if ((m_pcell->hasChanged()) || (m_pprevCell->hasChanged()))
		recalculateNextCell();

	// Check whether too many people are trying to cross it and so we could decide to pick another portal
	if ( myControl->glWin->globalNumSimulations%10 == 0) 
	{
		if (portalBottleNeck())
			lookForAlternativeDoor();
	}

	// Draw with route:
	if (local_motion == LINEAR_INTERP)
	{
		static bool YA=false;
		if ((!YA) && (m_humID == 1) && myControl->glWin->caete)
		{
			mTrait.SetImbalance(6);
			YA=true;
		}
	
		if (mTrait.GetImbalance() < 6) // If it is bigger than 4 it means i fell, so i won't move!!!!
		{
#ifdef TEST_CROWDS
			if (myControl->glWin->crowdModel == HIDAC)
				nextStep = calculateSteps();
			else if (myControl->glWin->crowdModel == SOCIAL_FORCES)
				nextStep = calculateStepsSocialForces();
			else if (myControl->glWin->crowdModel == RULE_BASED)
				nextStep = calculateStepsRuleBased();
			else if (myControl->glWin->crowdModel == CELL_AUTOMATA)
				nextStep = calculateStepsCellularAutomata();
#else
			nextStep = calculateSteps();
#endif
		}
		else
		{
			// I think if we remove it from the list of hums in this cell, we'll avoid doing collision agains him... not sure though...
			if (mTrait.GetImbalance() >= 6)// The first time, we transform that human in a list of obstacles
			{
				exitCell();
				m_building->removeHum(m_nextPortal,this);
				createNewObstacle();
			}
			nextStep = mPos;
		}
	}
//	else 
//		{nextStep = calculateStepsHelbing();}

	//m_route.push_back(nextStep);
	
	// before change above:	
	m_prevpos = mPos;
	mPos = nextStep;

	// VALIDATION! when the agent has just been created, the initial speed could be huge to avoid the overlapping. We don't want that data in the validation files
	// since it's not part of the real simulation.
	float l=length(nextStep-mPos);
	if (l/m_dt > m_speed + 0.0001)
	{
		m_prevpos = mPos;
		cout <<	"ERROR ERROR ERROR";
	}


	//cell = m_building->getCell(m_myCell);
	cell = m_pcell;			// OJO 15apr06
	decision = cell->getDecision();
	
	// If it's a stair we may need to increase or decrease the Y coord:

	sSTAIR stair;
//	float dif;
	int current_step; //, prev_step;

	if (m_pprevCell->getIsStair(&stair))
	{
		//mPos.y = stair.posCenterBe[1];
		// check whether we are moving in the X axis
		if (stair.incrX != 0)
		{			current_step = floor((stair.posCenterAb[0]-mPos.x)/stair.incrX);
			mPos.y = stair.posCenterAb[1] - current_step * stair.incrY;
			if (mPos.y > stair.posCenterAb[1])
				mPos.y = stair.posCenterAb[1];
			if (mPos.y < stair.posCenterBe[1])
				mPos.y = stair.posCenterBe[1];
		}
		// or we are moving in the Z axis
		else
		{
			current_step = floor((stair.posCenterAb[2]-mPos.z)/stair.incrZ);
			mPos.y = stair.posCenterAb[1] - current_step * stair.incrY;
			if (mPos.y > stair.posCenterAb[1])
				mPos.y = stair.posCenterAb[1];
			if (mPos.y < stair.posCenterBe[1])
				mPos.y = stair.posCenterBe[1];

		}

	}
		
	// Chose next cell to go only if the door has been crossed
	// We need to find out whether this humanoid is a leader or not:
	if ((m_doorCrossed) && (!m_reached_goal))
	{
		// When a door is crossed we want to update the information regarding flow rates and densities:
		m_building->addHumCrossed(m_nextPortal);   // OJO!!!

		// Once the information is updated we'll choose the next cell
		m_doorCrossed = false;
		if (LEADERSHIP)
		{
			if (decision.decided)
			{
			
				prob = rand()%100;
				if (prob < m_personality.leadership)	// If it's a leader it will decide, otherwise it will follow whoever decide before
				{
					chooseNextCell();
					//cout << "hum: " << m_humID << "chose: " << m_myCell.row<<","<<m_myCell.col<< "NOT" <<endl;
				}
				else

				{
					prevCell = m_building->getCell(m_prevCell);
					parent = prevCell->getName(); 

//					exitCell();
					m_prevCell = m_myCell;
					m_myCell = decision.cell;
//					enterCell(); // add info to new cell
					m_pcell = m_building->getCell(m_myCell);
					m_pLastCell = m_pprevCell;
					m_pprevCell = m_building->getCell(m_prevCell);
			
					if (searchType == DFS)
					{
						// We will always create an entry in our DFStree, so the root of the DFStree should be the first node where the
						// agent is located.
						createDFSnode(parent);
						resetTimeDFS();
					}

					//cout << "hum: " << m_humID << "chose: " << decision.cell.row<<","<<decision.cell.col<< "FOLLOWS" <<endl;
				}
				
			}
			else
			{
				//chooseNextCell();
				decision.decided = chooseNextCell(); //true;
				decision.cell = m_myCell;
				cell->setDecision(decision);
				//cout << "hum: " << m_humID << "chose: " << decision.cell.row<<","<<decision.cell.col<< endl;
			}
		}
		else
		{
			chooseNextCell();
		}
	}

	if (m_doTask)
	{
		if (m_tasks.front().timer > 0)
		{
			//actionID = 3 + (rand()%3);
			//cout << "actionID = " << actionID;
			m_tasks.front().timer -= m_dt;
			//cout <<" timer decrease= "<< m_tasks.front().timer<<endl;
		}
		else
		{
			m_tasks.front().timer = 0;
			m_doTask= false;
			//cout << "FALSE"<<endl;
			if (!m_tasks.empty())
				m_tasks.pop_front();
		}
	}

	// VALIDATION
	//if (m_densityR1 > DENSITY_LIMIT)
	//{
	//	glPointSize(4);
	//	glBegin(GL_POINTS);
	//	glColor3f(1.0,0.0,0.0);
	//	glVertex3f(mPos.x, mPos.y+0.7, mPos.z);
	//	glEnd();
	//}

	#ifdef DEBUG_DENSITY
	if (m_densityR1 > 0.7)
	{
		glPointSize(4);
		glBegin(GL_POINTS);
		//glColor3f(1.0,0.0,0.0);

		if (m_densityR1 > 3.5)
			glColor3f(1.0,0.0,0.0);
		else if (m_densityR1 > 3.0)
			glColor3f(1.0,0.2,0.2);
		else if (m_densityR1 > 2.5)
			glColor3f(1.0,0.4,0.4);
		else if (m_densityR1 > 2)
			glColor3f(1.0,0.6,0.6);
		else if (m_densityR1 > 1.5)
			glColor3f(1.0,0.8,0.8);
		else if (m_densityR1 > 1.0)
			glColor3f(1.0,0.95,0.95);

		glVertex3f(mPos.x, mPos.y+0.7, mPos.z);
		glEnd();
	}
	#endif


	

 }



 void Chumanoid::createNewObstacle()
 {
 	list<sOBSTACLE> lobst;
	sOBSTACLE obst;
	list<sWALL> lwall;
	sWALL wall;

	//mPos.x = 3.7;
//	mPos.x = 9.4;
//	mPos.y = 0;
//	mPos.z = 7;
//	m_desiredVel.x = -0.5;
//	m_desiredVel.y = 0;
//	m_desiredVel.z = -0.6;
//	normalize(&m_desiredVel);

	tVector A,B,oldA,oldB;
	tVector point1,point2;
	tVector UP = {0,1,0};
	tVector 
		rightVec = cross(UP,m_desiredVel);
	normalize(&rightVec);

	lobst.clear();
	lwall.clear();
	obst.radio = 0.3;
	obst.renderBV = true;
	//for (int i=1; i<10; i=i+2)
	//	for (int i=5; i<10; i=i+4)
	//	{
	//		obst.center[0] = mPos.x + m_desiredVel.x*1.5*i/10;
	//		obst.center[1] = mPos.y + m_desiredVel.y*1.5*i/10;
	//		obst.center[2] = mPos.z + m_desiredVel.z*1.5*i/10;
	//		lobst.push_back(obst);
	//	}

	obst.center[0] = mPos.x + m_desiredVel.x*1;
	obst.center[1] = mPos.y + m_desiredVel.y*1;
	obst.center[2] = mPos.z + m_desiredVel.z*1;
	obst.radio = 0.7;
	lobst.push_back(obst);
	obst.center[0] = mPos.x + m_desiredVel.x*0.4;
	obst.center[1] = mPos.y + m_desiredVel.y*0.4;
	obst.center[2] = mPos.z + m_desiredVel.z*0.4;
	obst.radio = 0.35;
	//lobst.push_back(obst);
	obst.center[0] = mPos.x + m_desiredVel.x*1.5;
	obst.center[1] = mPos.y + m_desiredVel.y*1.5;
	obst.center[2] = mPos.z + m_desiredVel.z*1.5;
	//obst.radio = 0.35;
	//lobst.push_back(obst);

	// if we are very close to one of the walls, we need farther obst avoidance
	tVector closestWall; 
	if (m_crossingState<2)
		closestWall = m_pprevCell->VecToClosestWall(mPos+m_desiredVel);
	else
		closestWall = m_pcell->VecToClosestWall(mPos+m_desiredVel);
		
	if ((length(closestWall) < 0.5) && (length(closestWall) > 0.01))// an agent can't walk between me and the wall
	{
		obst.center[0] = mPos.x + m_desiredVel.x + closestWall.x;
		obst.center[1] = mPos.y + m_desiredVel.y + closestWall.y;
		obst.center[2] = mPos.z + m_desiredVel.z + closestWall.z;
		//cout << "dist to wall: " << length(closestWall) << "   center: (" << obst.center[0] <<","<< obst.center[2] <<")"<< endl;
		obst.radio = length(closestWall)+0.5;
		//lobst.push_back(obst);
	}
	
/*
	// Wall normal is the cross product: UPx(B-A), where UP={0,1,0}
	length = sqrt((wall.pointB[2] - wall.pointA[2])*(wall.pointB[2] - wall.pointA[2]) + (wall.pointB[0] - wall.pointA[0])*(wall.pointB[0] - wall.pointA[0]));
	wall.normal[0] = (wall.pointB[2] - wall.pointA[2]) / length ;
	wall.normal[1] = 0.0;
	wall.normal[2] = -(wall.pointB[0] - wall.pointA[0]) / length ;
	wall.D = - (wall.pointA[0]*wall.normal[0]+wall.pointA[1]*wall.normal[1]+wall.pointA[2]*wall.normal[2]);

*/

	point1 = mPos + m_desiredVel*(-0.2);
	point2 = mPos + m_desiredVel*1.7;
	oldA = A = mPos + m_desiredVel*(-0.2) + rightVec*0.3;
	oldB = B = mPos + m_desiredVel*1.7 + rightVec*0.3;
	
	float minX, maxX, minZ, maxZ;
	if (point1.x < point2.x)
	{
		minX = point1.x;
		maxX = point2.x;
	}
	else
	{
		minX = point2.x;
		maxX = point1.x;
	}

	if (point1.z < point2.z)
	{
		minZ = point1.z;
		maxZ = point2.z;
	}
	else
	{
		minZ = point2.z;
		maxZ = point1.z;
	}

	wall.thikness = 0;
	wall.pointA[0] = wall.pointB[0] = minX;
	wall.pointA[1] = wall.pointB[1] = 0;
	wall.pointA[2] = maxZ;
	wall.pointB[2] = minZ;
	wall.normal[0] = -1;
	wall.normal[1] = 0;
	wall.normal[2] = 0;
	wall.D = - (wall.pointA[0]*wall.normal[0]+wall.pointA[1]*wall.normal[1]+wall.pointA[2]*wall.normal[2]);
	//lwall.push_back(wall);

	wall.pointA[0] = wall.pointB[0] = maxX;
	wall.pointA[1] = wall.pointB[1] = 0;
	wall.pointA[2] = minZ;
	wall.pointB[2] = maxZ;
	wall.normal[0] = 1;
	wall.normal[1] = 0;
	wall.normal[2] = 0;
	wall.D = - (wall.pointA[0]*wall.normal[0]+wall.pointA[1]*wall.normal[1]+wall.pointA[2]*wall.normal[2]);
	//lwall.push_back(wall);

	wall.pointA[0] = minX;
	wall.pointB[0] = maxX;
	wall.pointA[1] = wall.pointB[1] = 0;
	wall.pointA[2] = wall.pointB[2] = minZ;				
	wall.normal[0] = 0;
	wall.normal[1] = 0;
	wall.normal[2] = -1;
	wall.D = - (wall.pointA[0]*wall.normal[0]+wall.pointA[1]*wall.normal[1]+wall.pointA[2]*wall.normal[2]);
	//lwall.push_back(wall);

	wall.pointA[0] = minX;
	wall.pointB[0] = maxX;
	wall.pointA[1] = wall.pointB[1] = 0;
	wall.pointA[2] = wall.pointB[2] = maxZ;				
	wall.normal[0] = 0;
	wall.normal[1] = 0;
	wall.normal[2] = 1;
	wall.D = - (wall.pointA[0]*wall.normal[0]+wall.pointA[1]*wall.normal[1]+wall.pointA[2]*wall.normal[2]);
	//lwall.push_back(wall);

				// if we are very close to one of the walls, we need farther obst avoidance
	/*		
	tVector closestWall1 = m_pcell->VecToClosestWall(point1);
	tVector closestWall2 = m_pcell->VecToClosestWall(point2);
	if ((length(closestWall) < 1.0) && (length(closestWall) > 0.25))// an agent can't walk between me and the wall
	{
		obst.center[0] = mPos.x + m_desiredVel.x + closestWall.x;
		obst.center[1] = mPos.y + m_desiredVel.y + closestWall.y;
		obst.center[2] = mPos.z + m_desiredVel.z + closestWall.z;
		obst.radio = length(closestWall)+0.8;
		lobst.push_back(obst);
	}
	*/
	
	wall.pointA[0] = A.x;
	wall.pointA[1] = A.y;
	wall.pointA[2] = A.z;
	wall.pointB[0] = B.x;
	wall.pointB[1] = B.y;
	wall.pointB[2] = B.z;
	wall.normal[0] = rightVec.x;
	wall.normal[1] = rightVec.y;
	wall.normal[2] = rightVec.z;
	wall.D = - (wall.pointA[0]*wall.normal[0]+wall.pointA[1]*wall.normal[1]+wall.pointA[2]*wall.normal[2]);
	wall.thikness = 0.0;
	lwall.push_back(wall);

	A = A - rightVec*0.6;
	B = B - rightVec*0.6;
	
	wall.pointA[0] = B.x;
	wall.pointA[1] = B.y;
	wall.pointA[2] = B.z;
	wall.pointB[0] = A.x;
	wall.pointB[1] = A.y;
	wall.pointB[2] = A.z;
	wall.normal[0] = -rightVec.x;
	wall.normal[1] = -rightVec.y;
	wall.normal[2] = -rightVec.z;
	wall.D = - (wall.pointA[0]*wall.normal[0]+wall.pointA[1]*wall.normal[1]+wall.pointA[2]*wall.normal[2]);
	lwall.push_back(wall);
	
	wall.pointA[0] = A.x;
	wall.pointA[1] = A.y;
	wall.pointA[2] = A.z;
	wall.pointB[0] = oldA.x;
	wall.pointB[1] = oldA.y;
	wall.pointB[2] = oldA.z;
	wall.normal[0] = -m_desiredVel.x;
	wall.normal[1] = -m_desiredVel.y;
	wall.normal[2] = -m_desiredVel.z;
	wall.D = - (wall.pointA[0]*wall.normal[0]+wall.pointA[1]*wall.normal[1]+wall.pointA[2]*wall.normal[2]);
	lwall.push_back(wall);
	
	wall.pointB[0] = B.x;
	wall.pointB[1] = B.y;
	wall.pointB[2] = B.z;
	wall.pointA[0] = oldB.x;
	wall.pointA[1] = oldB.y;
	wall.pointA[2] = oldB.z;
	wall.normal[0] = m_desiredVel.x;
	wall.normal[1] = m_desiredVel.y;
	wall.normal[2] = m_desiredVel.z;
	wall.D = - (wall.pointA[0]*wall.normal[0]+wall.pointA[1]*wall.normal[1]+wall.pointA[2]*wall.normal[2]);
	lwall.push_back(wall);
	

	if (m_crossingState<2)
	{
		//m_pprevCell->addListObstacles(lobst);
		//m_pprevCell->addListWalls(lwall);
		m_pprevCell->setListWeakWalls(lwall);
		m_pprevCell->setListWeakObstacles(lobst);
	}
	else
	{
		//m_pcell->addListObstacles(lobst);
		//m_pcell->addListWalls(lwall);
		m_pcell->setListWeakWalls(lwall);
		m_pcell->setListWeakObstacles(lobst);
	}

	mTrait.IncreaseImbalance(1);
	
	//cout << "In create obst, ID+ "<< m_humID <<" m_equ=" <<mTrait.GetImbalance() << endl; 
 }

void Chumanoid::updateHumanoidRendering(float res,float bodyW, float bodyH, float headH, float legsH, float legsW)
{
	humBodyWidth = bodyW*0.5;
	humBodyHeight = bodyH;
	humRes = res;
	humHeadHeight = headH*0.5;
	humLegsHeight = legsH;
	humLegsWidth =legsW*0.5;
}


void Chumanoid::renderHuman2D(float posX, float posY, float posZ)
{
	GLUquadric *myQuad;
	
	glLineWidth(1);
	//glBegin(GL_LINES);
	//	glVertex3f(m_lookAheadA.x, m_lookAheadA.y, m_lookAheadA.z);
	//	glVertex3f(m_lookAheadB.x, m_lookAheadB.y, m_lookAheadB.z);
	//glEnd();
	glBegin(GL_LINES);
		glVertex3f(mPos.x,mPos.y,mPos.z);
		//glVertex3f(mPos.x+m_desiredVel.x, mPos.y+m_desiredVel.y, mPos.z+m_desiredVel.z);
		glVertex3f(mPos.x+m_orientation.x, mPos.y+m_orientation.y, mPos.z+m_orientation.z);
	glEnd();
	
	glPushMatrix();
		glTranslatef(posX,posY,posZ);
		glRotatef(90, 1.0, 0.0, 0.0);
		myQuad = gluNewQuadric();
	//	gluSphere(myQuad, humBodyWidth , humRes, humRes); //funda added personal space
		gluDisk(myQuad, humBodyWidth, humBodyWidth + mTrait.GetPersonalSpace(), humRes, humRes); //funda added personal space

	
	glPopMatrix();

	//cout << "rendering: "<< m_orientation.x << ", " << m_orientation.y << ", " << m_orientation.z << endl;
};


void Chumanoid::renderHumanCal3D(float posX, float posY, float posZ)
{
	float angle;
	static float temp_cont=0;
	float velocity;

	//Just to get rid of white spots when figures break
	float diffuse[4] = {0,0,0,0};
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, diffuse);

	glPushMatrix();
		glTranslatef(posX,posY,posZ);

		if (ABS(m_orientation.z) > 1) 
			m_orientation.z*=0.999;
		angle = acos(m_orientation.z) * 180 / M_PI;
		if (m_orientation.x >0)
			glRotatef(angle,0,1,0);
		else
			glRotatef(-angle,0,1,0);

		float vel = length(mPos - m_prevpos);
	
		velocity = vel/(m_dt);  //velocity in m/ms
		//velocity = vel/(m_dt*2);  //velocity in m/ms
		if (m_densityAhead > 0.8)
			m_speed = velocity;

//		cout << "real velocity = " << velocity << "USER_velocity = " << USER_velocity << endl;
//		cout << "state = " << m_pcal3dAgent->getState() << endl;

		if ((velocity < 0.00075) && (m_pcal3dAgent->getState()!= Model::STATE_WALK_SLOW))
		{
			m_pcal3dAgent->setStateAgent(Model::STATE_WALK_SLOW,0.2);
			//cout << "STATE_WALK_SLOW" << endl;
		}
		else if ((velocity > 0.00075) && (velocity < 0.00135) && (m_pcal3dAgent->getState()!= Model::STATE_WALK_NORMAL))
		{
			m_pcal3dAgent->setStateAgent(Model::STATE_WALK_NORMAL,0.2);
			//cout << "STATE_WALK_NORMAL" << endl;
		}
		else if ((velocity > 0.00135) && (m_pcal3dAgent->getState()!= Model::STATE_WALK_FAST))
		{
			m_pcal3dAgent->setStateAgent(Model::STATE_WALK_FAST,0.2);
			//cout << "STATE_WALK_FAST" << endl;
		}
//		else
//			cout << "error?" <<endl;


		if (velocity != 0)
		{
			if (m_pcal3dAgent->getState() == Model::STATE_WALK_SLOW)
			{
				myControl->glWin->elapsed_time = (0.04 - ((0.0007-velocity)/0.02));
				//elapsed_time = (0.06 - ((0.0007-velocity)/0.01));
			}
			else if (m_pcal3dAgent->getState() == Model::STATE_WALK_NORMAL)
			{
				//elapsed_time = (0.08 - ((0.0013-velocity)/0.01)); //old one
				myControl->glWin->elapsed_time = (0.05 - ((0.0013-velocity)/0.02));
			}
			else if (m_pcal3dAgent->getState() == Model::STATE_WALK_FAST)
			{
				myControl->glWin->elapsed_time = (0.07 - ((0.0017-velocity)/0.01));
				//elapsed_time = (0.05 - ((0.00175-velocity)/0.01));
			}
		}
		else
		{
			if (m_doTask)
			{
				myControl->glWin->elapsed_time = m_dt/1000.0; // Time in seconds
				//cout << "task elapsed time"<<endl; 
			}
			else
				myControl->glWin->elapsed_time = 0;
		}

		int actionID=0; //6,7,8,9
		if (myControl->glWin->execute_idle || m_execute_idle)
		{
			//actionID = 6 + (rand()%3);
			//cout << "executing action"<<endl;
			//actionID = 3;
			//cout << "actionID = " << actionID;
//			m_tasks.front().timer -= m_dt;
//			cout <<" timer decrease= "<< m_tasks.front().timer<<endl;
			//m_pcal3dAgent->executeAction(actionID);
			if (!m_tasks.empty())
			if (!m_tasks.front().laction.empty())
			{
				actionID = m_tasks.front().laction.front();
				//cout << "actionID= " << actionID << endl;
				m_tasks.front().aux_actionTimer = m_pcal3dAgent->getCalCoreModel()->getCoreAnimation(actionID)->getDuration();
				m_pcal3dAgent->executeAction(actionID);
			}
			myControl->glWin->execute_idle = false;
			m_execute_idle = false;
		}
		
		// Jan added this line to be stupid
		//if (!m_tasks.empty())
		//	m_pcal3dAgent->getCalCoreModel()->getCoreAnimation(actionID)->setDuration(m_tasks.front().duration);

		// While we are in a task we'll be executing idle actions...
		if (m_doTask)
		{
			m_tasks.front().aux_actionTimer -= myControl->glWin->elapsed_time;
			if (m_tasks.front().aux_actionTimer<0)
			{
				// Jan added this line to be stupid should be timer?
				//m_pcal3dAgent->getCalCoreModel()->getCoreAnimation(actionID)->setDuration(m_tasks.front().timer);

				//cout << "action finished"<<endl;

				actionID = m_tasks.front().laction.front();  //Jan commented this out to see about smoother tasks
				m_tasks.front().laction.pop_front();
				m_tasks.front().laction.push_back(actionID);
				actionID = m_tasks.front().laction.front();		
				cout << "actionID= " << actionID << endl;
				m_pcal3dAgent->executeAction(actionID);
				m_tasks.front().aux_actionTimer = m_pcal3dAgent->getCalCoreModel()->getCoreAnimation(actionID)->getDuration();
			}
			//m_pcal3dAgent->getCalCoreModel()->getCoreAnimation(1)->getDuration();
		}

// OJO NURIA TE HAS QUEDADO AQUI!!!!!
		if (myControl->glWin->elapsed_time > 2)
			myControl->glWin->elapsed_time = 0.05;
		m_pcal3dAgent->onRender(myControl->glWin->elapsed_time);
		//cout << myControl->glWin->elapsed_time << endl;
//		cout << "myControl->glWin->elapsed_time = " << elapsed_time << endl;

	
	glPopMatrix();
}

void Chumanoid::renderHuman3D(float posX, float posY, float posZ)
{
	GLUquadric *myQuad;
	double angle;
//	tVector dir;
	float legAangle=0, legBangle=0;
	static float prevdist;	


	glEnable(GL_COLOR_MATERIAL);	

	tVector dif = mPos-m_prevpos;
	float val = LENGTH(dif.x,dif.z);
	if ((mPos != m_prevpos) && (LENGTH(dif.x,dif.z)>0.005)) //0.003
	{
		m_step_legs++;
	}
	else m_step_legs = 0;
	
	m_step_legs = m_step_legs % 30;

	switch (m_step_legs)
	{
	case 0:
	case 1:
	case 16:
	case 17: legAangle = legBangle = 0;
		break;

	case 2:
	case 3: legAangle = 2;
			legBangle = -4;
			break;
	case 4:
	case 5: legAangle = 5;
			legBangle = -10;
			break;
	case 6:
	case 7: legAangle = 10;
			legBangle = -20;
			break;
	case 8:
	case 9: legAangle = 15;
			legBangle = -30;
			break;
	case 10:
	case 11: legAangle = 10;
			legBangle = -20;
			break;
	case 12:
	case 13: legAangle = 5;
			legBangle = -10;
			break;
	case 14:
	case 15: legAangle = 2;
			legBangle = -4;
			break;

	case 18:
	case 19: legAangle = -2;
			legBangle = 4;
			break;
	case 20:
	case 21: legAangle = -5;
			legBangle = 10;
			break;
	case 22:
	case 23: legAangle = -10;
			legBangle = 20;
			break;
	case 24:
	case 25: legAangle = -15;
			legBangle = 30;
			break;
	case 26:
	case 27: legAangle = -10;
			legBangle = 20;
			break;
	case 28:
	case 29: legAangle = -5;
			legBangle = 10;
			break;
	case 30:
	case 31: legAangle = -2;
			legBangle = 4;
			break;
	
	};
	
	glPushMatrix();
		glTranslatef(posX,posY,posZ);
		

		// I want to orient the figure in the direction of its velocity vector
		// current orientation after rotating -90  in x is: (x,y,z)=(0,0,1)
		//length = LENGTH(m_desiredVel.x,m_desiredVel.z);
		//length < 0.0001 ? 0.0001 : length; 
		//dir = m_desiredVel/length;
		if (ABS(m_orientation.z) > 1) 
			m_orientation.z*=0.999;
		angle = acos(m_orientation.z) * 180 / M_PI;
		if (m_orientation.x >0)
			glRotatef(angle,0,1,0);
		else
			glRotatef(-angle,0,1,0);
		
		if (mTrait.GetImbalance() > 5) // fall!
		{
			glTranslatef(0,0.15,0);
			glRotatef(90,1,0,0);
			glTranslatef(0,-0.3,0);
		}

		glRotatef(-90, 1.0, 0.0, 0.0);
		myQuad = gluNewQuadric();
		// Legs
		if (male)
			glColor4fv(m_colorTrousers);
		else 
			glColor4fv(m_colorLegs);

		glPushMatrix();			
			glTranslatef(humBodyWidth*0.3,0.0,0.0);
			glTranslatef(0.0,0.0,humLegsHeight);
			glRotatef(legAangle,1,0,0);
			glTranslatef(0.0,0.0,-humLegsHeight);
			gluCylinder(myQuad,humLegsWidth,humLegsWidth,humLegsHeight,humRes,humRes);
			glTranslatef(0.0,0.0,humLegsHeight);
			glRotatef(legBangle,1,0,0);
			glTranslatef(0.0,0.0,-humLegsHeight);
			glTranslatef(-humBodyWidth*0.3*2,0.0,0.0);
			//glRotatef(-20,1,0,0);
			gluCylinder(myQuad,humLegsWidth,humLegsWidth,humLegsHeight,humRes,humRes);
		glPopMatrix();
		
		glTranslatef(0.0,0.0,humLegsHeight);
		myQuad = gluNewQuadric();
		// Body
		glColor4fv(m_colorTrousers);
		glPushMatrix();
			glPushMatrix();
				glScalef(1.0,0.7,1.0);
				if (male)
					gluCylinder(myQuad,humBodyWidth*0.7,humBodyWidth,humBodyHeight*0.35,humRes,humRes);
				else
				{
					glTranslatef(0.0,0.0,-0.15);
					gluCylinder(myQuad,humBodyWidth,humBodyWidth*0.7,humBodyHeight*0.55,humRes,humRes);
				}
			glPopMatrix();
			glTranslatef(0.0,0.0,humBodyHeight*0.35);


			
			glColor4fv(m_color);


			glPushMatrix();
				glScalef(1.0,0.7,1.0);
				if (male)
				   gluCylinder(myQuad,humBodyWidth,humBodyWidth,humBodyHeight*0.45,humRes,humRes);
				else
					gluCylinder(myQuad,humBodyWidth*0.7,humBodyWidth,humBodyHeight*0.45,humRes,humRes);
			glPopMatrix();
			glTranslatef(0.0,0.0,humBodyHeight*0.45);
			glPushMatrix();
				glScalef(1.0,0.7,1.0);
				gluCylinder(myQuad,humBodyWidth,humBodyWidth*0.2,humBodyHeight*0.2,humRes,humRes);
			glPopMatrix();
			myQuad = gluNewQuadric();
		
		// Head
		//glTranslatef(0.0,0.0,0.5);
			
			glTranslatef(0.0,0.0,humBodyHeight*0.2+humHeadHeight*0.8);
			glColor4fv(m_colorLegs);
			gluSphere(myQuad, humHeadHeight, humRes*2, humRes*2);
			glTranslatef(0.0,0.025,0.025);
			//funda glColor4fv(m_colorHairOriginal);
			if(mShowColorHairPersonality)
				glColor4fv(mTrait.GetHairColorPersonality());
			else
				glColor4fv(m_colorHairOriginal);
			gluSphere(myQuad, humHeadHeight, humRes*2, humRes*2); //hair
			if (!male)  // We draw a little pony tail for women
			{
				glTranslatef(0.0,humHeadHeight-0.025,-humHeadHeight*2);
				gluCylinder(myQuad,0.09,0.02,humHeadHeight*2,humRes,humRes);
			}
		glPopMatrix();
	glPopMatrix();

	glEnable(GL_COLOR_MATERIAL);	
}



void Chumanoid::draw()
{
	
	list<TASKS>::iterator iter;
//	double dt;

	//tVector nextStep;
		/*
	if (!SMOOTH_MOVE)
	{
		glColor3fv(m_color);
		renderHuman(m_myCell.col*SIZE_CELL+m_incrx,-m_myCell.row*SIZE_CELL-m_incry);
	}

	else
	{
		// Draw with route:
		if (local_motion == LINEAR_INTERP)
			{mPos = nextStep = calculateSteps();}
		else 
			{mPos = nextStep = calculateStepsHelbing();}
		m_route.push_back(nextStep);
		
		glColor3fv(m_color);
		renderHuman(nextStep.x,nextStep.y);

		tVector orient;
		orient.x = mPos.x - m_prevpos.x;
		orient.y = mPos.y - m_prevpos.y;
		script << m_humID<< "\t"<< mPos.x << "\t" << mPos.y<<"\t"<<orient.x<<"\t"<<orient.y<<"\t"<<endl;
	}
	*/

	#ifndef DEBUG_CROSSING_DOORS
	glColor4fv(m_color);	// OJO VALIDATION 15MAY07
	#endif
	
		if (myControl->glWin->RENDER2D)
		{
			renderHuman2D(mPos.x, mPos.y, mPos.z);
		}
		else
		{
			//funda #ifdef CAL3D_RENDERING
			if (mDrawCal3d)
				renderHumanCal3D(mPos.x, mPos.y, mPos.z);  //	RENDER CAL3D HUMAN FIGURE
			//funda #else
			else
				renderHuman3D(mPos.x, mPos.y, mPos.z);		//  RENDER OPENGL CUTE STICK FIGURE :)
			/*	//render the task locations
				for(iter = m_tasks.begin();iter != m_tasks.end();iter++)
				{
					glColor3f(0,1,0);
					glPushMatrix();
					glTranslatef(iter->location.x,iter->location.y+1,iter->location.z);					
					glutSolidSphere(0.5,5,5);
					glPopMatrix();
				}
			*/	
			//funda #endif
		}

		// VISUALIZATION OF DEBUGGING STUFF:
//		glPointSize(4);
//		glBegin(GL_POINTS);
//		glColor3f(1.0,0.0,0.0);
//		glVertex3f(mPos.x, mPos.y, mPos.z);
		//cout << " " << mPos.x << " " << mPos.y << " " << mPos.z << endl;
//		glEnd();

//		tVector orient;
//		orient.x = mPos.x - m_prevpos.x;
//		orient.y = mPos.y - m_prevpos.y;
//		script << m_humID<< "\t"<< mPos.x << "\t" << mPos.y<<"\t"<<orient.x<<"\t"<<orient.y<<"\t"<<endl;
};

void Chumanoid::drawVR()
{
	#ifndef DEBUG_CROSSING_DOORS
		glColor4fv(m_color);	// OJO VALIDATION 15MAY07
	#endif
	
	#ifdef CAL3D_RENDERING
				renderHumanCal3D(mPos.x, mPos.y, mPos.z);  //	RENDER CAL3D HUMAN FIGURE
			#else
				renderHuman3D(mPos.x, mPos.y, mPos.z);		//  RENDER OPENGL CUTE STICK FIGURE :)
			#endif
		
//		tVector orient;
//		orient.x = mPos.x - m_prevpos.x;
//		orient.y = mPos.y - m_prevpos.y;
//		script << m_humID<< "\t"<< mPos.x << "\t" << mPos.y<<"\t"<<orient.x<<"\t"<<orient.y<<"\t"<<endl;
};

void Chumanoid::updateBlockedList()
{
	// Look if any of my neighbor cells is blocked, and thus update my list
	//coord pos;
	CCell *cell;
	CCell *next_cell;
	map<int,sPORTAL> table_portals;
	map<int,sPORTAL>::iterator iter_portals;
	int id_nextCell;

	// cell = m_building->getCell(m_myCell);
	cell = m_pcell;		// OJO 15apr06

	table_portals = cell->getTablePortals();

	// look if any of my neighbouring cells is blocked:
	for (iter_portals = table_portals.begin(); iter_portals != table_portals.end(); iter_portals++)
	{
		id_nextCell = (*iter_portals).second.nextcellID;
		if (id_nextCell > 0)	// -1 means it's an exit, and therefore there's no hazard there.
		{

			next_cell = m_building->getCell(id_nextCell); 
			if (next_cell->getHazard()->getType())
				addBlockedCell(id_nextCell);
		}
	}
};

/*
void Chumanoid::updateBlockedList()
{
	// Look if any of my neighbor cells is blocked, and thus update my list
	//coord pos;
	CCell *cell;
	CCell *next_cell;
	map<int,sPORTAL> table_portals;
	map<int,sPORTAL>::iterator iter_portals;
	int id_nextCell;

	cell = m_building->getCell(m_myCell);
	table_portals = cell->getTablePortals();

	// look if any of my neighbouring cells is blocked:
	for (iter_portals = table_portals.begin(); iter_portals != table_portals.end(); iter_portals++)
	{
		id_nextCell = (*iter_portals).second.nextcellID;
		if (id_nextCell > 0)	// -1 means it's an exit, and therefore there's no hazard there.
		{

			next_cell = m_building->getCell(id_nextCell); 
			if (next_cell->getHazard()->getType())
				addBlockedCell(id_nextCell);
		}
	}
}
*/

// If we use hash tables:







void Chumanoid::updateClosedPortals()
{
	map<int,sPORTAL> table_portals;
	map<int,sPORTAL>::iterator iter_portals;
	int portalID;

	table_portals = m_pcell->getTablePortals();

	for (iter_portals = table_portals.begin(); iter_portals != table_portals.end(); iter_portals++)
	{
		// If a portal is opened and it appears in my memory of closed portals, then i'll remove it from that memory
/*		if (m_pcell->getPortal((*iter_portals).first).open) // Door Opened 
			if (m_listClosedPortals.find((*iter_portals).first) != m_listClosedPortals.end()) // Appears in my closed doors memory
				m_listClosedPortals.erase((*iter_portals).first);
		if (!m_pcell->getPortal((*iter_portals).first).open) // Door Closed
			m_listClosedPortals[(*iter_portals).first] = 1;
*/
		portalID = (*iter_portals).first;
		if (m_building->getPortalInfo(portalID).open) // Door Opened
			if (m_listClosedPortals.find(portalID) != m_listClosedPortals.end()) // Appears in my closed doors memory
				m_listClosedPortals.erase(portalID);
		if (!m_building->getPortalInfo(portalID).open)	// Door Closed
				m_listClosedPortals[portalID] = 1;
	}		
}

int Chumanoid::blocked(list<pairIDs> path)
{
	int val;
	// I need to check whether any of the cells in the exit path is in my hash table of blocked cells
	// with a value higher than 0

	list<pairIDs>::iterator path_iter;
	map<int,int>::iterator map_iter;
	

	if (!m_blockedTable.empty())
	{
		for (path_iter = path.begin(); path_iter != path.end(); path_iter++)
		{
			map_iter = m_blockedTable.find((*path_iter).idCell);
			if (map_iter != m_blockedTable.end())
			{
				val = m_blockedTable[(*path_iter).idCell];
				if (val > 0)
				{
					return 1; // as soon as one of the cells in the path is bloked, the whole path is discarded
				}
			}
			
		}
	}
	return 0;
}


void Chumanoid::enterCell()
{

	#ifdef COMMUNICATION
	{

		(m_building->getCell(m_prevCell))->mergeEnter(m_blockedTable, this);
	//funda	if (DFScom)
			if(mCommunicate)
			(m_building->getCell(m_prevCell))->shareFINISHEDcells(m_visitedCells,this);
	}
	#else //if (local_motion == HELBING)
	{
		// I need to update the people in each cell in case i want to use helbing
		(m_building->getCell(m_prevCell))->humsEnter(this);   // OJO 08MAY07  esto no se si hace falta!!!!! de hecho introduce errores!!!! pero si lo quito tengo intersecciones :(
	}
	#endif
}



void Chumanoid::exitCell()
{
	//(m_maze->getCell(m_prevCell))->mergeExit(m_blockedTable, this); // Esto lo usaba cuando la funcion era llamada DESPUES de decidir
	#ifdef COMMUNICATION
	{
		(m_building->getCell(m_prevCell))->mergeExit(m_blockedTable, this);	// Ahora salgo de la celda antes de decidir
	//funda	if (DFScom)
		if(mCommunicate)
			(m_building->getCell(m_prevCell))->removeFINISHEDcells(this);
	}
	#else //if (local_motion == HELBING)
	{
		// I need to update the people in each cell in case i want to use helbing
		(m_building->getCell(m_prevCell))->humsExit(this);
	}
	#endif
}


void Chumanoid::insertCoords(pair<int,int> newCoord)
{
	m_blockedTable[newCoord.first] = 2;  // ALWAYS VALUE 2, CANNOT BE BIGGER (newCoord.second + 1);
}

int Chumanoid::getLevelKnown(pair<int,int> newCoord)
{
	return (m_blockedTable[newCoord.first]);	
}


void Chumanoid::insertFinishedCells(pair<int,DFSdata> newdataDFS)
{
	//map<string,DFSdata>::iterator iter_DFS;
	//string cellName;
	//string parent;
	//map<string,DFSdata>::iterator iter_parent;
//	CCell *prevCell; 
	map<int,DFSdata>::iterator iter_DFS;
//	int cellName;
	int parent;
	map<int,DFSdata>::iterator iter_parent;

	
	// The humanoid will copy as FINISHED the newdataDFS (which is obtained from the m_finishedCell list of the current cell visited)
	// If i had not visited that cell yet, then the whole node information will be copied. If i had already visited the cell, then
	// I just neet to update the state to FINISHED.
	// IMP: before adding the cell as FINISHED, we'll now check whether that cell appears on my parents path to the root. In
	// such case, we wont copy that cell as FINISHED, we'll just ignore it. (intentamos evitar inconsistencias en el arbol y
	// que el agent se quede bloquedo)
	iter_DFS = 	m_visitedCells.find(newdataDFS.first);

	if (m_visitedCells.size() > 0)
	{
	// If newdataDFS doesn't exit in my DFS tree, then i'll add it, but keep in mind that i need to change the parent link so that
	// my DFS tree is coherent. My parent will be m_prevCell
	// If it already exists in my DFS tree, then i'll change its state to FINISHED but only when this change won't block my path. that is,
	// it's not on my parents path to the root of my DFS tree.
		if (iter_DFS == m_visitedCells.end())            
		{
//			newdataDFS.second.parentID = m_prevCell;
			m_visitedCells[newdataDFS.first] = newdataDFS.second;
		}
		else
		{
			//iter_parent = m_visitedCells.find(m_myCell);
			iter_parent = m_visitedCells.find(m_prevCell); // Find the cell where i am currently
			if (iter_parent == m_visitedCells.end()) // if m_myCell is not stored yet in m_visitedCells, then parent is m_prevCell
			{
				//parent = m_prevCell; 
				parent = m_pLastCell->getName(); 
			}
			else // else parent is given in the dataDFS information
				//parent = m_visitedCells[m_myCell].parentID;
				parent = m_visitedCells[m_prevCell].parentID;
			// The third parameter in NodeInParentsPath is used to make the search sorter, so if i set a big number is like not shortening the search
			if ((newdataDFS.first != HAZARD) &&	(parent != NO_PARENT) && (!NodeInParentsPath(newdataDFS.first,parent,1000)))
				m_visitedCells[newdataDFS.first].state = FINISHED;
		}
	}
}







// Depth First Search: 
// It returns the portal you need to go though
void Chumanoid::findDFScell()		// Depth first search algorithm used when the humanoid is lost.
{
	// We start the DFS tree from the cell we are originally. In this implementation, the exploration
	// part starts when i realize i'm lost. Whatever i've visited before doesn't count
	DFSdata data;
	//CCell *cell;
//	CCell *p_neighbor;
	//coord neighbor;
	map<int, DFSdata>::iterator iter_DFS;
	map<int,sPORTAL> table_portals;
	map<int,sPORTAL>::iterator iter_portals;
	int nrand;

	//cell = m_maze->getCell(m_myCell);

	exitCell(); // delete info from previous cell

	if (timeDFS == 0) // OJO A lo mejor deberia poner esto en la inicializacion
	{
		//first time we start the algorith, we'll set "wall" and "hazard" as finished cells. That will simplify the
		// thing i need to check during the algorithm
		data.state = FINISHED;
		// The root node of the DFS tree won't have a father (I may also need to include this in the hash table... not sure yet!)
		data.parentID = NO_PARENT;
		//m_visitedCells["wall"] =	data;
		m_visitedCells[HAZARD] = data;
		// NURIA! NO TENGO MUY CLARO LO QUE HACER CON ESTE CASO!!!
		
		//data.parent = "no parent";
	}
	if (m_visitedCells.find(m_myCell) == m_visitedCells.end())
	{// If it was an undiscoverd cell...
		data.parentID = m_prevCell; //m_maze->getCell(m_prevCell)->getName().name; // "no parent";
		data.state = VISITED;		// discovered

		// OJO:  no puede pasar que en una cell no haya una puerta q me lleve fuera del maze, pq en ese caso
		// estaria siguiendo el camino de la cell con lo cual no entraria en esta funcion!!!

		// Check what i have in each portal and write that information on my data structure DFSdata
		table_portals = m_pcell->getTablePortals();
		for (iter_portals = table_portals.begin(); iter_portals != table_portals.end(); iter_portals++)
		{
			if (m_building->getCell((*iter_portals).second.nextcellID)->getHazard()->getType())
				data.adjCells[(*iter_portals).first] = HAZARD;
			else	
				data.adjCells[(*iter_portals).first] = (*iter_portals).second.nextcellID; 			
		}
		m_visitedCells[m_myCell] = data;
	}
	else
	{ // The node had already been visited, and therefore all its information should be in the hast table:
		data = m_visitedCells[m_myCell];
	}

	// I need to select the door to follow among the ones that haven't been discovered yet. 
	int n_undiscoveredDoors = 0;
	int undiscovCells[20];

	// if the name stored for each direction is not in the hash table m_visitedCells it means i haven't 
	// discovered that cell yet
	table_portals = m_pcell->getTablePortals();

	for (iter_portals = table_portals.begin(); iter_portals != table_portals.end(); iter_portals++)
	{
		//printf("visit: %d, %d", (*iter_portals).first, data.adjCells[(*iter_portals).first]);
		if ( m_visitedCells.find(data.adjCells[(*iter_portals).first]) == m_visitedCells.end() ) 
		{
			// Make sure the door is opened
//			if (m_pcell->getPortal((*iter_portals).first).open)
			if (m_building->getPortalInfo((*iter_portals).first).open)
			{
				undiscovCells[n_undiscoveredDoors++] = (*iter_portals).second.nextcellID;
			}
		}
	}
	//printf("\n");

	m_prevCell = m_myCell;
	if (n_undiscoveredDoors == 0) // means all the neighbour cells have been visited, and so this cell will become black
	{
		data.state = FINISHED;	// Black, cell finished
		m_visitedCells[m_myCell] = data;	
		// if there's nothing left to explore, i'll just go backwards following the parent pointers until i reach a cell
		// that still has sth left to visit
		if (data.parentID != NO_PARENT)
		{
			m_myCell = data.parentID; // (m_maze->getCellbyName(data.parent))->getName();
			// check if there is a loop, so that we don't follow all the way back the same way we came, but skip it

			int neighbCell;
			// Find if any of my adjacent cells a part from my parent is in my path towards the root
			for (iter_portals = table_portals.begin(); iter_portals != table_portals.end(); iter_portals++)
			{
				//data.adjCells[(*iter_portals).first] = (*iter_portals).second.nextcellID; 			
				if ((m_visitedCells[data.adjCells[(*iter_portals).first]].state == VISITED) && (data.adjCells[(*iter_portals).first] != data.parentID))
					neighbCell = data.adjCells[(*iter_portals).first];
			}

			if (m_visitedCells[neighbCell].parentID != NO_PARENT)
			{
//ojo nuria				if (NodeInParentsPath(neighbCell,m_myCell,5))
//ojo nuria	 				updateNodesInParentsPath(neighbCell,m_myCell,5);
			}
		}
		
		else // I need to do something when i am in the root node and therefore parent = "no parent"
			// so far, i'll just check if any of the child nodes is in the state: VISITED and i'll go to that one.
		{

			for (iter_portals = table_portals.begin(); iter_portals != table_portals.end(); iter_portals++)
			{
				//printf("adj: p=%d, c=%d", data.adjCells[(*iter_portals).first)
				if ( m_visitedCells[data.adjCells[(*iter_portals).first]].state == VISITED)
				{
					// make sure the door is still opened!
//					if (m_pcell->getPortal((*iter_portals).first).open)
					if (m_building->getPortalInfo((*iter_portals).first).open)
						m_myCell = (*iter_portals).second.nextcellID;
				}
			}
/*
			if ( m_visitedCells[data.Ncell].state == VISITED)
					m_myCell.row = m_myCell.row - 1;
				else if ( m_visitedCells[data.Scell].state == VISITED)
					m_myCell.row = m_myCell.row + 1;
				else if ( m_visitedCells[data.Ecell].state == VISITED)
					m_myCell.col = m_myCell.col + 1;
				else if ( m_visitedCells[data.Wcell].state == VISITED)
					m_myCell.col = m_myCell.col - 1;
*/
		}
			
	}
	else // There are still cells to visit from the current cell. I have those possible cells stored in undiscovCells, so
		// I'll just chose one randomly
	{
		nrand = rand() % n_undiscoveredDoors;
		//m_myCell.name = undiscovCells[nrand];
		m_myCell = undiscovCells[nrand];//    undiscovCells[nrand];
	}

	timeDFS++;

	// save myself from getting stuck:
	if (m_myCell == m_prevCell)
	{
		forgetFINISHEDcells();	
		createDFSnode(NO_PARENT);
	}
	
	enterCell(); // add info to new cell
	m_pcell = m_building->getCell(m_myCell);
	m_pLastCell = m_pprevCell;
	m_pprevCell = m_building->getCell(m_prevCell);
		
}


void Chumanoid::createDFSnode(int parent)
{

	DFSdata data;
	CCell *cell;
	CCell *p_neighbor;
	int neighbor;
	map<int, DFSdata>::iterator iter_DFS;
	int numDoors=0;
	map<int,sPORTAL> table_portals;
	map<int,sPORTAL>::iterator iter_portals;


	cell = m_building->getCell(m_prevCell);
	if (m_prevCell == 3)
		int prueba = 0;
	
	if (m_visitedCells.find(m_prevCell) == m_visitedCells.end())
	{// If it was an undiscoverd cell...
		if (m_visitedCells.size() > 0)
			data.parentID = parent; // m_maze->getCell(parent)->getName().name; // "no parent";
		else
			data.parentID = NO_PARENT; // If it is the first node being explored, it will be the root of the DFS tree
			//data.parent = "no parent"; // If it is the first node being explored, it will be the root of the DFS tree
		data.state = VISITED;		// discovered

		// OJO:  no puede pasar que en una cell no haya una puerta q me lleve fuera del maze, pq en ese caso
		// estaria siguiendo el camino de la cell con lo cual no entraria en esta funcion!!!

		// Check what i have in each direction and write that information on my data structure DFSdata
		// What do i have in the north? could be another cell, a door or a hazard

		// go through all the portals of my current cell
		
		table_portals = cell->getTablePortals();
		for (iter_portals = table_portals.begin(); iter_portals != table_portals.end(); iter_portals++)
		{
			neighbor = (*iter_portals).second.nextcellID;
			
			
			if (neighbor != NO_PARENT)
			{
				p_neighbor = m_building->getCell(neighbor);
				if(p_neighbor!=NULL)
				{
					if (p_neighbor->getHazard()->getType()) //there's a hazarad
						data.adjCells[(*iter_portals).first] = HAZARD;
					else // there's another cell, which is the neighbout
					{
						data.adjCells[(*iter_portals).first] = neighbor;
						numDoors++;
					}
				}
			}
		}

///
//		if (cell->getWall('N')) 
//		{// There's a wall
//			data.Ncell = "wall";
//		}
//		else 
//		{	
//			neighbor.row = m_prevCell.row - 1;
//			neighbor.col = m_prevCell.col;
//			p_neighbor = m_maze->getCell(neighbor);
//			if (p_neighbor->getHazard()->getType()) //there's a hazarad
//				data.Ncell = "hazard";
//			else // there's another cell, which is the neighbout
//			{
//				data.Ncell = p_neighbor->getName().name;
//				numDoors++;
//			}
//		}
///
		if (numDoors == 1)
		{
			// Since this room has only the exit from which we came, we can mark it as FINISHED
			data.state = FINISHED;
		}
		m_visitedCells[m_prevCell] = data;
	}
	else
	{ // The node had already been visited, and therefore all its information should be in the hast table:
	//	data = m_visitedCells[cell->getName().name];
	} // para que hago esto ultimo???? creo q no necesito esto para nada!!!! asi q lo comento
}


void Chumanoid::resetTimeDFS()
{
	map<string, DFSdata>	finishedCells;
	map<string, DFSdata>::iterator iter_DFS;

	/* // NEW!!! IMP!!! so far we don't want to loose our memory
	// Includes memory! those cells that where in the FINISHED status are kept, and therefore we won't explore them again
	if (timeDFS != 0) // It means there was a DFS created, so i need to save the memory of known cells.
	{
		// Search for finished cells:
		for (iter_DFS = m_visitedCells.begin(); iter_DFS != m_visitedCells.end(); iter_DFS++)
		{
			if ((*iter_DFS).second.state == FINISHED)
				finishedCells[(*iter_DFS).first] = (*iter_DFS).second;
		}
		m_visitedCells.clear();
		m_visitedCells = finishedCells;
	}
	*/

	timeDFS=0;
	
}



// To find out whether that node is in my list of nodes towards the root of the tree
// this function will be called recursively until the parent is the root of the DFS tree of the node matches the current node
bool Chumanoid::NodeInParentsPath(int finishedNode,int currentParent,int steps)
{
	// El current parent que meto al principio aun no esta en la estructura, asi q lo mejor es hacer 
	// esa comprobacion antes de llamar a esta funcion y luego llamar a esta funcion con el padre de ese nodo
	// en caso de q exista.
	// lo deberia comprobar aqui dentro por modularidad, y crear otra funcion para la
	// parte recursiva.

	if (steps == 0)
		return false;
	steps--;

	map<int, DFSdata>::iterator iter_cells;

	if (finishedNode == currentParent)
		return true;
	
	else
	{	
		{
		//	cout << currentParent << "  " << m_visitedCells[currentParent].parent << endl;
			if (m_visitedCells[currentParent].parentID == NO_PARENT) // If we reach the root, then the funtion returns false
				return false;
			else if (m_visitedCells[currentParent].state != FINISHED)
				return NodeInParentsPath(finishedNode,m_visitedCells[currentParent].parentID, steps);
		}
	}
}

// If there was a loop of visited cells, instead of following all the path back step by step, i'll update all the nodes and 


bool Chumanoid::updateNodesInParentsPath(int finishedNode,int currentParent,int steps)
{
	DFSdata data;
	int n_undiscoveredDoors = 0;
	map<int, sPORTAL> table_portals;
	map<int, sPORTAL>::iterator iter_portals;

	if (steps == 0)
		return false;
	steps--;

	if (finishedNode == currentParent)
	{
		m_myCell = currentParent; //(m_maze->getCellbyName(currentParent))->getName();
		return true;
	}
	else
	{	
	//	cout << currentParent << "  " << m_visitedCells[currentParent].parent << endl;
		if (m_visitedCells[currentParent].parentID == NO_PARENT) // If we reach the root, then the funtion returns false
			return false;
		else
		{
			data = m_visitedCells[m_visitedCells[currentParent].parentID];
			// if all the nodes around the parent have been visited of finished then we'll mark that one as finished too
			table_portals = m_pcell->getTablePortals();
			for (iter_portals = table_portals.begin(); iter_portals != table_portals.end(); iter_portals++)
			{
				if ( m_visitedCells.find(data.adjCells[(*iter_portals).first]) == m_visitedCells.end() ) 
					n_undiscoveredDoors++;
			
			}
			if (n_undiscoveredDoors == 0)			
			{
				data = m_visitedCells[m_visitedCells[currentParent].parentID];
				data.state = FINISHED;
				m_visitedCells[m_visitedCells[currentParent].parentID] = data;
				
				return updateNodesInParentsPath(finishedNode,m_visitedCells[currentParent].parentID,steps);
			}
			else
				return false;
		}
	}
}


void Chumanoid::forgetFINISHEDcells()
{
//	string aux_visitedCells[10000];
//	map<string, DFSdata>::iterator iter_visited;
//	int n=0;
//	map<string, DFSdata> aux_cells;
	DFSdata data;
/*
	aux_cells["wall"] = m_visitedCells["wall"];
	aux_cells["hazard"] = m_visitedCells["hazard"];

	for (iter_visited = m_visitedCells.begin(); iter_visited != m_visitedCells.end(); iter_visited++)
	{
			aux_visitedCells[n++] = (*iter_visited).first;
	}
	for (int i=0; i<n; i++)
		m_visitedCells.erase(aux_visitedCells[i]);
*/

	m_visitedCells.clear();
	data.state = FINISHED;
	data.parentID = NO_PARENT;
	m_visitedCells[HAZARD] = data;
	
	//(m_building->->getCell(m_myCell))->deleteALLcells();	
	m_pcell->deleteALLcells();   //	OJO NO ESTOY SEGURA DE QUE SEA M_PCELL O M_PPREVCELL, O NINGUNO

}

/*
void Chumanoid::forgetFINISHEDcells()
{
	string aux_visitedCells[10000];
	map<string, DFSdata>::iterator iter_visited;
	int n=0;
	map<string, DFSdata> aux_cells;

	aux_cells["wall"] = m_visitedCells["wall"];
	aux_cells["hazard"] = m_visitedCells["hazard"];

	//aux_visitedCells = m_visitedCells;
	for (iter_visited = m_visitedCells.begin(); iter_visited != m_visitedCells.end(); iter_visited++)
	{
//		cout << (*iter_visited).first << " " << (*iter_visited).second.state << endl;
		//if ((*iter_visited).second.state == FINISHED)
			aux_visitedCells[n++] = (*iter_visited).first;
	//		aux_visitedCells.erase(iter_visited);
	}
	for (int i=0; i<n; i++)
		m_visitedCells.erase(aux_visitedCells[i]);

	m_visitedCells["wall"] = aux_cells["wall"];
	m_visitedCells["hazard"] = aux_cells["hazard"];

	(m_maze->getCell(m_myCell))->deleteALLcells();	
}
*/



/*

tVector Chumanoid::calculateStepsHelbing()
{
	tVector step;
	float dif;
//	tVector result;
	tVector destPos;
	float offset = 0.0;//4.0;	// How much into the next cell you want to enter1 (attractor point)
	float offsetWall = 0.8;	//0.4 //0.3//  Distance you want to keep from the skeleton of the wall
	float offsetDoor = 0.4;
	float offsetPeople = 1.4; //1.3// Distance you want to keep between centers of people
	tVector collForce = {0.0,0.0};
	tVector wallForce = {0.0,0.0};
	float wallForceVal = 2.0;
	tVector forwards = {0.0,0.0};
	tVector attracForce;
	tVector distDoor;
	char dir='S'; // Lo inicializa a S para que no me falle cuando la m_myCell and m_prevCell son la misma!!!!

	dif = SIZE_CELL / (float)NUM_tVectorS;

	destPos = mPos;
	

	int pruebaB=0;
	if ((m_myCell.name == "r0c0"))// && (m_humID == 5))
		pruebaB++;

	// In order to avoid gridlocks, we introduce some randomnes in the offsetpeople:
	offsetPeople += pow(-1,rand())*0.05;
	offsetPeople += pow(-1,rand())*0.03;


	if (m_myCell.col != m_prevCell.col)
	{
		destPos.y = -m_myCell.row*SIZE_CELL - SIZE_CELL/2.0;

		forwards.y = 0.0;

		if (m_myCell.col < m_prevCell.col)
		{
			destPos.x = m_myCell.col*SIZE_CELL + SIZE_CELL - offset;
			forwards.x =  -0.1;
			dir = 'W';
			//wall collision:
			//if (fabs(mPos.x-(m_myCell.col+1)*SIZE_CELL) < 1.0)
			//	wallForce.x = wallForceVal;
		}
		else
		{
			destPos.x = m_myCell.col*SIZE_CELL + offset;
			forwards.x = 0.1;
			dir = 'E';
			//wall collision:
			//if (fabs(mPos.x-(m_myCell.col)*SIZE_CELL) < 1.0)
			//	wallForce.x = -wallForceVal;
		}
	}

	else if (m_myCell.row != m_prevCell.row)
	{
		destPos.x = m_myCell.col*SIZE_CELL + SIZE_CELL/2.0;
		forwards.x = 0.0;

		if (m_myCell.row < m_prevCell.row)
		{
			destPos.y = -m_myCell.row*SIZE_CELL - SIZE_CELL  + offset;
			forwards.y = 0.1;
			dir = 'N';
			//wall collision:
			//if (fabs(mPos.y+(m_myCell.row+1)*SIZE_CELL) < 1.0)
				//wallForce.y = +wallForceVal;
		}
		else
		{
			destPos.y = -m_myCell.row*SIZE_CELL - offset;
			forwards.y = -0.1;
			dir = 'S';
			//wall collision:
			//if (fabs(mPos.y+(m_myCell.row)*SIZE_CELL) < 1.0)
				//wallForce.y = -wallForceVal;
		}
	}

	// Avoid Collision with other hums:
	list<Chumanoid*> listHums;
	list<Chumanoid*>::iterator iterHums;
	tVector posHum, diff;
	

	Chumanoid *pruebaHum;
	listHums = m_pcell->getHumanoidsList();
	for (iterHums = listHums.begin(); iterHums != listHums.end(); iterHums++)
	{
		pruebaHum = (*iterHums);
		if ((*iterHums) != this)
		{
			posHum = (*iterHums)->getPosition();
			diff.x = (mPos.x - posHum.x);
			diff.y = (mPos.y - posHum.y);
		
			float dist = sqrt(diff.x*diff.x + diff.y*diff.y); 

			if (dist < 0.2)
			{
				pruebaB++;
				diff.x = 0.2;
				diff.y = 0.2;
			}

			float randValx = 0;//(rand()%10)*0.01;
			float randValy = 0;//(rand()%10)*0.01;
		
			//if ((fabs(diff.x) < offsetPeople ) && (fabs(diff.y) < offsetPeople ))
			if (dist < offsetPeople )
			{
				//collForce.x = collForce.x + diff.x; //+ SIZE_CELL*0.1 - diff.x;
				//collForce.y = collForce.y + diff.y; //+ SIZE_CELL*0.1 - diff.y;
				if (fabs(diff.x) > 0.0)
					collForce.x = collForce.x + (diff.x/(fabs(diff.x))* (offsetPeople - fabs(diff.x)))+randValx; //+ SIZE_CELL*0.1 - diff.x;
				if (fabs(diff.y) > 0.0)
					collForce.y = collForce.y + (diff.y/(fabs(diff.y))* (offsetPeople - fabs(diff.y)))+randValy; //+ SIZE_CELL*0.1 - diff.y;
			}
		}
		//float randValx = (rand()%10)*0.01;
		//float randValy = (rand()%10)*0.01;
		//collForce.x += randValx;
		//collForce.y += randValy;
		
	
	}


	listHums = m_pprevCell->getHumanoidsList();
	for (iterHums = listHums.begin(); iterHums != listHums.end(); iterHums++)
	{
		pruebaHum = (*iterHums);
		if ((*iterHums) != this)
		{
			posHum = (*iterHums)->getPosition();
			diff.x = (mPos.x - posHum.x);
			diff.y = (mPos.y - posHum.y);

			float dist = sqrt(diff.x*diff.x + diff.y*diff.y); 

			if (dist < 0.2)
			{
				pruebaB++;
				diff.x = 0.2;
				diff.y = 0.2;
			}

			float randValx = 0;//(rand()%10)*0.01;
			float randValy = 0;//(rand()%10)*0.01;
		
			//if ((fabs(diff.x) < offsetPeople ) && (fabs(diff.y) < offsetPeople ))
			if (dist < offsetPeople)
			{
				//collForce.x = collForce.x + diff.x; //+ SIZE_CELL*0.1 - diff.x;
				//collForce.y = collForce.y + diff.y; //+ SIZE_CELL*0.1 - diff.y;
				if (fabs(diff.x) > 0.0) //????
					collForce.x = collForce.x + (diff.x/(fabs(diff.x))* (offsetPeople - fabs(diff.x)))+randValx; //+ SIZE_CELL*0.1 - diff.x;
				if (fabs(diff.y) > 0.0) //????
					collForce.y = collForce.y + (diff.y/(fabs(diff.y))* (offsetPeople - fabs(diff.y)))+randValy; //+ SIZE_CELL*0.1 - diff.y;
			}
			//float randValx = (rand()%10)*0.01;
			//float randValy = (rand()%10)*0.01;
			//collForce.x += randValx;
			//collForce.y += randValy;
		}
	}



	//Avoid collision with walls:

	tVector center;
	center.x = m_prevCell.col * SIZE_CELL + SIZE_CELL*0.5;
	center.y = -m_prevCell.row * SIZE_CELL- SIZE_CELL*0.5;


	//float pruebaX = (destPos.x - mPos.x) / (NUM_tVectorS - num_step);
	//float pruebaY = (destPos.y - mPos.y) / (NUM_tVectorS - num_step);
	
	float dist = sqrt((mPos.x-destPos.x)*(mPos.x-destPos.x) + (mPos.y-destPos.y)*(mPos.y-destPos.y)); 
	
	tVector desiredVel;
	float lengthVel; 

	desiredVel.x = mPos.x - m_prevpos.x;
	desiredVel.y = mPos.y - m_prevpos.y;
	lengthVel = sqrt(desiredVel.x*desiredVel.x + desiredVel.y*desiredVel.y);
	if (lengthVel == 0)
		lengthVel = 0.001;
	desiredVel.x = desiredVel.x / lengthVel;
	desiredVel.y = desiredVel.y / lengthVel;


	m_prevpos = mPos;
	if ((dist < offset*0.0) && (dist > 0.0))
	{
		step.x = mPos.x + forwards.x*10.0 / 1.0*(NUM_tVectorS - num_step)  + collForce.x*1.0;// + wallForce.x*0.3;
		step.y = mPos.y + forwards.y*10.0 / 1.0*(NUM_tVectorS - num_step) + collForce.y*1.0;// + wallForce.y*0.3;
		//step.x = mPos.x + (destPos.x - mPos.x) / (NUM_tVectorS - num_step) + collForce.x*0.3;// + wallForce.x*0.3;
		//step.y = mPos.y + (destPos.y - mPos.y) / (NUM_tVectorS - num_step) + collForce.y*0.3;// + wallForce.y*0.3;
	}
	else
	{
		float dist = sqrt(collForce.x*collForce.x + collForce.y*collForce.y);
		if (dist > 1)
		{
			collForce.x = 1*(collForce.x / dist);
			collForce.y = 1*(collForce.y / dist);
		}
		tVector sumForces;
	
		// Force towards attraction point:
		//tVector attracForce;
		attracForce.x = (destPos.x - mPos.x) / (NUM_tVectorS - num_step);
		attracForce.y = (destPos.y - mPos.y) / (NUM_tVectorS - num_step);

		int prueba = 0;
		if ((m_myCell.row == 9) && (m_myCell.col == 9))
			prueba++;
	//0.0,0.4,0.5
//		sumForces.x = 0.0*desiredVel.x + 0.6*attracForce.x  + collForce.x*1.3;// + wallForce.x*0.3;
//		sumForces.y = 0.0*desiredVel.y + 0.6*attracForce.y  + collForce.y*1.3;// + wallForce.y*0.3;

		sumForces.x = 0.0*desiredVel.x + 0.5*attracForce.x  + collForce.x*0.5;//0.7;// + wallForce.x*0.3;
		sumForces.y = 0.0*desiredVel.y + 0.5*attracForce.y  + collForce.y*0.5;//0.7;// + wallForce.y*0.3;

		dist = sqrt(sumForces.x*sumForces.x + sumForces.y*sumForces.y);
		if (dist > 1.0)
		{
			sumForces.x /= dist;
			sumForces.y /= dist;
		}
		step.x = mPos.x + sumForces.x;
		step.y = mPos.y + sumForces.y;

	}

	float pruebadist = sqrt((mPos.x-step.x)*(mPos.x-step.x) + (mPos.y-step.y)*(mPos.y-step.y));


	// Check collision against the walls:
	
	float wallpos;
	//float wallDist = 0.25;
	if (m_myCell.col == m_prevCell.col) // check collision against vertical walls (x values)
	{
		if (mPos.x < step.x) // it has moved towards the right
		{
			wallpos = ((m_myCell.col+1) * SIZE_CELL) - offsetWall; // wallDist/2.0;
			if (step.x > wallpos)
				step.x = wallpos;// - offsetWall;
		}
		else if (mPos.x > step.x) // it has moved towards the left
		{
			wallpos = ((m_myCell.col) * SIZE_CELL) + offsetWall; // wallDist/2.0;
			 if (step.x < wallpos)
				step.x = wallpos;
		}

		//horizontal doors
		
		wallpos = ((m_myCell.col) * SIZE_CELL);// + offsetWall ; //wallDist/2.0;
		//if ((step.x < (wallpos + SIZE_CELL/4.0 + offsetWall*0.5)) || (step.x > (wallpos + SIZE_CELL*3.0/4.0 - offsetWall*0.5)))
		if ((step.x < (wallpos + SIZE_CELL/4.0 + offsetDoor)) || (step.x > (wallpos + SIZE_CELL*3.0/4.0 - offsetDoor)))
		{
			if (mPos.y < step.y) // it has moved up
			{
				wallpos = (- (m_myCell.row+1) * SIZE_CELL) - offsetWall;//*3; //wallDist/2.0;
				if ((step.y > wallpos) && (dir=='N'))
					step.y = wallpos;// - offsetWall; //wallDist;
			}
			else if (mPos.y > step.y) // it has moved down
			{
				wallpos = (- (m_myCell.row) * SIZE_CELL) + offsetWall;//*3; //wallDist/2.0;
				if ((step.y < wallpos) && (dir=='S'))
					step.y = wallpos;// + offsetWall; //wallDist;
			}
		}
		

	}
	else if (m_myCell.row == m_prevCell.row) // check collision against horizontal walls (y values)
	{
		if (mPos.y < step.y) // it has moved up
		{
			wallpos = (- m_myCell.row * SIZE_CELL) - offsetWall;//wallDist/2.0;
			if (step.y > wallpos)
				step.y = wallpos;// - offsetWall; //wallDist;
		}
		else if (mPos.y > step.y) // it has moved down
		{
			wallpos = (- (m_myCell.row+1) * SIZE_CELL) + offsetWall;//wallDist/2.0;
			if (step.y < wallpos)
				step.y = wallpos;// + offsetWall;//wallDist;
		}

		// Vertical doors:
		wallpos = (- m_myCell.row * SIZE_CELL);
		if ((step.y > (wallpos - (SIZE_CELL/4.0) - offsetDoor)) || (step.y < (wallpos - (SIZE_CELL*3.0/4.0) + offsetDoor)))
		{
			if ((mPos.x < step.x) && (dir=='E')) // it has moved towards the right
			{
				wallpos = ((m_myCell.col) * SIZE_CELL) - offsetWall;//*3;//wallDist/2.0;
				if ((step.x > wallpos))// && (mPos.x < wallpos))
					step.x = wallpos;// - offsetWall;//wallDist;
			}
			else if ((mPos.x > step.x) && (dir=='W'))// it has moved towards the left
			{
				wallpos = ((m_myCell.col+1) * SIZE_CELL) + offsetWall;//*3;//wallDist/2.0;
				if ((step.x < wallpos))// && (mPos.x > wallpos))
					step.x = wallpos;// + offsetWall;//wallDist;
			}
		}
		
	}
	

	num_step = (num_step + 1) % NUMtVectorS;
	
	// OJO this is done to avoid having all in the exit
		if (((0 == m_myCell.row) && (0 == m_myCell.col))|| ((0 == m_prevCell.row) && (0 == m_prevCell.col)))
		{
			step.y += 0.25;
		}
		else
		if (((9 == m_myCell.row) && (9 == m_myCell.col))|| ((9 == m_prevCell.row) && (9 == m_prevCell.col)))
		{
			step.y -= 0.25;
		}
	
    // Check the distance to the attractor point, and when it's very closed, we'll change m_myCell
	distDoor.x = step.x - destPos.x;
	distDoor.y = step.y - destPos.y;

	if (sqrt((distDoor.x)*(distDoor.x) + (distDoor.y)*(distDoor.y)) < 1.25) // Distance to attractor = 1.25
		m_doorCrossed = true;


	return step;

}
*/

// Calculates NUMtVectorS steps between its position in the current cell and its position in the next cell:

tVector Chumanoid::calculateSteps()
{
	tVector step;
//	float dif;
//	tVector org,dst;
	
	//time_t time;
	float dist, dist2;
	float distPrevAtt, distLastAtt; 
	tVector fTotal, fAttrac, fWalls, fObst = {0.0,0.0,0.0};
	tVector fDesiredSpeed = {0.0,0.0,0.0};
	tVector fLook4Walls = {0.0, 0.0, 0.0};
	tVector fOtherAgents = {0.0, 0.0, 0.0};
	tVector fOthers0 = {0.0, 0.0, 0.0};
	tVector fOthers1 = {0.0, 0.0, 0.0};
	double speed;
//	double dt;
	float fWeightAttrac;
	double* pos;
	tVector fAvoidFallenAgents;
	float fFallenAgent;
	tVector fAvoidCircles;
	tVector aux_orientation, aux_velocity;
	tVector f_friendship = {0.0, 0.0, 0.0};
	tVector f_lynching = {0.0, 0.0, 0.0};
	float	weight_friendship = 1.0;
	float	weight_lynching = 1.0;

	unsigned long currTime;
	static unsigned long incrTime=0;
	static unsigned long frameRate=0;
	static unsigned long prevTime=0;
	//static bool nextTask = false;
	
	//cout << "HiDAC Model"<<endl;

	_timeb tm;	
	_ftime(&tm);
	currTime = tm.time*1000 + tm.millitm; 
	incrTime += (currTime - prevTime);

	// Accelerate
	if (m_speed < m_maxSpeed)
	{
		m_speed = m_speed + 0.000001*m_dt;
	}
	else
		m_speed = m_maxSpeed;

	speed = m_speed;

	// //Decelerate
	if (m_densityAhead > 0.8)
		if (m_speed > 0.0001)
			m_speed = m_speed - 0.000001*m_dt;
		else
			m_speed = 0;

//	frameRate++;

	//cout << "FR= " << incrTime<< endl;
//	if (incrTime>10000)
//	{
//		cout << "FR= " << frameRate/10.0 << endl;
//		incrTime = 0;
//		frameRate = 0;
//	}
	
	#ifdef REAL_TIME
	//	_timeb tm;
	//	_ftime(&tm);
	//	currTime = tm.time*1000 + tm.millitm; 
		m_dt = currTime - prevTime; 
		//printf("dt = %f",dt);
	//	m_time = currTime ;
	#else
		m_dt = dif_millSecs; //200;
	#endif

	prevTime = currTime ;

	// We reduce the speed of the human when they are going downstairs
	sSTAIR stair;
	if (m_pprevCell->getIsStair(&stair))
	{
		speed = speed * 0.8; //0.57;
	}

	
	if ((!m_nextTask) && (!m_doTask) && (m_crossingState<0) && (!m_tasks.empty()))
		if ((m_tasks.front()).roomID == m_prevCell) // OJO antes era prev!!!!
			m_nextTask=true;
		

	if (m_nextTask)
		fAttrac = m_tasks.front().location - mPos;
	else 
		fAttrac = m_attractor - mPos;

	dist = LENGTH(fAttrac.x,fAttrac.z);
	if (dist  > 0)
		fAttrac = fAttrac / dist;
	if (dist < 1.5)
		fWeightAttrac = 35;
	else
		fWeightAttrac = 10;	
	
	fObst = m_pprevCell->RepulsionObstacles(mPos, m_lookAheadA, m_lookAheadB, m_desiredVel,0); // 0 means non-traversable obstacles
	
	// I need repulsion from others in both my current cell and the previous one, to avoid collisions in the portals
	// Repulsion Other Agents will give the repulsion force from other agent that is getting too close, or a vector of length 1
	// if we only need to turn a little bit because there's another agent a bit ahead of us, or a vector of lengh 2 is we have
	// somebody right in the position where we want to move to, and therefore we shouldn't move
	
	//desPos = mPos + fAttrac/4; ///3; // It is now calculated inside the RepulsionOtherAgentsFunction
	
	

	glColor4fv(m_color);
	fOtherAgents = m_pprevCell->RepulsionOtherAgents(this, m_lookAheadA, m_lookAheadB, fAttrac,m_portalIDcoll);

	if(mTrait.GetFriendship())
		f_friendship = m_pprevCell->ComputeFriendship(this);

	if(mTrait.GetLynching())
	{
		f_lynching = m_pprevCell->ComputeLynchingBehavior(this);
		fAttrac.x = fAttrac.y = fAttrac.z = 0;  //clear all attraction forces
	}

	


	fAvoidFallenAgents = m_pprevCell->tangentialWall(mPos, fAttrac);
	fAvoidCircles = m_pprevCell->RepulsionObstacles(mPos, m_lookAheadA, m_lookAheadB, fAttrac, 1);// false means the obstacles can be traversable

//	fWalls = m_pprevCell->RepulsionWalls(mPos, m_desiredVel);
	fWalls = m_pprevCell->RepulsionWalls(mPos, fAttrac);

	float lengthForceAgents = LENGTH(fOtherAgents.x, fOtherAgents.z);	
	// if lengthForceAgents < 1 it means we are being pushed, therefore, a pushing force of 0.8 or higher, i'll consider it
	// strong enough to affect my equilibrium...
	//if ((lengthForceAgents < 1) && (lengthForceAgents>0.1125) && (rand()%70 > 25))
	//	mTrait.IncreaseImbalance(1);
	//else 
	//	mTrait.SetImbalance(0);	// set a 0 if i haven't been pushed


//	mTrait.IncreaseImbalance(lengthForceAgents);

	// when tangentical forces apply or non forces fAgents will have a value different than 0
	// If repulsion forces need to be applied then fAgents will be 0
	float fAgents = 0;

	if (lengthForceAgents == 1)
	{
		return mPos; // If there's anybody in front of me and i'm not in panic, then i don't move at all....
	}
	if (( lengthForceAgents > 1.9) || (lengthForceAgents < 0.001))
	{
		if (mTrait.GetPanicLevel() > 5)
			fAgents = 10; //15; //20; //1;//6;	//15;
		else
			fAgents = 4;//5; //10; //15; // 1;//3;	//4; 

		if (myControl->glWin->RENDER2D)
		{
			// FOR DEBUGGING PURPOSES WE RENDER THE AVOIDANCE FORCES WITH OTHER AGENTS
			glBegin(GL_LINES);
			glColor3f(1.0, 1.0, 0.3);
			glVertex3f(mPos.x, mPos.y, mPos.z);
			glVertex3f(mPos.x+fOtherAgents.x*0.5, mPos.y+fOtherAgents.y*0.5, mPos.z+fOtherAgents.z*0.5);
			glEnd();	
		}
	}
	
	// if lengthForceAgents < 1 it means we are being pushed, therefore, a pushing force of 0.8 or higher, i'll consider it
	// strong enough to affect my equilibrium...
	//funda
	//if ((lengthForceAgents < 1) && (lengthForceAgents>0.1125) && (rand()%70 > 25))
	//if ((lengthForceAgents>0.1125) && (rand()%70 > 25))
	//	mTrait.IncreaseImbalance(1);
	//else 
	//	mTrait.SetImbalance(0);	// set a 0 if i haven't been pushed

	//cout<<mTrait.GetImbalance()<<endl;

	float fWeightWalls = 0;
	float lengthForceWalls = LENGTH(fWalls.x, fWalls.z);
	if ((lengthForceWalls > 0.95) || (lengthForceWalls < 0.00001))
	{
		fWeightWalls = 25; 
		if (myControl->glWin->RENDER2D)
		{
			glBegin(GL_LINES);
			glColor3f(1.0, 0.7, 0.7);
			glVertex3f(mPos.x, mPos.y, mPos.z);
			glVertex3f(mPos.x+fWalls.x, mPos.y+fWalls.y, mPos.z+fWalls.z);
			glEnd();	
		}
	}
	float fWeightObst = 0;
	float lengthForceObst = LENGTH(fObst.x, fObst.z);
	if ((lengthForceObst > 0.95) || (lengthForceObst < 0.00001))
	{
		fWeightObst = 15;
		if (myControl->glWin->RENDER2D)
		{
			glBegin(GL_LINES);
			glColor3f(0.4, 1.0, 1.0);
			glVertex3f(mPos.x, mPos.y, mPos.z);
			glVertex3f(mPos.x+fObst.x, mPos.y+fObst.y, mPos.z+fObst.z);
			glEnd();	
		}
	}

	weight_friendship = 2;
	//fTotal = fAttrac + fWalls*fWeightWalls*0 + fObst + fDesiredSpeed + fLook4Walls*0 + fOtherAgents*fAgents; 
	//fTotal = fAttrac*fWeightAttrac + fWalls*fWeightWalls + fObst*fWeightObst + fDesiredSpeed + fLook4Walls*0 + fOtherAgents*fAgents; 
	fTotal = fAttrac*fWeightAttrac + fWalls*fWeightWalls + fObst*fWeightObst + fDesiredSpeed + fOtherAgents*fAgents + 
			 weight_friendship* f_friendship + weight_lynching *f_lynching; 
	
	normalize(&fTotal);
	
	if (fAgents == 0) // there is intersection with an agent
	{

		//glPointSize(6);
		//glBegin(GL_POINTS);
		//glColor3f(1,1,1);
		//glVertex3f(mPos.x, mPos.y+0.5, mPos.z);
		//glEnd();

		// If the pushing force is against my desired direction, then i'll stop trying to move for a little bit:
		tVector unitForceAgents = fOtherAgents/lengthForceAgents;
		float dotProd = unitForceAgents.x*m_desiredVel.x + unitForceAgents.z*m_desiredVel.z;
		
		if (myControl->glWin->RENDER2D)
		{
			if ( dotProd < -0.35)
			{
				if (dotProd < -0.9)
					glColor3f(0.0, 0.0, 0.0);
				else if (dotProd < -0.8)
					glColor3f(0.0, 1.0, 0.0);
				else
					glColor3f(0.0, 0.0, 1.0);

				//DRAW STUFF
				/* 
				glBegin(GL_LINES);
				glVertex3f(mPos.x, mPos.y, mPos.z);
				glVertex3f(mPos.x+m_desiredVel.x, mPos.y+m_desiredVel.y, mPos.z+m_desiredVel.z);
				glColor3f(1.0, 0.0, 0.0); 
				glVertex3f(mPos.x, mPos.y, mPos.z);
				glVertex3f(mPos.x+unitForceAgents.x, mPos.y+unitForceAgents.y, mPos.z+unitForceAgents.z);
				glEnd();
				*/
			}
		}

		// Depending on the direction of the repulsion forces between agents, i'll make the person temporaly stop
		if ( dotProd < -0.9)
		{
			m_stopShaking = rand()%30;//20; 
			//DRAW STUFF
		//	glLineWidth(2);
		//	glBegin(GL_LINES);
		//	glColor3f(1.0, 0.0, 0.0);
		//	glVertex3f(mPos.x, mPos.y, mPos.z);
		//	glVertex3f(mPos.x+unitForceAgents.x, mPos.y+unitForceAgents.y, mPos.z+unitForceAgents.z);
		//	glColor3f(0.0, 0.0, 0.0);
		//	glVertex3f(mPos.x, mPos.y, mPos.z);
		//	glVertex3f(mPos.x+m_desiredVel.x, mPos.y+m_desiredVel.y, mPos.z+m_desiredVel.z);
		//	glEnd();
		}
		else if (dotProd < -0.7) //-0.5) //-0.8)
		{
			m_stopShaking = rand()%20;//15;
		}
		else if (dotProd <  -0.4) //0)//-0.5)//-0.35)
		{
			m_stopShaking = rand()%10;//10;
		}
		else 
		{
			if (m_stopShaking > 7)
				m_stopShaking = 7;
			else
				m_stopShaking--; // = -1;
		}
		m_stopShaking -= mTrait.GetPanicLevel();

		// If the walls are far away, then avoid intersection with other agents
		if (fWeightWalls > 0)
		{
			step.x = mPos.x + m_dt*speed*fTotal.x*1.0*0 + fOtherAgents.x; //*(0.80);//0.8;//*0.2;//OFFSET_HUMS;
			step.y = mPos.y;
			step.z = mPos.z + m_dt*speed*fTotal.z*1.0*0 + fOtherAgents.z; //*(0.80);//0.8;//*0.2;//OFFSET_HUMS;
		}
		else  // else avoid intersection with the walls always
		{
			step.x = mPos.x + m_dt*speed*fTotal.x*0.8*0 + fWalls.x*(0.8) + fOtherAgents.x*(0.2);//*0.2;//OFFSET_HUMS;
			step.y = mPos.y;
			step.z = mPos.z + m_dt*speed*fTotal.z*0.8*0 + fWalls.z*(0.8) + fOtherAgents.x*(0.2);//*0.2;//OFFSET_HUMS;
		}
	}
	else // there is no intersection with an agent
	{
		if (fWeightWalls == 0)  // if there is intersection with the walls
		{
			step.x = mPos.x + m_dt*speed*fTotal.x*0.5*0 + fWalls.x; //*(0.8); // NURIA 21MAY07
			step.y = mPos.y;
			step.z = mPos.z + m_dt*speed*fTotal.z*0.5*0 + fWalls.z; //*(0.8); // NURIA 21MAY07

			//glPointSize(6);
			//glBegin(GL_POINTS);
			//glColor3f(1,0,0);
			//glVertex3f(mPos.x, mPos.y+0.5, mPos.z);
			//glEnd();
		}
		else // no intersection with walls and no intersection with obstacles
		{
			step.x = mPos.x + m_dt*speed*fTotal.x;
			step.y = mPos.y;
			step.z = mPos.z + m_dt*speed*fTotal.z;

			//glPointSize(6);
			//glBegin(GL_POINTS);
			//glColor3f(0,0,0);
			//glVertex3f(mPos.x, mPos.y+0.5, mPos.z);
			//glEnd();

		}
		m_stopShaking = -1;
	}


	float agentsPushing = 0;

	if (fWeightObst == 0)// Avoid intersection with obstacles!
	{
		// if walls are far away:
		if (fWeightWalls > 0)
		{
			if (fAgents == 0)
				agentsPushing = 1;
	
			/*
			step.x = mPos.x + m_dt*speed*fTotal.x*0.1 + fObst.x*(0.1);
			step.y = mPos.y;
			step.z = mPos.z + m_dt*speed*fTotal.z*0.1 + fObst.z*(0.1);
			*/
			// We need to multiply the fObst by 5, because for this type of intersection, the length of the
			// vector returned is 1/5 of the unit
			step.x = mPos.x + m_dt*speed*(fTotal.x*0.1*0 +5*fObst.x*0.9) + agentsPushing*fOtherAgents.x*(0.3);; // + dt*speed*fObst.x*(0.9);
			step.y = mPos.y;
			step.z = mPos.z + m_dt*speed*(fTotal.z*0.1*0 +5*fObst.z*0.9) + agentsPushing*fOtherAgents.z*(0.3);; // + dt*speed*fObst.z*(0.9);

			//glPointSize(6);
			//glBegin(GL_POINTS);
			//glColor3f(0,1,0);
			//glVertex3f(mPos.x, mPos.y+0.5, mPos.z);
			//glEnd();

		}
	}

	// If there are not walls or obstacles, then try to avoid the obstacles that can be walk over or through
	if ((fWeightObst>0) && (fWeightWalls > 0) && ((length(fAvoidFallenAgents) > 0)||(length(fAvoidCircles)>0)))
	{
		if (fAgents == 0)
			agentsPushing = 1;

		
	
		//cout << "IN" << fWeightObst << " " << fWeightWalls << " " << length(fAvoidFallenAgents) <<" "<<length(fOtherAgents)<<endl;
		step.x = mPos.x + m_dt*speed*fTotal.x*0.35 + fAvoidFallenAgents.x*(0.01) + fAvoidCircles.x*0.01 + agentsPushing*fOtherAgents.x*(0.7);
		step.y = mPos.y;
		step.z = mPos.z + m_dt*speed*fTotal.z*0.35 + fAvoidFallenAgents.z*(0.01) + fAvoidCircles.z*0.01 + agentsPushing*fOtherAgents.x*(0.7);

		//glPointSize(6);
		//glBegin(GL_POINTS);
		//glColor3f(0,0,1);
		//glVertex3f(mPos.x, mPos.y+0.5, mPos.z);
		//glEnd();

	}



	m_stopShaking--;
	
	// Limit how much the desiredVel vector can change!
	tVector dirMovement;
	dirMovement = mPos - m_prevpos;
	float distStep = normalize(&dirMovement);
	//distStep = LENGTH(dirMovement.x,dirMovement.z);
	//if (distStep > 0) 
	//	dirMovement = dirMovement/distStep;

	aux_velocity = m_desiredVel;
	aux_orientation = m_orientation;

	m_desiredVel = m_desiredVel*2  + dirMovement + fAttrac;
	m_desiredVel.y = 0;
	dist2 = normalize(&m_desiredVel);

	// The orientation should change less abruptly
	m_orientation = m_orientation*26 + m_desiredVel; // OJO CAMBIOS SIGGRAPH

	normalize(&m_orientation);
	
	fDesiredSpeed = m_desiredVel;
//	calculateLookAheadEnds();
	
	if ((AVOID_SHAKING) && (m_stopShaking>-1 ))


	{
		if (myControl->glWin->RENDER2D)
		{
			if (m_stopShaking>29)
				glColor3f(0.0, 0.0, 0.0);
			else if (m_stopShaking>14)
				glColor3f(0.0, 1.0, 0.0);
			else
				glColor3f(0.0, 0.0, 1.0);

			//glPointSize(10.0);
			//glBegin(GL_POINTS);
			//glVertex3f(mPos.x+0.15, mPos.y, mPos.z);
			//glEnd();
		}

		if (lengthForceAgents > 0.1)
		{
			if ((fWeightWalls > 0) || (lengthForceWalls < 0.01))// If the walls are far away, then avoid intersection with other agents
			{
				step.x = mPos.x + fOtherAgents.x*(0.20);//0.8;//*0.2;//OFFSET_HUMS;
				step.y = mPos.y;
				step.z = mPos.z + fOtherAgents.z*(0.20);//0.8;//*0.2;//OFFSET_HUMS;
			}
			else  // else avoid intersection with the walls always
			{
				//step.x = mPos.x + m_dt*speed*fTotal.x + fWalls.x*(0.5) + fOtherAgents.x*(0.2);//*0.2;//OFFSET_HUMS;
				//step.y = mPos.y;
				//step.z = mPos.z + m_dt*speed*fTotal.z + fWalls.z*(0.5) + fOtherAgents.x*(0.2);//*0.2;//OFFSET_HUMS;
				// Nuevo 5 April 06, I'm making the walls forces weaker, since the simualtion step is very small:
				step.x = mPos.x + fWalls.x*(0.2);// + fOtherAgents.x*(0.2);//*0.2;//OFFSET_HUMS;
				step.y = mPos.y;
				step.z = mPos.z + fWalls.z*(0.2);// + fOtherAgents.x*(0.2);//*0.2;//OFFSET_HUMS;
			}
		}
		else
		{
			step = mPos;
		}
	}

	
	#ifdef MYDEBUG
	{
		//	DRAW DEBUG FORCES
/*		glBegin(GL_POINTS);
			glColor3f(0.0, 0.0, 0.0);
			glVertex3f(mPos.x, 1.0, mPos.z);	
		glEnd();
		glBegin(GL_LINES);
			glColor3f(1.0, 1.0, 0.0);
			glVertex3f(mPos.x, 1.0, mPos.z);
			tVector deb_force = mPos + fOtherAgents*20;
			glVertex3f(deb_force.x, 1.0, deb_force.z);
		glEnd();
		//  END DEBUG FORCES
*/
	}
	#endif

	num_step = (num_step + 1) % NUM_tVectorS;


	
	if (m_nextTask)
	{
		glColor3f(m_color[0],m_color[1],m_color[2]);
		glBegin(GL_POINTS);
		glScalef(5,5,5);
		glVertex3f(m_tasks.front().location.x,m_tasks.front().location.y+0.1,m_tasks.front().location.z); 
		glEnd();
			
		if (dist < m_tasks.front().proximity)
		if (!m_doTask)
		{
			// Jan working something out: I'm automatically setting the duration of the task based on the
			// length of the animation clip * times the "speed" of the computer.
			//m_tasks.front().duration = m_pcal3dAgent->getCalCoreModel()->getCoreAnimation(3)->getDuration(7)*460;
			list<int>::iterator iter_lactions;
			int actionNum;
			m_tasks.front().duration = 100; //1000;//5000;
			// museum 	m_tasks.front().duration = 240000; //do the task even if not cal3d 
			for (iter_lactions = m_tasks.front().laction.begin(); 
				iter_lactions!=m_tasks.front().laction.end();
				iter_lactions++)
			{
				actionNum = *iter_lactions;
				if (m_pcal3dAgent != NULL)
					m_tasks.front().duration = m_tasks.front().duration + m_pcal3dAgent->getCalCoreModel()->getCoreAnimation(actionNum)->getDuration()*460;
				else
					m_tasks.front().duration = m_tasks.front().duration ;
				//funda std::cout << "Action number = " << actionNum << std::endl;
				//funda std::cout << "Duration of task = " << m_tasks.front().duration << std::endl;
			}
			
			m_tasks.front().timer = m_tasks.front().duration; // Initiate the timer to the duration of the action
			m_doTask = true;	// will be true while the task is being performed, and thus we know we have to decrease the timer
			myControl->glWin->execute_idle=true;   // will be true when the cal3d action needs to be started and then it will be set to false immediately
			m_execute_idle=true;
			//cout << "ACCION";
			m_nextTask=false;
		}
	}
	if (m_doTask)
	{
		step = mPos;
		m_desiredVel = aux_velocity;
		m_orientation = aux_orientation;
	}
	else if (!m_nextTask)
	{
		switch (m_crossingState)
		{
			case NOT_CROSSING:
				if (dist < 1.25*SIZE_CELL)
					m_crossingState = APPROACHING_A;
				
				break;
			case APPROACHING_A:
				m_pprevCell->increaseNumPeopleCrossing(m_nextPortal, mPos);
				if (dist < 0.9*SIZE_CELL)
				{
					m_crossingState = REACHING_A;
					//m_building->printListHums(m_nextPortal);
					m_building->insertHum(m_nextPortal,this);
					//m_building->printListHums(m_nextPortal);
					
					//m_portalIDcoll = m_nextPortal;
					// Add the human to the list of humans trying to cross this portal
			//		m_pcell->increaseNumPeopleCrossing(m_nextPortal);
			//		m_tryingToCross = true;
				}
				break;
			case REACHING_A:
				m_pprevCell->increaseNumPeopleCrossing(m_nextPortal, mPos);
				if (dist < 0.6*SIZE_CELL)
				{
					m_portalIDcoll = m_nextPortal;
					m_crossingState = CLOSE_TO_A;
					m_crossingPortal = true;
					if (m_nextPortal > 0)
					{		
						pos = m_pcell->getPortal(m_nextPortal).pos;
						// I need to keep track of the attractor previous to crossing the door, so that i know exactly when the door
						// is crossed, which will be when the distance to the new m_attractor is smaller than the distance to the 
						// previous one m_prevAttractor.
						m_prevAttractor = m_attractor;
						m_attractor.x = pos[0];
						m_attractor.y = pos[1];
						m_attractor.z = pos[2];	
					}
				}
				break;
			case CLOSE_TO_A:
				m_pprevCell->increaseNumPeopleCrossing(m_nextPortal, mPos);
				
				distPrevAtt = length(m_prevAttractor-mPos);
				// I want to find out the exact moment in which the door is crossed
				//if (dist < 0.4*SIZE_CELL)
				if (dist <= distPrevAtt) // Needs to be <= cause otherwise it doesn't work in the exit, since they will be the same thing...
				{
					//cout << " door crossed: " << myControl->glWin->globalTimeMillisecs << endl;
					m_crossingState = LEAVING_B;
					m_doorCrossed = true;		// The human will chooseNextCell when the door has finally been crossed
					m_doorCrossedFR = true;		// This is used to count the number of people crossing to calculate flow rates during the validation
					
					// This was commented out, because i could observe people suddenly dissapearing even though they haven't found
					// an exit. I'm not sure why i ever made this case special... 			
					if (m_pcell == m_pprevCell) // it means we have reached an exit
					{
						int numExits = m_building->getNumExits();
						for (int i=0; i<numExits; i++)
						{
							if (m_pcell->getPath(i).length==0)
							{
								m_reached_goal = true;
								//cout << "hum: " <<m_humID<< "removed in cell: " << m_myCell << endl;
								exitCell();
								m_portalIDcoll = -1;   // -1 means that we are not crossing a portal anymore, and therefore collision detection with those humans is not necessary
								m_building->removeHum(m_nextPortal,this);
							}
						}
					}

					num_step = 0; 
					m_crossingPortal = false;
					m_tryingToCross = false;
					m_lastAttracPos = m_attractor; // We need to save this information, because in ChooseNextCell will modify the attractor
  					m_lastAttracID   = m_nextPortal;

					// Here it counts as one more person crossed the portal
					m_building->increasePeopleCrossed(m_nextPortal);
				}
				break;
			case LEAVING_B: // When we are far away from B then we can remove the human from the list of humans crossing that is used for collision detection purposes
				//if ((LENGTH((mPos.x-m_lastAttracPos.x),(mPos.z-m_lastAttracPos.z)) > 0.8*SIZE_CELL) || //> 0.6*SIZE_CELL) ||
				//	(dist < 0.7*SIZE_CELL)) // OR is necessary in case the next attractor is closer than the distance necessary to leave B			
				
				distPrevAtt = length(m_prevAttractor-mPos);
				distLastAtt = length(m_lastAttracPos-mPos);
				if (((distLastAtt > 0.8*SIZE_CELL) && (distPrevAtt>distLastAtt)) || //> 0.6*SIZE_CELL) ||
					(dist < 0.7*SIZE_CELL)) // OR is necessary in case the next attractor is closer than the distance necessary to leave B			
				{
					m_crossingState = NOT_CROSSING;
					m_portalIDcoll = -1;   // -1 means that we are not crossing a portal anymore, and therefore collision detection with those humans is not necessary
					m_building->removeHum(m_lastAttracID,this);
				}
				break;
		};
	}

/*
	// Check the distance to the attractor point, and when it's very close, we'll change m_myCell
	// If we are close to the portal, we'll increase the list of people trying to cross this portal
	if ((dist < SIZE_CELL) && !(m_tryingToCross))
	{
//		m_pcell->increaseNumPeopleCrossing(m_nextPortal);
		m_tryingToCross = true;
	}
	// We have two attractors, one before the portal and another one right after the portal, so when the 
	// first one is crossed we set crossingPortal to true, and when the second attractor is approached
	// it means we are already in the next cell
	if ((dist < 0.6*SIZE_CELL) && !(m_crossingPortal))
	{
		m_crossingPortal = true;
		if (m_nextPortal > 0)
		{
			float* pos = m_pcell->getPortal(m_nextPortal).pos;
			m_attractor.x = pos[0];
			m_attractor.y = pos[1];
			m_attractor.z = pos[2];
		}
	}
	else if ((dist < 0.4*SIZE_CELL) && (m_crossingPortal)) // Distance to attractor
	{
		m_doorCrossed = true;
		num_step = 0;
		m_crossingPortal = false;
		m_tryingToCross = false;
//		m_pcell->getPortal(m_nextPortal).numPeopleCrossing--;
	}
*/
	return step;
}






tVector Chumanoid::calculateStepsSocialForces()
{
	tVector step;
	float dist, dist2;
	float distPrevAtt, distLastAtt; 
	tVector fTotal, fAttrac, fWalls, fObst = {0.0,0.0,0.0};
	tVector fDesiredSpeed = {0.0,0.0,0.0};
	tVector fLook4Walls = {0.0, 0.0, 0.0};
	tVector fOtherAgents = {0.0, 0.0, 0.0};
	tVector fOthers0 = {0.0, 0.0, 0.0};
	tVector fOthers1 = {0.0, 0.0, 0.0};
	double speed;
	float fWeightAttrac;
	double* pos;
	tVector fAvoidFallenAgents;
	float fFallenAgent;
	tVector fAvoidCircles;

	unsigned long currTime;
	static unsigned long incrTime=0;
	static unsigned long frameRate=0;
	static unsigned long prevTime=0;
	
	//cout << "Social Forces Model"<<endl;

	_timeb tm;	
	_ftime(&tm);
	currTime = tm.time*1000 + tm.millitm; 
	incrTime += (currTime - prevTime);

	// Accelerate
	if (m_speed < m_maxSpeed)
	{
		m_speed = m_speed + 0.000001*m_dt;
	}
	else
		m_speed = m_maxSpeed;

	speed = m_speed;

	// //Decelerate
	if (m_densityAhead > 0.8)
		if (m_speed > 0.0001)
			m_speed = m_speed - 0.000001*m_dt;
		else
			m_speed = 0;
	
	#ifdef REAL_TIME
	//	_timeb tm;
	//	_ftime(&tm);
	//	currTime = tm.time*1000 + tm.millitm; 
		m_dt = currTime - prevTime; 
		//printf("dt = %f",dt);
	//	m_time = currTime ;
	#else
		m_dt = dif_millSecs; //200;
	#endif

	prevTime = currTime ;

	// We reduce the speed of the human when they are going downstairs
	sSTAIR stair;
	if (m_pprevCell->getIsStair(&stair))
	{
		speed = speed * 0.8; //0.57;
	}

	if ((!m_nextTask) && (!m_doTask) && (m_crossingState<0) && (!m_tasks.empty()))
		if ((m_tasks.front()).roomID == m_prevCell) // OJO antes era prev!!!!
			m_nextTask=true;
		
	if (m_nextTask)
		fAttrac = m_tasks.front().location - mPos;
	else 
		fAttrac = m_attractor - mPos;

	dist = LENGTH(fAttrac.x,fAttrac.z);
	if (dist  > 0)
		fAttrac = fAttrac / dist;
	if (dist < 1.5)
		fWeightAttrac = 35;
	else
		fWeightAttrac = 10;	
	
	fObst = m_pprevCell->RepulsionObstacles(mPos, m_lookAheadA, m_lookAheadB, m_desiredVel,0); // 0 means non-traversable obstacles
	
	// I need repulsion from others in both my current cell and the previous one, to avoid collisions in the portals
	// Repulsion Other Agents will give the repulsion force from other agent that is getting too close, or a vector of length 1
	// if we only need to turn a little bit because there's another agent a bit ahead of us, or a vector of lengh 2 is we have
	// somebody right in the position where we want to move to, and therefore we shouldn't move
	
	glColor4fv(m_color);
	fOtherAgents = m_pprevCell->RepulsionOtherAgents(this, m_lookAheadA, m_lookAheadB, fAttrac,m_portalIDcoll);

	fAvoidFallenAgents = m_pprevCell->tangentialWall(mPos, fAttrac);
	fAvoidCircles = m_pprevCell->RepulsionObstacles(mPos, m_lookAheadA, m_lookAheadB, fAttrac, 1);// false means the obstacles can be traversable

	fWalls = m_pprevCell->RepulsionWalls(mPos, fAttrac);

	// when tangentical forces apply or non forces fAgents will have a value different than 0
	// If repulsion forces need to be applied then fAgents will be 0
	float fAgents = 0;
	float lengthForceAgents = LENGTH(fOtherAgents.x, fOtherAgents.z);
	if (lengthForceAgents == 1)
	{
		return mPos; // If there's anybody in front of me and i'm not in panic, then i don't move at all....
	}
	if (( lengthForceAgents > 1.9) || (lengthForceAgents < 0.001))
	{
		if (mTrait.GetPanicLevel() > 5)
			fAgents = 10; //15; //20; //1;//6;	//15;
		else
			fAgents = 4;//5; //10; //15; // 1;//3;	//4; 

		if (myControl->glWin->RENDER2D)
		{
			// FOR DEBUGGING PURPOSES WE RENDER THE AVOIDANCE FORCES WITH OTHER AGENTS
			glBegin(GL_LINES);
			glColor3f(1.0, 1.0, 0.3);
			glVertex3f(mPos.x, mPos.y, mPos.z);
			glVertex3f(mPos.x+fOtherAgents.x*0.5, mPos.y+fOtherAgents.y*0.5, mPos.z+fOtherAgents.z*0.5);
			glEnd();	
		}
	}
	

	float fWeightWalls = 0;
	float lengthForceWalls = LENGTH(fWalls.x, fWalls.z);
	if ((lengthForceWalls > 0.95) || (lengthForceWalls < 0.00001))
	{
		fWeightWalls = 25; 
		if (myControl->glWin->RENDER2D)
		{
			glBegin(GL_LINES);
			glColor3f(1.0, 0.7, 0.7);
			glVertex3f(mPos.x, mPos.y, mPos.z);
			glVertex3f(mPos.x+fWalls.x, mPos.y+fWalls.y, mPos.z+fWalls.z);
			glEnd();	
		}
	}
	float fWeightObst = 0;
	float lengthForceObst = LENGTH(fObst.x, fObst.z);
	if ((lengthForceObst > 0.95) || (lengthForceObst < 0.00001))
	{
		fWeightObst = 15;
		if (myControl->glWin->RENDER2D)
		{
			glBegin(GL_LINES);
			glColor3f(0.4, 1.0, 1.0);
			glVertex3f(mPos.x, mPos.y, mPos.z);
			glVertex3f(mPos.x+fObst.x, mPos.y+fObst.y, mPos.z+fObst.z);
			glEnd();	
		}
	}

	fTotal = fAttrac*fWeightAttrac + fWalls*fWeightWalls + fObst*fWeightObst + fDesiredSpeed + fOtherAgents*fAgents; 
	normalize(&fTotal);
	
	if (fAgents == 0) // there is intersection with an agent
	{

		//glPointSize(6);
		//glBegin(GL_POINTS);
		//glColor3f(1,1,1);
		//glVertex3f(mPos.x, mPos.y+0.5, mPos.z);
		//glEnd();

		// If the pushing force is against my desired direction, then i'll stop trying to move for a little bit:
		tVector unitForceAgents = fOtherAgents/lengthForceAgents;
		float dotProd = unitForceAgents.x*m_desiredVel.x + unitForceAgents.z*m_desiredVel.z;
		
		if (myControl->glWin->RENDER2D)
		{
			if ( dotProd < -0.35)
			{
				if (dotProd < -0.9)
					glColor3f(0.0, 0.0, 0.0);
				else if (dotProd < -0.8)
					glColor3f(0.0, 1.0, 0.0);
				else
					glColor3f(0.0, 0.0, 1.0);

				//DRAW STUFF
				/* 
				glBegin(GL_LINES);
				glVertex3f(mPos.x, mPos.y, mPos.z);
				glVertex3f(mPos.x+m_desiredVel.x, mPos.y+m_desiredVel.y, mPos.z+m_desiredVel.z);
				glColor3f(1.0, 0.0, 0.0); 
				glVertex3f(mPos.x, mPos.y, mPos.z);
				glVertex3f(mPos.x+unitForceAgents.x, mPos.y+unitForceAgents.y, mPos.z+unitForceAgents.z);
				glEnd();
				*/
			}
		}


		// If the walls are far away, then avoid intersection with other agents
		if (fWeightWalls > 0)
		{
			step.x = mPos.x + m_dt*speed*fTotal.x*1.0*0 + fOtherAgents.x; //*(0.80);//0.8;//*0.2;//OFFSET_HUMS;
			step.y = mPos.y;
			step.z = mPos.z + m_dt*speed*fTotal.z*1.0*0 + fOtherAgents.z; //*(0.80);//0.8;//*0.2;//OFFSET_HUMS;
		}
		else  // else avoid intersection with the walls always
		{
			step.x = mPos.x + m_dt*speed*fTotal.x*0.8*0 + fWalls.x*(0.8) + fOtherAgents.x*(0.2);//*0.2;//OFFSET_HUMS;
			step.y = mPos.y;
			step.z = mPos.z + m_dt*speed*fTotal.z*0.8*0 + fWalls.z*(0.8) + fOtherAgents.x*(0.2);//*0.2;//OFFSET_HUMS;
		}
	}
	else // there is no intersection with an agent
	{
		if (fWeightWalls == 0)  // if there is intersection with the walls
		{
			step.x = mPos.x + m_dt*speed*fTotal.x*0.5*0 + fWalls.x; //*(0.8); // NURIA 21MAY07
			step.y = mPos.y;
			step.z = mPos.z + m_dt*speed*fTotal.z*0.5*0 + fWalls.z; //*(0.8); // NURIA 21MAY07

			//glPointSize(6);
			//glBegin(GL_POINTS);
			//glColor3f(1,0,0);
			//glVertex3f(mPos.x, mPos.y+0.5, mPos.z);
			//glEnd();
		}
		else // no intersection with walls and no intersection with obstacles
		{
			step.x = mPos.x + m_dt*speed*fTotal.x;
			step.y = mPos.y;
			step.z = mPos.z + m_dt*speed*fTotal.z;

			//glPointSize(6);
			//glBegin(GL_POINTS);
			//glColor3f(0,0,0);
			//glVertex3f(mPos.x, mPos.y+0.5, mPos.z);
			//glEnd();

		}
	}


	float agentsPushing = 0;
	if (fWeightObst == 0)// Avoid intersection with obstacles!
	{
		// if walls are far away:
		if (fWeightWalls > 0)
		{
			if (fAgents == 0)
				agentsPushing = 1;
	
			step.x = mPos.x + m_dt*speed*(fTotal.x*0.1*0 +5*fObst.x*0.9) + agentsPushing*fOtherAgents.x*(0.3);; // + dt*speed*fObst.x*(0.9);
			step.y = mPos.y;
			step.z = mPos.z + m_dt*speed*(fTotal.z*0.1*0 +5*fObst.z*0.9) + agentsPushing*fOtherAgents.z*(0.3);; // + dt*speed*fObst.z*(0.9);

			//glPointSize(6);
			//glBegin(GL_POINTS);
			//glColor3f(0,1,0);
			//glVertex3f(mPos.x, mPos.y+0.5, mPos.z);
			//glEnd();

		}
	}

	// If there are not walls or obstacles, then try to avoid the obstacles that can be walk over or through
	if ((fWeightObst>0) && (fWeightWalls > 0) && ((length(fAvoidFallenAgents) > 0)||(length(fAvoidCircles)>0)))
	{
		if (fAgents == 0)
			agentsPushing = 1;
	
		//cout << "IN" << fWeightObst << " " << fWeightWalls << " " << length(fAvoidFallenAgents) <<" "<<length(fOtherAgents)<<endl;
		step.x = mPos.x + m_dt*speed*fTotal.x*0.35 + fAvoidFallenAgents.x*(0.01) + fAvoidCircles.x*0.01 + agentsPushing*fOtherAgents.x*(0.7);
		step.y = mPos.y;
		step.z = mPos.z + m_dt*speed*fTotal.z*0.35 + fAvoidFallenAgents.z*(0.01) + fAvoidCircles.z*0.01 + agentsPushing*fOtherAgents.x*(0.7);

		//glPointSize(6);
		//glBegin(GL_POINTS);
		//glColor3f(0,0,1);
		//glVertex3f(mPos.x, mPos.y+0.5, mPos.z);
		//glEnd();

	}

	// Limit how much the desiredVel vector can change!
	tVector dirMovement;
	dirMovement = mPos - m_prevpos;
	float distStep = normalize(&dirMovement);

	m_desiredVel = m_desiredVel*2  + dirMovement + fAttrac;
	m_desiredVel.y = 0;
	dist2 = normalize(&m_desiredVel);

	// The orientation should change less abruptly
	m_orientation = m_orientation*26 + m_desiredVel; // OJO CAMBIOS SIGGRAPH

	normalize(&m_orientation);

	fDesiredSpeed = m_desiredVel;
	
	#ifdef MYDEBUG
	{
		//	DRAW DEBUG FORCES
/*		glBegin(GL_POINTS);
			glColor3f(0.0, 0.0, 0.0);
			glVertex3f(mPos.x, 1.0, mPos.z);	
		glEnd();
		glBegin(GL_LINES);
			glColor3f(1.0, 1.0, 0.0);
			glVertex3f(mPos.x, 1.0, mPos.z);
			tVector deb_force = mPos + fOtherAgents*20;
			glVertex3f(deb_force.x, 1.0, deb_force.z);
		glEnd();
		//  END DEBUG FORCES
*/
	}
	#endif

	num_step = (num_step + 1) % NUM_tVectorS;
	
	if (m_nextTask)
	{
		glColor3f(m_color[0],m_color[1],m_color[2]);
		glBegin(GL_POINTS);
		glVertex3f(m_tasks.front().location.x,m_tasks.front().location.y+0.1,m_tasks.front().location.z); 
		glEnd();
			
		if (dist < m_tasks.front().proximity)
		if (!m_doTask)
		{
			m_tasks.front().timer = m_tasks.front().duration; // Initiate the timer to the duration of the action
			m_doTask = true;	// will be true while the task is being performed, and thus we know we have to decrease the timer
			myControl->glWin->execute_idle=true;   // will be true when the cal3d action needs to be started and then it will be set to false immediately
			m_execute_idle=true;
			//cout << "ACCION";
			m_nextTask=false;
		}
	}
	if (m_doTask)
	{
		step = mPos;
	}
	else if (!m_nextTask)
	{
		switch (m_crossingState)
		{
			case NOT_CROSSING:
				if (dist < 1.25*SIZE_CELL)
					m_crossingState = APPROACHING_A;
				
				break;
			case APPROACHING_A:
				m_pprevCell->increaseNumPeopleCrossing(m_nextPortal, mPos);
				if (dist < 0.9*SIZE_CELL)
				{
					m_crossingState = REACHING_A;
					//m_building->printListHums(m_nextPortal);
					m_building->insertHum(m_nextPortal,this);
					//m_building->printListHums(m_nextPortal);
					
					//m_portalIDcoll = m_nextPortal;
					// Add the human to the list of humans trying to cross this portal
			//		m_pcell->increaseNumPeopleCrossing(m_nextPortal);
			//		m_tryingToCross = true;
				}
				break;
			case REACHING_A:
				m_pprevCell->increaseNumPeopleCrossing(m_nextPortal, mPos);
				if (dist < 0.6*SIZE_CELL)
				{
					m_portalIDcoll = m_nextPortal;
					m_crossingState = CLOSE_TO_A;
					m_crossingPortal = true;
					if (m_nextPortal > 0)
					{		
						pos = m_pcell->getPortal(m_nextPortal).pos;
						// I need to keep track of the attractor previous to crossing the door, so that i know exactly when the door
						// is crossed, which will be when the distance to the new m_attractor is smaller than the distance to the 
						// previous one m_prevAttractor.
						m_prevAttractor = m_attractor;
						m_attractor.x = pos[0];
						m_attractor.y = pos[1];
						m_attractor.z = pos[2];	
					}
				}
				break;
			case CLOSE_TO_A:
				m_pprevCell->increaseNumPeopleCrossing(m_nextPortal, mPos);
				
				distPrevAtt = length(m_prevAttractor-mPos);
				// I want to find out the exact moment in which the door is crossed
				//if (dist < 0.4*SIZE_CELL)
				if (dist <= distPrevAtt) // Needs to be <= cause otherwise it doesn't work in the exit, since they will be the same thing...
				{
					//cout << " door crossed: " << myControl->glWin->globalTimeMillisecs << endl;
					m_crossingState = LEAVING_B;
					m_doorCrossed = true;		// The human will chooseNextCell when the door has finally been crossed
					m_doorCrossedFR = true;		// This is used to count the number of people crossing to calculate flow rates during the validation
					
					// This was commented out, because i could observe people suddenly dissapearing even though they haven't found
					// an exit. I'm not sure why i ever made this case special... 			
					if (m_pcell == m_pprevCell) // it means we have reached an exit
					{
						int numExits = m_building->getNumExits();
						for (int i=0; i<numExits; i++)
						{
							if (m_pcell->getPath(i).length==0)
							{
								m_reached_goal = true;
								//cout << "hum: " <<m_humID<< "removed in cell: " << m_myCell << endl;
								exitCell();
								m_portalIDcoll = -1;   // -1 means that we are not crossing a portal anymore, and therefore collision detection with those humans is not necessary
								m_building->removeHum(m_nextPortal,this);
							}
						}
					}

					num_step = 0; 
					m_crossingPortal = false;
					m_tryingToCross = false;
					m_lastAttracPos = m_attractor; // We need to save this information, because in ChooseNextCell will modify the attractor
  					m_lastAttracID   = m_nextPortal;

					// Here it counts as one more person crossed the portal
					m_building->increasePeopleCrossed(m_nextPortal);
				}
				break;
			case LEAVING_B: // When we are far away from B then we can remove the human from the list of humans crossing that is used for collision detection purposes
				//if ((LENGTH((mPos.x-m_lastAttracPos.x),(mPos.z-m_lastAttracPos.z)) > 0.8*SIZE_CELL) || //> 0.6*SIZE_CELL) ||
				//	(dist < 0.7*SIZE_CELL)) // OR is necessary in case the next attractor is closer than the distance necessary to leave B			
				
				distPrevAtt = length(m_prevAttractor-mPos);
				distLastAtt = length(m_lastAttracPos-mPos);
				if (((distLastAtt > 0.8*SIZE_CELL) && (distPrevAtt>distLastAtt)) || //> 0.6*SIZE_CELL) ||
					(dist < 0.7*SIZE_CELL)) // OR is necessary in case the next attractor is closer than the distance necessary to leave B			
				{
					m_crossingState = NOT_CROSSING;
					m_portalIDcoll = -1;   // -1 means that we are not crossing a portal anymore, and therefore collision detection with those humans is not necessary
					m_building->removeHum(m_lastAttracID,this);
				}
				break;
		};
	}
	return step;
}



tVector Chumanoid::calculateStepsRuleBased()
{
	tVector step;
	float dist, dist2;
	float distPrevAtt, distLastAtt; 
	tVector fTotal, fAttrac, fWalls, fObst = {0.0,0.0,0.0};
	tVector fDesiredSpeed = {0.0,0.0,0.0};
	tVector fLook4Walls = {0.0, 0.0, 0.0};
	tVector fOtherAgents = {0.0, 0.0, 0.0};
	tVector fOthers0 = {0.0, 0.0, 0.0};
	tVector fOthers1 = {0.0, 0.0, 0.0};
	double speed;
	float fWeightAttrac;
	double* pos;
//	tVector fAvoidFallenAgents;
	float fFallenAgent;
	tVector fAvoidCircles;

	unsigned long currTime;
	static unsigned long incrTime=0;
	static unsigned long frameRate=0;
	static unsigned long prevTime=0;
	
	//cout << "Rule Based Model"<<endl;

	_timeb tm;	
	_ftime(&tm);
	currTime = tm.time*1000 + tm.millitm; 
	incrTime += (currTime - prevTime);

	// Accelerate
	if (m_speed < m_maxSpeed)
	{
		m_speed = m_speed + 0.000001*m_dt;
	}
	else
		m_speed = m_maxSpeed;

	speed = m_speed;

	// //Decelerate
	if (m_densityAhead > 0.8)
		if (m_speed > 0.0001)
			m_speed = m_speed - 0.000001*m_dt;
		else
			m_speed = 0;
	
	#ifdef REAL_TIME
	//	_timeb tm;
	//	_ftime(&tm);
	//	currTime = tm.time*1000 + tm.millitm; 
		m_dt = currTime - prevTime; 
		//printf("dt = %f",dt);
	//	m_time = currTime ;
	#else
		m_dt = dif_millSecs; //200;
	#endif

	prevTime = currTime ;

	// We reduce the speed of the human when they are going downstairs
	sSTAIR stair;
	if (m_pprevCell->getIsStair(&stair))
	{
		speed = speed * 0.8; //0.57;
	}

	if ((!m_nextTask) && (!m_doTask) && (m_crossingState<0) && (!m_tasks.empty()))
		if ((m_tasks.front()).roomID == m_prevCell) // OJO antes era prev!!!!
			m_nextTask=true;
		
	if (m_nextTask)
		fAttrac = m_tasks.front().location - mPos;
	else 
		fAttrac = m_attractor - mPos;

	dist = LENGTH(fAttrac.x,fAttrac.z);
	if (dist  > 0)
		fAttrac = fAttrac / dist;
	if (dist < 1.5)
		fWeightAttrac = 35;
	else
		fWeightAttrac = 10;	
	
	fObst = m_pprevCell->OnlyAvoidanceObstacles(mPos, m_desiredVel); // non-traversable obstacles
	
	// I need repulsion from others in both my current cell and the previous one, to avoid collisions in the portals
	// Repulsion Other Agents will give the repulsion force from other agent that is getting too close, or a vector of length 1
	// if we only need to turn a little bit because there's another agent a bit ahead of us, or a vector of lengh 2 is we have
	// somebody right in the position where we want to move to, and therefore we shouldn't move
	
	glColor4fv(m_color);
	fOtherAgents = m_pprevCell->OnlyAvoidanceOtherAgents(this, m_lookAheadA, m_lookAheadB, fAttrac,m_portalIDcoll);

	fWalls = m_pprevCell->OnlyAvoidanceWalls(mPos, fAttrac);
	
	float fAgents = 8;//4;

		if (myControl->glWin->RENDER2D)
		{
			// FOR DEBUGGING PURPOSES WE RENDER THE AVOIDANCE FORCES WITH OTHER AGENTS
			glBegin(GL_LINES);
			glColor3f(1.0, 1.0, 0.3);
			glVertex3f(mPos.x, mPos.y, mPos.z);
			glVertex3f(mPos.x+fOtherAgents.x, mPos.y+fOtherAgents.y, mPos.z+fOtherAgents.z);
			glEnd();	
		}
	
	float fWeightWalls = 15;

	float fWeightObst = 5;  // Here fobst can only be of length 1 if there's avoidance, or 0 if there's not

	fTotal = fAttrac*fWeightAttrac + fWalls*fWeightWalls + fObst*fWeightObst + fDesiredSpeed + fOtherAgents*fAgents; 
	normalize(&fTotal);
	step.x = mPos.x + m_dt*speed*fTotal.x; //*(0.80);//0.8;//*0.2;//OFFSET_HUMS;
	step.y = mPos.y;		
	step.z = mPos.z + m_dt*speed*fTotal.z; //*(0.80);//0.8;//*0.2;//OFFSET_HUMS;

	// Limit how much the desiredVel vector can change!
	tVector dirMovement;
	dirMovement = mPos - m_prevpos;
	float distStep = normalize(&dirMovement);

	m_desiredVel = m_desiredVel*2  + dirMovement + fAttrac;
	m_desiredVel.y = 0;
	dist2 = normalize(&m_desiredVel);

	// The orientation should change less abruptly
	m_orientation = m_orientation*26 + m_desiredVel; // OJO CAMBIOS SIGGRAPH

	normalize(&m_orientation);
	fDesiredSpeed = m_desiredVel;

	#ifdef MYDEBUG
	{
		//	DRAW DEBUG FORCES
/*		glBegin(GL_POINTS);
			glColor3f(0.0, 0.0, 0.0);
			glVertex3f(mPos.x, 1.0, mPos.z);	
		glEnd();
		glBegin(GL_LINES);
			glColor3f(1.0, 1.0, 0.0);
			glVertex3f(mPos.x, 1.0, mPos.z);
			tVector deb_force = mPos + fOtherAgents*20;
			glVertex3f(deb_force.x, 1.0, deb_force.z);
		glEnd();
		//  END DEBUG FORCES
*/
	}
	#endif

	num_step = (num_step + 1) % NUM_tVectorS;
	
	if (m_nextTask)
	{
		glColor3f(m_color[0],m_color[1],m_color[2]);
		glBegin(GL_POINTS);
		glVertex3f(m_tasks.front().location.x,m_tasks.front().location.y+0.1,m_tasks.front().location.z); 
		glEnd();
			
		if (dist < m_tasks.front().proximity)
		if (!m_doTask)
		{
			m_tasks.front().timer = m_tasks.front().duration; // Initiate the timer to the duration of the action
			m_doTask = true;	// will be true while the task is being performed, and thus we know we have to decrease the timer
			myControl->glWin->execute_idle=true;   // will be true when the cal3d action needs to be started and then it will be set to false immediately
			m_execute_idle=true;
			//cout << "ACCION";
			m_nextTask=false;
		}
	}
	if (m_doTask)
	{
		step = mPos;
	}
	else if (!m_nextTask)
	{
		switch (m_crossingState)
		{
			case NOT_CROSSING:
				if (dist < 1.25*SIZE_CELL)
					m_crossingState = APPROACHING_A;
				
				break;
			case APPROACHING_A:
				m_pprevCell->increaseNumPeopleCrossing(m_nextPortal, mPos);
				if (dist < 0.9*SIZE_CELL)
				{
					m_crossingState = REACHING_A;
					m_building->insertHum(m_nextPortal,this);
					//m_building->printListHums(m_nextPortal);
				}
				break;
			case REACHING_A:
				m_pprevCell->increaseNumPeopleCrossing(m_nextPortal, mPos);
				if (dist < 0.6*SIZE_CELL)
				{
					m_portalIDcoll = m_nextPortal;
					m_crossingState = CLOSE_TO_A;
					m_crossingPortal = true;
					if (m_nextPortal > 0)
					{		
						pos = m_pcell->getPortal(m_nextPortal).pos;
						// I need to keep track of the attractor previous to crossing the door, so that i know exactly when the door
						// is crossed, which will be when the distance to the new m_attractor is smaller than the distance to the 
						// previous one m_prevAttractor.
						m_prevAttractor = m_attractor;
						m_attractor.x = pos[0];
						m_attractor.y = pos[1];
						m_attractor.z = pos[2];	
					}
				}
				break;
			case CLOSE_TO_A:
				m_pprevCell->increaseNumPeopleCrossing(m_nextPortal, mPos);
				
				distPrevAtt = length(m_prevAttractor-mPos);
				// I want to find out the exact moment in which the door is crossed
				//if (dist < 0.4*SIZE_CELL)
				if (dist <= distPrevAtt) // Needs to be <= cause otherwise it doesn't work in the exit, since they will be the same thing...
				{
					//cout << " door crossed: " << myControl->glWin->globalTimeMillisecs << endl;
					m_crossingState = LEAVING_B;
					m_doorCrossed = true;		// The human will chooseNextCell when the door has finally been crossed
					m_doorCrossedFR = true;		// This is used to count the number of people crossing to calculate flow rates during the validation
					
					// This was commented out, because i could observe people suddenly dissapearing even though they haven't found
					// an exit. I'm not sure why i ever made this case special... 			
					if (m_pcell == m_pprevCell) // it means we have reached an exit
					{
						int numExits = m_building->getNumExits();
						for (int i=0; i<numExits; i++)
						{
							if (m_pcell->getPath(i).length==0)
							{
								m_reached_goal = true;
								//cout << "hum: " <<m_humID<< "removed in cell: " << m_myCell << endl;
								exitCell();
								m_portalIDcoll = -1;   // -1 means that we are not crossing a portal anymore, and therefore collision detection with those humans is not necessary
								m_building->removeHum(m_nextPortal,this);
							}
						}
					}

					num_step = 0; 
					m_crossingPortal = false;
					m_tryingToCross = false;
					m_lastAttracPos = m_attractor; // We need to save this information, because in ChooseNextCell will modify the attractor
  					m_lastAttracID   = m_nextPortal;

					// Here it counts as one more person crossed the portal
					m_building->increasePeopleCrossed(m_nextPortal);
				}
				break;
			case LEAVING_B: // When we are far away from B then we can remove the human from the list of humans crossing that is used for collision detection purposes
				//if ((LENGTH((mPos.x-m_lastAttracPos.x),(mPos.z-m_lastAttracPos.z)) > 0.8*SIZE_CELL) || //> 0.6*SIZE_CELL) ||
				//	(dist < 0.7*SIZE_CELL)) // OR is necessary in case the next attractor is closer than the distance necessary to leave B			
				
				distPrevAtt = length(m_prevAttractor-mPos);
				distLastAtt = length(m_lastAttracPos-mPos);
				if (((distLastAtt > 0.8*SIZE_CELL) && (distPrevAtt>distLastAtt)) || //> 0.6*SIZE_CELL) ||
					(dist < 0.7*SIZE_CELL)) // OR is necessary in case the next attractor is closer than the distance necessary to leave B			
				{
					m_crossingState = NOT_CROSSING;
					m_portalIDcoll = -1;   // -1 means that we are not crossing a portal anymore, and therefore collision detection with those humans is not necessary
					m_building->removeHum(m_lastAttracID,this);
				}
				break;
		};
	}

	return step;
}




tVector Chumanoid::calculateStepsCellularAutomata()
{
	tVector step;
	float dist, dist2;
	float distPrevAtt, distLastAtt; 
	tVector fTotal, fAttrac, fWalls, fObst = {0.0,0.0,0.0};
	tVector fDesiredSpeed = {0.0,0.0,0.0};
	tVector fLook4Walls = {0.0, 0.0, 0.0};
	tVector fOtherAgents = {0.0, 0.0, 0.0};
	tVector fOthers0 = {0.0, 0.0, 0.0};
	tVector fOthers1 = {0.0, 0.0, 0.0};
//	double speed;
//	double dt;
//	float fWeightAttrac;
	double* pos;
//	tVector fAvoidFallenAgents;
//	float fFallenAgent;
//	tVector fAvoidCircles;

	unsigned long currTime;
	static unsigned long incrTime=0;
	static unsigned long frameRate=0;
	static unsigned long prevTime=0;
	//static bool nextTask = false;

	//cout << "Cellular Automata Model"<<endl;

	_timeb tm;	
	_ftime(&tm);
	currTime = tm.time*1000 + tm.millitm; 
	incrTime += (currTime - prevTime);

	//speed = m_maxSpeed;

	// In a CA the step has to be in cells of size 0.5x0.5
	float floorX = floor(mPos.x);
	float decX   = mPos.x - floorX;
	if (decX < 0.5)
		mPos.x = floorX;
	else 
		mPos.x = floorX+0.5;

	float floorZ = floor(mPos.z);
	float decZ   = mPos.z - floorZ;
	if (decZ < 0.5)
		mPos.z = floorZ;
	else 
		mPos.z = floorZ+0.5;	
	
	#ifdef REAL_TIME
	//	_timeb tm;
	//	_ftime(&tm);
	//	currTime = tm.time*1000 + tm.millitm; 
		m_dt = currTime - prevTime; 
		//printf("dt = %f",dt);
	//	m_time = currTime ;
	#else
		m_dt = dif_millSecs; //200;
	#endif

	prevTime = currTime ;

	// We reduce the speed of the human when they are going downstairs
	//sSTAIR stair;
	//if (m_pprevCell->getIsStair(&stair))
	//{
	//	speed = speed * 0.8; //0.57;
	//}

	
	if ((!m_nextTask) && (!m_doTask) && (m_crossingState<0) && (!m_tasks.empty()))
		if ((m_tasks.front()).roomID == m_prevCell) // OJO antes era prev!!!!
			m_nextTask=true;
		
	if (m_nextTask)
		fAttrac = m_tasks.front().location - mPos;
	else 
		fAttrac = m_attractor - mPos;

	dist = LENGTH(fAttrac.x,fAttrac.z);
	normalize(&fAttrac);

	fTotal = fAttrac; //*fWeightAttrac + fWalls*fWeightWalls + fObst*fWeightObst + fDesiredSpeed + fOtherAgents*fAgents; 
	normalize(&fTotal);

	
	float dx=0,dz=0;
	m_pprevCell->findNeighbourCellCA(this, &dx, &dz, fAttrac, m_portalIDcoll);

	//if (fTotal.x < -0.333) dx = -0.5;
	//else if (fTotal.x < 0.333) dx = 0;
	//else dx = 0.5;
	//
	//if (fTotal.z < -0.333) dz = -0.5;
	//else if (fTotal.z < 0.333) dz = 0;
	//else dz = 0.5;

	step.x = mPos.x + dx; 
	step.y = mPos.y;
	step.z = mPos.z + dz; 

	// Limit how much the desiredVel vector can change!
	tVector dirMovement;
	dirMovement = mPos - m_prevpos;
	float distStep = normalize(&dirMovement);

	//m_desiredVel = m_desiredVel*2  + dirMovement + fAttrac;
	m_desiredVel = fTotal;
	m_desiredVel.y = 0;
	dist2 = normalize(&m_desiredVel);

	// The orientation should change less abruptly
	m_orientation = m_orientation + m_desiredVel; // OJO CAMBIOS SIGGRAPH
	normalize(&m_orientation);
	
	fDesiredSpeed = m_desiredVel;
	
	#ifdef MYDEBUG
	{
		//	DRAW DEBUG FORCES
/*		glBegin(GL_POINTS);
			glColor3f(0.0, 0.0, 0.0);
			glVertex3f(mPos.x, 1.0, mPos.z);	
		glEnd();
		glBegin(GL_LINES);
			glColor3f(1.0, 1.0, 0.0);
			glVertex3f(mPos.x, 1.0, mPos.z);
			tVector deb_force = mPos + fOtherAgents*20;
			glVertex3f(deb_force.x, 1.0, deb_force.z);
		glEnd();
		//  END DEBUG FORCES
*/
	}
	#endif

	num_step = (num_step + 1) % NUM_tVectorS;
	
	if (m_nextTask)
	{
		glColor3f(m_color[0],m_color[1],m_color[2]);
		glBegin(GL_POINTS);
		glVertex3f(m_tasks.front().location.x,m_tasks.front().location.y+0.1,m_tasks.front().location.z); 
		glEnd();
			
		if (dist < m_tasks.front().proximity)
		if (!m_doTask)
		{
			m_tasks.front().timer = m_tasks.front().duration; // Initiate the timer to the duration of the action
			m_doTask = true;	// will be true while the task is being performed, and thus we know we have to decrease the timer
			myControl->glWin->execute_idle=true;   // will be true when the cal3d action needs to be started and then it will be set to false immediately
			m_execute_idle=true;
			//cout << "ACCION";
			m_nextTask=false;
		}
	}
	if (m_doTask)
	{
		step = mPos;
	}
	else if (!m_nextTask)
	{
		switch (m_crossingState)
		{
			case NOT_CROSSING:
				if (dist < 1.25*SIZE_CELL)
					m_crossingState = APPROACHING_A;
				
				break;
			case APPROACHING_A:
				m_pprevCell->increaseNumPeopleCrossing(m_nextPortal, mPos);
				if (dist < 0.9*SIZE_CELL)
				{
					m_crossingState = REACHING_A;
					//m_building->printListHums(m_nextPortal);
					m_building->insertHum(m_nextPortal,this);
					//m_building->printListHums(m_nextPortal);
					
					//m_portalIDcoll = m_nextPortal;
					// Add the human to the list of humans trying to cross this portal
			//		m_pcell->increaseNumPeopleCrossing(m_nextPortal);
			//		m_tryingToCross = true;
				}
				break;
			case REACHING_A:
				m_pprevCell->increaseNumPeopleCrossing(m_nextPortal, mPos);
				if (dist < 0.6*SIZE_CELL)
				{
					m_portalIDcoll = m_nextPortal;
					m_crossingState = CLOSE_TO_A;
					m_crossingPortal = true;
					if (m_nextPortal > 0)
					{		
						pos = m_pcell->getPortal(m_nextPortal).pos;
						// I need to keep track of the attractor previous to crossing the door, so that i know exactly when the door
						// is crossed, which will be when the distance to the new m_attractor is smaller than the distance to the 
						// previous one m_prevAttractor.
						m_prevAttractor = m_attractor;
						m_attractor.x = pos[0];
						m_attractor.y = pos[1];
						m_attractor.z = pos[2];	
					}
				}
				break;
			case CLOSE_TO_A:
				m_pprevCell->increaseNumPeopleCrossing(m_nextPortal, mPos);
				
				distPrevAtt = length(m_prevAttractor-mPos);
				// I want to find out the exact moment in which the door is crossed
				//if (dist < 0.4*SIZE_CELL)
				if (dist <= distPrevAtt) // Needs to be <= cause otherwise it doesn't work in the exit, since they will be the same thing...
				{
					//cout << " door crossed: " << myControl->glWin->globalTimeMillisecs << endl;
					m_crossingState = LEAVING_B;
					m_doorCrossed = true;		// The human will chooseNextCell when the door has finally been crossed
					m_doorCrossedFR = true;		// This is used to count the number of people crossing to calculate flow rates during the validation
					
					// This was commented out, because i could observe people suddenly dissapearing even though they haven't found
					// an exit. I'm not sure why i ever made this case special... 			
					if (m_pcell == m_pprevCell) // it means we have reached an exit
					{
						int numExits = m_building->getNumExits();
						for (int i=0; i<numExits; i++)
						{
							if (m_pcell->getPath(i).length==0)
							{
								m_reached_goal = true;
								//cout << "hum: " <<m_humID<< "removed in cell: " << m_myCell << endl;
								exitCell();
								m_portalIDcoll = -1;   // -1 means that we are not crossing a portal anymore, and therefore collision detection with those humans is not necessary
								m_building->removeHum(m_nextPortal,this);
							}
						}
					}

					num_step = 0; 
					m_crossingPortal = false;
					m_tryingToCross = false;
					m_lastAttracPos = m_attractor; // We need to save this information, because in ChooseNextCell will modify the attractor
  					m_lastAttracID   = m_nextPortal;

					// Here it counts as one more person crossed the portal
					m_building->increasePeopleCrossed(m_nextPortal);
				}
				break;
			case LEAVING_B: // When we are far away from B then we can remove the human from the list of humans crossing that is used for collision detection purposes
				//if ((LENGTH((mPos.x-m_lastAttracPos.x),(mPos.z-m_lastAttracPos.z)) > 0.8*SIZE_CELL) || //> 0.6*SIZE_CELL) ||
				//	(dist < 0.7*SIZE_CELL)) // OR is necessary in case the next attractor is closer than the distance necessary to leave B			
				
				distPrevAtt = length(m_prevAttractor-mPos);
				distLastAtt = length(m_lastAttracPos-mPos);
				if (((distLastAtt > 0.8*SIZE_CELL) && (distPrevAtt>distLastAtt)) || //> 0.6*SIZE_CELL) ||
					(dist < 0.7*SIZE_CELL)) // OR is necessary in case the next attractor is closer than the distance necessary to leave B			
				{
					m_crossingState = NOT_CROSSING;
					m_portalIDcoll = -1;   // -1 means that we are not crossing a portal anymore, and therefore collision detection with those humans is not necessary
					m_building->removeHum(m_lastAttracID,this);
				}
				break;
		};
	}

/*
	// Check the distance to the attractor point, and when it's very close, we'll change m_myCell
	// If we are close to the portal, we'll increase the list of people trying to cross this portal
	if ((dist < SIZE_CELL) && !(m_tryingToCross))
	{
//		m_pcell->increaseNumPeopleCrossing(m_nextPortal);
		m_tryingToCross = true;
	}
	// We have two attractors, one before the portal and another one right after the portal, so when the 
	// first one is crossed we set crossingPortal to true, and when the second attractor is approached
	// it means we are already in the next cell
	if ((dist < 0.6*SIZE_CELL) && !(m_crossingPortal))
	{
		m_crossingPortal = true;
		if (m_nextPortal > 0)
		{
			float* pos = m_pcell->getPortal(m_nextPortal).pos;
			m_attractor.x = pos[0];
			m_attractor.y = pos[1];
			m_attractor.z = pos[2];
		}
	}
	else if ((dist < 0.4*SIZE_CELL) && (m_crossingPortal)) // Distance to attractor
	{
		m_doorCrossed = true;
		num_step = 0;
		m_crossingPortal = false;
		m_tryingToCross = false;
//		m_pcell->getPortal(m_nextPortal).numPeopleCrossing--;
	}
*/
	return step;
}

sLeadingBehavior Chumanoid::getPersonality()
{
	return m_personality;
}

void Chumanoid::setPersonality(sLeadingBehavior pers)
{
	m_personality = pers;
}

void Chumanoid::calculateLookAheadEnds()
{
	// UP = {0,1,0}
	// PV = position of agent plus desired velocity vector, so it's a bit ahead the agent in the direction of desired movement
	tVector desiredDistance = m_desiredVel; //*1.6;
	tVector PV = mPos + desiredDistance;
/*
	// Look Ahead A is computed by UPxPV
	m_lookAheadA.x = m_desiredVel.z;
	m_lookAheadA.z = -m_desiredVel.x;
	m_lookAheadA = m_lookAheadA*0.4;
	m_lookAheadA = PV + m_lookAheadA;
	// Look Ahead A is computed by PVxUP
	m_lookAheadB.x = -m_desiredVel.z;
	m_lookAheadB.z = m_desiredVel.x;
	m_lookAheadB = m_lookAheadB*0.4;
	m_lookAheadB = PV + m_lookAheadB;
*/
	// Look Ahead A is computed by UPxPV
	m_vecA.x = desiredDistance.z;
	m_vecA.z = -desiredDistance.x;
	m_vecA   = m_vecA*0.4;
	m_lookAheadA = PV + m_vecA;
	// Look Ahead A is computed by PVxUP
	m_vecB.x = -desiredDistance.z;
	m_vecB.z = desiredDistance.x;
	m_vecB = m_vecB*0.4;
	m_lookAheadB = PV + m_vecB;

}

tVector Chumanoid::lookAhead4Agents(tVector pos)
{
	tVector force={0.0, 0.0, 0.0};
	float distA, distB;
	tVector endA, endB;

	endA = pos - m_lookAheadA;
	endB = pos - m_lookAheadB;
	distA = LENGTH(endA.x, endA.z);
	distB = LENGTH(endB.x, endB.z);

	if (distA < distB)
		return m_vecB;
	else
		return m_vecA;
	return force;
}


int Chumanoid::getOthersFirst(){return m_letOthersFirst;};
void Chumanoid::setOthersFirst(int othersFirst){m_letOthersFirst=othersFirst;};

// Based on current bottleneck and my patiece level i check whether i should pick another portal
bool Chumanoid::portalBottleNeck()
{
	sPORTAL portal;

	// If i'm a patience person, i'll just wait trying to get through this portal
	if (mTrait.GetImpatience() < 5)
		return false;
	else  // Else, depending on the number of people i'll decide what to do
	{
		portal = m_pprevCell->getPortal(m_nextPortal);
		//if ((m_crossingState<0) && (portal.numPeople[0] > 2))
		//if ((m_crossingState<0) && ((portal.numPeople[0]+portal.numPeople[1]) > 12))
		if ((m_crossingState<1) && ((portal.numPeople[0]+portal.numPeople[1]) > 5)) //12))
		{
			m_bottleNeck = portal.id;
			return true;
		}
		else
			return false;
	}
};


bool Chumanoid::initCal3d(int numhum)
{
//	GLuint m_cursorTextureId;
// load the cursor texture
  std::string m_strDatapath;

  
  //int IDmodel = 0;
  int IDmodel = numhum%NUM_MODELS;
  //int IDmodel = PENGUIN; //Jan for penguin simulation
  //int IDmodel = LADY2;

  m_strDatapath = "cal3d/data/";
//	m_strDatapath = "Cal3Davatars/james/";

  static int numModel = 0;
  cout << "Model = " << numModel<< endl;
  numModel++;

/* to test agents crossing thru
  if(mTrait.GetGroupId()%2 == 0)
	  IDmodel = numhum%10;
  else
	  IDmodel = numhum%10 + 10;
	  */
  //initcal3d according to personality
  /*
	if(mTrait.GetFFM(EXTRO) < 0)
		IDmodel = HITCH1;
	else
		IDmodel = HITCH7;
*/

  /*
  if(mTrait.GetFFM(AGREE) < 0 && mTrait.GetFFM(CONSCIENT) < 0)
	  IDmodel = HITCH0;
  else if(mTrait.GetFFM(AGREE) < 0)
	  IDmodel = HITCH8;	
  else if(mTrait.GetFFM(CONSCIENT) < 0)
	  IDmodel = HITCH3;
  else
	  IDmodel = HITCH2;
*/

  //neuroticism
  if(mTrait.GetPanicLevel() > 5)
	  IDmodel = HITCH0;
  else
  {
	IDmodel = numhum%NUM_MODELS + 1;
	if(IDmodel > LADY9)
		IDmodel--;
  }




  switch (IDmodel)
  {
	//case HITCH:
	//case HITCH_HAIR:
	//case HITCH_HAIR_THINNER:
	//case HITCH_SKINNY:
	//case HITCH_THINNER:
	//case HITCH_24: 
	//case HITCH_30: 
	//  m_pcal3dAgent->setPath("cal3d/data/hitch/");
	//  break;
	//case LADY:
	//  m_pcal3dAgent->setPath("cal3d/data/Lady/");
	//  break;
  case HITCH0:  //Jan
	case HITCH1:
	case HITCH2:
	case HITCH3:
	case HITCH4:
	case HITCH5:
	case HITCH6:
	case HITCH7:
	case HITCH8:
	case HITCH9:
	  m_pcal3dAgent->setPath("cal3d/data/hitch/");
	  break;
	case LADY0:	 //Jan
	case LADY1:	
	case LADY2:	
	case LADY3:	
	case LADY4:	
	case LADY5:	
	case LADY6:	
	case LADY7:	
	case LADY8:	
	case LADY9:	
	  m_pcal3dAgent->setPath("cal3d/data/Lady/");
	  break;
	case CALLY:
	  m_pcal3dAgent->setPath("cal3d/data/cally/");
	  break;
	case LAMBERT: 
	  m_pcal3dAgent->setPath("cal3d/data/lambert/");
	  break;
	case PENGUIN: 
	  m_pcal3dAgent->setPath("cal3d/data/penguin/");
	  break;
	case KID0:
	case KID1:
	case KID2:
	case KID3:
	case KID4:
	case KID5:
	  m_pcal3dAgent->setPath("cal3d/data/kid/");
	  break;
  };
//    m_pcal3dAgent->setPath("cal3d/data/cally/");
//  m_pcal3dAgent->setPath("cal3d/data/hitch/");
//    m_pcal3dAgent->setPath("cal3d/data/paladin/");
//  m_pcal3dAgent->setPath("cal3d/data/jack/");
//  m_pcal3dAgent->setPath("Cal3Davatars/james/");

  // initialize models
  Model *pModel;

  // load 'cally' model
  std::cout << "Loading 'cally' model ..." << std::endl;

 // if (m_strCal3D_Datapath != "")
 //   pModel->setPath( m_strCal3D_Datapath + "/" + "cally/" );

 
 // if(!m_pcal3dAgent->onInit(m_strDatapath + "james_gm2.cfg"))
 // if(!m_pcal3dAgent->onInit(m_strDatapath + "paladin.cfg"))
 // if(!m_pcal3dAgent->onInit(m_strDatapath + "cally.cfg"))

  string cfg_file;

  switch (IDmodel)
  {
	//case HITCH: 
	//  cfg_file = "hitch.cfg";
	//  break;
	//case HITCH_HAIR:
	//  cfg_file = "hitchHair.cfg";
	//  break;
	//case HITCH_HAIR_THINNER:
	//  cfg_file = "hitchHairThinner.cfg";
	//  break;
	//case HITCH_SKINNY:
	//  cfg_file = "hitchSkinny.cfg";
	//  break;
	//case HITCH_THINNER:
	//  cfg_file = "hitchThinner.cfg";
	//  break;
	// case HITCH_24: 
	//  cfg_file = "hitch_24.cfg";
	//  break;
	//case HITCH_30: 
	//  cfg_file = "hitch_30.cfg";
	//  break;
	//case LADY:
	//  cfg_file = "Lady.cfg";
	//  break;
	case HITCH0: 
	  cfg_file = "hitch0.cfg";
	  break;
	case HITCH1: 
	  cfg_file = "hitch1.cfg";
	  break;
	case HITCH2: 
	  cfg_file = "hitch2.cfg";
	  break;
	case HITCH3: 
	  cfg_file = "hitch3.cfg";
	  break;
	case HITCH4: 
	  cfg_file = "hitch4.cfg";
	  break;
	case HITCH5: 
	  cfg_file = "hitch5.cfg";
	  break;
	case HITCH6: 
	  cfg_file = "hitch6.cfg";
	  break;
	case HITCH7: 
	  cfg_file = "hitch7.cfg";
	  break;
	case HITCH8: 
	  cfg_file = "hitch8.cfg";
	  break;
	case HITCH9: 
	  cfg_file = "hitch9.cfg";
	  break;
	case LADY0:
	  cfg_file = "Lady0.cfg";
	  break;
	  case LADY1:
	  cfg_file = "Lady1.cfg";
	  break;
	case LADY2:
	  cfg_file = "Lady2.cfg";
	  break;
	case LADY3:
	  cfg_file = "Lady3.cfg";
	  break;
	case LADY4:
	  cfg_file = "Lady4.cfg";
	  break;
	case LADY5:
	  cfg_file = "Lady5.cfg";
	  break;
	case LADY6:
	  cfg_file = "Lady6.cfg";
	  break;
	case LADY7:
	  cfg_file = "Lady7.cfg";
	  break;
	case LADY8:
	  cfg_file = "Lady8.cfg";
	  break;
	case LADY9:
	  cfg_file = "Lady9.cfg";
	  break;
	
	case CALLY:
	  cfg_file = "cally.cfg";
	  break;
	case LAMBERT:
	  cfg_file = "lambert.cfg";
	  break;
	case PENGUIN:
	  cfg_file = "penguin.cfg";
	  break;

  	case KID0:
	  cfg_file = "kid0.cfg";
	  break;
	case KID1:
	  cfg_file = "kid1.cfg";
	  break;
	case KID2:
	  cfg_file = "kid2.cfg";
	  break;
	case KID3:
	  cfg_file = "kid3.cfg";
	  break;
	case KID4:
	  cfg_file = "kid4.cfg";
	  break;
	case KID5:
	  cfg_file = "kid5.cfg";
	  break;
  };
  if(!m_pcal3dAgent->onInit(m_strDatapath + cfg_file))
  {
    delete pModel;
    std::cerr << "Model initialization failed! (cally)" << std::endl;
    return false;
  }
 
  std::cout << std::endl;

  std::cout << std::endl;

  return true;
};

void Chumanoid::removeTasks()
{

	m_nextTask = false;
	m_doTask = false;
	m_execute_idle = false;
	if (!m_tasks.empty())
		m_tasks.clear();
}






// VR_USER functions:
// We need to check whether the new desired position is possible
bool Chumanoid::canUserMove(float *newPos)
{
	tVector auxPos;
	float2tVector(newPos,&auxPos);
	tVector pushing;

	// We need to calculate whether this new position intersects with any element in the room:
	// Intersection against walls:
	if (m_pprevCell->intersectsWall(auxPos, m_desiredVel) || (m_pprevCell->intersectsObst(auxPos)))
		return false;

	pushing = m_pprevCell->intersectsOtherAgents(auxPos, m_humID, m_desiredVel, mTrait.GetPersonalSpace());
	if (length(pushing) > 0)
		return false;

	return true; // If there's no intersection with any element, then it's a valid movement
}

// We need to check whether the user crosses any of the portals in the room
bool Chumanoid::assignNewCell(float *newPos)
{
	map<int, sPORTAL> tablePortals;
	map<int, sPORTAL>::iterator iter;
	float dist1;
	float dist2=1000;
	tVector doorPos;
	static int portal;
//	static int cellID;
	static int state = 0;
	float crossingPos[3];	// Will be the value of the attractor at the other side of the portal. 
							// when the user is closer to that point than to the previous one, then
							// it has crossed the portal and we need to update the cell ID

	float minDist = 1000;
	tablePortals = m_pprevCell->getTablePortals();
	if (m_crossingState == NOT_CROSSING)
	{
		// If it is not crossing any portal yet, then we need to figure out whether it's getting close to any portal
		for (iter=tablePortals.begin(); iter!=tablePortals.end();iter++)
		{
			double2tVector((*iter).second.pos,&doorPos);
			dist1 = length(doorPos-mPos);	
			if ((dist1<1) && (dist1<minDist))
			{
				minDist = dist1;
				m_myCell = (*iter).second.nextcellID;
				if (m_myCell>-1)
					m_pcell = m_building->getCell(m_myCell);
				portal = (*iter).first;
				crossingPos[0] = m_pcell->getPortal(portal).pos[0];
				crossingPos[1] = m_pcell->getPortal(portal).pos[1];
				crossingPos[2] = m_pcell->getPortal(portal).pos[2];
				m_crossingState = APPROACHING_A;
				m_building->removeHum(portal,this); // We remove it first, just in case he was already in the list by coming from the other side of the door
				m_building->insertHum(portal,this);
			}
		}
	}
	else if (m_crossingState == APPROACHING_A)
	{
		// Check when the user crosses the portal
		if (m_myCell>-1)
		{
			double2tVector(m_pcell->getPortal(portal).pos,&doorPos);
			dist2 = length(doorPos-mPos);	
		}
		double2tVector(m_pprevCell->getPortal(portal).pos,&doorPos);
		dist1 = length(doorPos-mPos);	
		

		// OJO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//	Al entrar aqui se quitara de la lista de humanos cruzando, y hasta que no me mueva, no lo metere de nuevo en la lista,//
		// con lo cual el user puede aparecer temporalmente transparente a efectos de colision detection
		if (dist2 < dist1) // crossing portal
		{
			exitCell();
			m_prevCell = m_myCell;
			m_pprevCell = m_pcell;
			m_crossingState = NOT_CROSSING;
			enterCell();
			m_building->removeHum(portal,this);
			return true;
		}
		if ((dist1>1) && (dist2>1)) // it's not crossing, but instead moving back
		{
			m_crossingState = NOT_CROSSING;
			m_building->removeHum(portal,this);
		}
	}
	return false;
}
