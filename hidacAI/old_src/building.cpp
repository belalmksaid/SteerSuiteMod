//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

//****************************************************************************//
// building.cpp                                                        //
// Copyright (C) 2007 Nuria Pelechano                                        //
//****************************************************************************//
// This code is covered by a disclosure agreement filed by the 
// University of Pennsylvania.  Do not distribute with out permission of
// the author.  
//****************************************************************************// 
#pragma warning (disable : 4715)

//#include "stdafx.h"
#include <stdlib.h>
#include "building.h"
#include "CPG/CPG.h"

//#include <GL/glut.h>
#include "globalVbles.h"
#include <algorithm>
#include "humanoid.h"


// OBJECTS:
//#include "objectsDL.h"
//CobjectsDL object;
#include "StaticModelInstance.h"
#include "Textures\filehandling.h"
#include "Textures\texture.h"
#include "ModelType.h"




Cbuilding::Cbuilding()
{
	CPG cpg;
	FILE *pSaveFile;
	FILE *pInputFile;

	//char *filename;

//	filename = _strdup("small floorplan.txt");
//	filename = _strdup("CarlinBuilding4xMirror.txt");
//	filename = _strdup("CarlinBuildingTextured.txt");
//	filename = _strdup("NEW FLOORPLAN.txt");
//	filename = _strdup("janbuilding1.txt");
//	filename = _strdup("demoTex.txt");
//	filename = _strdup("city_big.txt");
//	filename = _strdup("demoTex3.txt");
//	filename = _strdup("taskRoom.txt");
//	filename = _strdup("PenguinRoom.txt");
  //filename = _strdup("party.txt");
	//filename = _strdup("fundaextroversion.txt"); //single table

//	filename = _strdup("fundaopenness.txt");  //several tables
//	filename = _strdup("fundaopennessoffice.txt");
//	filename = _strdup("fundaempty.txt");
//	filename = _strdup("fundaleader.txt");
//	filename = _strdup("city.txt");
	filename = _strdup("fundaNoWalls.txt");

	
	//filename = _strdup("fundamuseum.txt");  //openness
		
	
	#ifdef MYDEBUG
		//pInputFile = fopen("building1_SMALL.txt","r");
		fopen_s(&pInputFile,"building1_SMALL.txt","r");
	#else
		//pInputFile = fopen("janbuilding1.txt","r");
		//fopen_s(&pInputFile, "janbuilding1.txt","r");
		fopen_s(&pInputFile, filename,"r");
		//fopen_s(&pInputFile, "carlin.building.txt","r");
	#endif
	

	fopen_s(&pSaveFile, "outCPG.txt","w");
	cpg.loadFile(pInputFile);
	cpg.createCPG();

	nexitsBuilding = 0;

	m_building3D.copyBuilding(cpg.getGrid(), cpg.getNumFoors(), cpg.getSizeX(), cpg.getSizeZ());

	createEnvironment(&cpg);
	createTiles(&cpg);

//	computeLandMarks();

	cpg.saveFile(pSaveFile);
	fclose(pInputFile);

	// for textures
	loadFloorplan(filename);
	//Did have texture loading here

};

Cbuilding::~Cbuilding()
{
	int i,j;

	for (i=0; i<nfloors; i++)
	{
		for (j=0; j<sizeX; j++)
			delete m_tiles[i][j];
		delete [] m_tiles[i];
	}
	delete [] m_tiles;

	for (i=1; i<=m_building.size(); i++)
	{
		delete m_building[i]; 
	}
}
/*
void Cbuilding::createCPG(CPG *cpg)
{
		//CPG cpg;
	FILE *pSaveFile;
	FILE *pInputFile;

	pInputFile = fopen("building1.txt","r");
	pSaveFile = fopen("outCPG.txt","w");
	cpg->loadFile(pInputFile);
	cpg->createCPG();

	nexitsBuilding = 0;

	createEnvironment(cpg);
	createTiles(cpg);

	cpg->saveFile(pSaveFile);
	fclose(pInputFile);
}
*/

//Method no longer needed with the addition of the loadObjs method.
/*
void Cbuilding::addFurniture(bool obstacles)
{
	double pos[3]={0,0,0};
	double rot[3]={0,0,0};

	// EXAMPLE of how to do furniture now:

	// For this to work, there must be an "objects" directory in the project directory, to hold the .obj, .mtl, and texture files

//Room 1 ----------------------------------------------------------------------------------------------------------------------
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 90;  //orientation of desk
	pos[0]=5;
	pos[1]=0;
	pos[2]=4;
	addFurnitureToRooms(obstacles,"desk.obj",1,pos,rot,0.012, 0.4);  //radius is automatically calculated now
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=4;
	pos[1]=0;
	pos[2]=4;
	addFurnitureToRooms(obstacles,"office_chair.obj",1,pos,rot,0.01,0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]= 1.5;
	pos[1]=0.7;
	pos[2]=6;
	addFurnitureToRooms(obstacles,"m_plnt5.obj",1,pos,rot,0.04, 0.3);
	
	rot[0] = 0;
	rot[1] = 90;
	rot[2] = 0;
	pos[0]=5;
	pos[1]=1.1;
	pos[2]=4;
	addFurnitureToRooms(false,"Monitor.obj",1,pos,rot,0.09, 0.4);
	
	rot[0] = 0;
	rot[1] = 90;
	rot[2] = 0;
	pos[0]= 4.8;
	pos[1]= 0.8;
	pos[2]= 4;
	addFurnitureToRooms(false,"keyboard.obj",1,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = -90; //orientation
	pos[0]= 1.55;
	pos[1]= 0.05;
	pos[2]= 1.6;
	addFurnitureToRooms(obstacles,"4_drawer_file.obj",1,pos,rot,0.01,0.4);

//Room 10: Office ---------------------------------------------------------------------------------------------------------------
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 90;  //orientation of desk
	pos[0]=5;
	pos[1]=0;
	pos[2]=10;
	addFurnitureToRooms(obstacles,"desk.obj",10,pos,rot,0.012, 0.4);  //radius is automatically calculated now
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=4;
	pos[1]=0;
	pos[2]=10;
	addFurnitureToRooms(obstacles,"office_chair.obj",10,pos,rot,0.01,0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]= 1.5;
	pos[1]=0.7;
	pos[2]=12;
	addFurnitureToRooms(obstacles,"m_plnt5.obj",10,pos,rot,0.04, 0.3);
	
	rot[0] = 0;
	rot[1] = 90;
	rot[2] = 0;
	pos[0]=5;
	pos[1]=1.1;
	pos[2]=10;
	addFurnitureToRooms(false,"Monitor.obj",10,pos,rot,0.09, 0.4);
	
	rot[0] = 0;
	rot[1] = 90;
	rot[2] = 0;
	pos[0]= 4.8;
	pos[1]= 0.8;
	pos[2]= 10;
	addFurnitureToRooms(false,"keyboard.obj",10,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = -90; //orientation
	pos[0]= 1.55;
	pos[1]= 0.05;
	pos[2]= 7.6;
	addFurnitureToRooms(obstacles,"4_drawer_file.obj",10,pos,rot,0.01,0.4);

//Room 11: Office ---------------------------------------------------------------------------------------------------------------
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 90;  //orientation of desk
	pos[0]=5;
	pos[1]=0;
	pos[2]=16;
	addFurnitureToRooms(obstacles,"desk.obj",11,pos,rot,0.012, 0.4);  //radius is automatically calculated now
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=4;
	pos[1]=0;
	pos[2]=16;
	addFurnitureToRooms(obstacles,"office_chair.obj",11,pos,rot,0.01,0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]= 1.5;
	pos[1]=0.7;
	pos[2]=18;
	addFurnitureToRooms(obstacles,"m_plnt5.obj",11,pos,rot,0.04, 0.3);
	
	rot[0] = 0;
	rot[1] = 90;
	rot[2] = 0;
	pos[0]=5;
	pos[1]=1.1;
	pos[2]=16;
	addFurnitureToRooms(false,"Monitor.obj",11,pos,rot,0.09, 0.4);
	
	rot[0] = 0;
	rot[1] = 90;
	rot[2] = 0;
	pos[0]= 4.8;
	pos[1]= 0.8;
	pos[2]= 16;
	addFurnitureToRooms(false,"keyboard.obj",11,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = -90; //orientation
	pos[0]= 1.55;
	pos[1]= 0.05;
	pos[2]= 13.6;
	addFurnitureToRooms(obstacles,"4_drawer_file.obj",11,pos,rot,0.01,0.4);

//Room 13: Office ---------------------------------------------------------------------------------------------------------------
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 90;  //orientation of desk
	pos[0]=5;
	pos[1]=0;
	pos[2]=22;
	addFurnitureToRooms(obstacles,"desk.obj",13,pos,rot,0.012, 0.4);  //radius is automatically calculated now
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=4;
	pos[1]=0;
	pos[2]=22;
	addFurnitureToRooms(obstacles,"office_chair.obj",13,pos,rot,0.01,0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]= 1.5;
	pos[1]=0.7;
	pos[2]=26;
	addFurnitureToRooms(obstacles,"m_plnt5.obj",13,pos,rot,0.04, 0.3);
	
	rot[0] = 0;
	rot[1] = 90;
	rot[2] = 0;
	pos[0]=5;
	pos[1]=1.1;
	pos[2]=22;
	addFurnitureToRooms(false,"Monitor.obj",13,pos,rot,0.09, 0.4);
	
	rot[0] = 0;
	rot[1] = 90;
	rot[2] = 0;
	pos[0]= 4.8;
	pos[1]= 0.8;
	pos[2]= 22;
	addFurnitureToRooms(false,"keyboard.obj",13,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = -90; //orientation
	pos[0]= 1.55;
	pos[1]= 0.05;
	pos[2]= 19.6;
	addFurnitureToRooms(obstacles,"4_drawer_file.obj",13,pos,rot,0.01,0.4);

	//Room 20: Office ---------------------------------------------------------------------------------------------------------------
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 90;  //orientation of desk
	pos[0]=5;
	pos[1]=0;
	pos[2]=29;
	addFurnitureToRooms(obstacles,"desk.obj",20,pos,rot,0.012, 0.4);  //radius is automatically calculated now
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=4;
	pos[1]=0;
	pos[2]=29;
	addFurnitureToRooms(obstacles,"office_chair.obj",20,pos,rot,0.01,0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]= 1.5;
	pos[1]=0.7;
	pos[2]=32;
	addFurnitureToRooms(obstacles,"m_plnt5.obj",20,pos,rot,0.04, 0.3);
	
	rot[0] = 0;
	rot[1] = 90;
	rot[2] = 0;
	pos[0]=5;
	pos[1]=1.1;
	pos[2]=29;
	addFurnitureToRooms(false,"Monitor.obj",20,pos,rot,0.09, 0.4);
	
	rot[0] = 0;
	rot[1] = 90;
	rot[2] = 0;
	pos[0]= 4.8;
	pos[1]= 0.8;
	pos[2]= 29;
	addFurnitureToRooms(false,"keyboard.obj",20,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = -90; //orientation
	pos[0]= 1.55;
	pos[1]= 0.05;
	pos[2]= 27.6;
	addFurnitureToRooms(obstacles,"4_drawer_file.obj",20,pos,rot,0.01,0.4);

//Room 2: Empty hallway -----------------------------------------------------------------------------------------
//Room 3: -------------------------------------------------------------------------------------------------------
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]= 40.2;
	pos[1]=0.83;
	pos[2]=2.5;
	addFurnitureToRooms(obstacles,"plant02.obj",3,pos,rot,0.02, 0.4);
//Room 6: Conference room ---------------------------------------------------------------------------------------------------------
	rot[0] = 0;
	rot[1] = 90;
	rot[2] = 0;
	pos[0]= 17;
	pos[1]= 0.4;
	pos[2]= 10;
	addFurnitureToRooms(obstacles,"frntblm2.obj",6,pos,rot,0.04,0.4);

	rot[0] = 0;
	rot[1] = -90;
	rot[2] = 0;
	pos[0]= 18;
	pos[1]= 0.6;
	pos[2]= 5.25;
	addFurnitureToRooms(false,"whiteboard.obj",6,pos,rot,0.01,0.4);

	rot[0] = 0;
	rot[1] = 90; //orientation
	rot[2] = 0;
	pos[0]= 15;
	pos[1]= 0.05;
	pos[2]= 18.9;
	addFurnitureToRooms(obstacles,"longtable.obj",6,pos,rot,0.01,0.4);

	rot[0] = 0;
	rot[1] = 90; //orientation
	rot[2] = 0;
	pos[0]= 18;
	pos[1]= 0.05;
	pos[2]= 18.9;
	addFurnitureToRooms(obstacles,"longtable.obj",6,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=16;
	pos[1]=0;
	pos[2]=9.5;
	addFurnitureToRooms(obstacles,"office_chair.obj",6,pos,rot,0.01,0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=16;
	pos[1]=0;
	pos[2]=10.5;
	addFurnitureToRooms(obstacles,"office_chair.obj",6,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 180;
	pos[0]=18;
	pos[1]=0;
	pos[2]=9.5;
	addFurnitureToRooms(obstacles,"office_chair.obj",6,pos,rot,0.01,0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 180;
	pos[0]=18;
	pos[1]=0;
	pos[2]=10.5;
	addFurnitureToRooms(obstacles,"office_chair.obj",6,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 90;
	pos[0]=17;
	pos[1]=0;
	pos[2]=12;
	addFurnitureToRooms(obstacles,"office_chair.obj",6,pos,rot,0.01,0.4);

//Room 8: Dining room ---------------------------------------------------------------------------------------------------------

	rot[0] = 0;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=25;
	pos[1]=0;
	pos[2]=15;
	addFurnitureToRooms(obstacles,"roundTable.obj",8,pos,rot,0.145, 0.4);
	
	rot[0] = 0;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=30;
	pos[1]=0;
	pos[2]=15;
	addFurnitureToRooms(obstacles,"roundTable.obj",8,pos,rot,0.145, 0.4);
	
	rot[0] = 0;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=30;
	pos[1]=0;
	pos[2]=10;
	addFurnitureToRooms(obstacles,"roundTable.obj",8,pos,rot,0.145, 0.4);

//Room 7: Small room ---------------------------------------------------------------------------------------	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=22;
	pos[1]=0.1;
	pos[2]=5.5;
	addFurnitureToRooms(obstacles,"a3watjug.obj",7,pos,rot,0.02, 0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]= 26.5;
	pos[1]= 0.05;
	pos[2]= 5.5;
	addFurnitureToRooms(obstacles,"trashcan.obj",7,pos,rot,0.01,0.4);
	
	rot[0] = 0;
	rot[1] = -90; //orient
	rot[2] = 0;
	pos[0]= 26.5;
	pos[1]= 0.05;
	pos[2]= 7;
	addFurnitureToRooms(obstacles,"server.obj",7,pos,rot,0.01,0.4);

	rot[0] = 0;
	rot[1] = 0; //orientation
	rot[2] = 0;
	pos[0]= 21;
	pos[1]= 0.05;
	pos[2]= 8;
	addFurnitureToRooms(obstacles,"longtable.obj",7,pos,rot,0.01,0.4);
//Room 9: ---------------------------------------------------------------------------------------------------------
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]= 36.5;
	pos[1]= 0.1;
	pos[2]= 5.5;
	addFurnitureToRooms(obstacles,"guest_seating.obj",9,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]= 40;
	pos[1]=0.7;
	pos[2]=5.8;
	addFurnitureToRooms(obstacles,"m_plnt5.obj",9,pos,rot,0.04, 0.3);

	rot[0] = 0;
	rot[1] = 180;  //orientation
	rot[2] = 0;
	pos[0]=40;
	pos[1]=1.1;
	pos[2]=10;
	addFurnitureToRooms(false,"Monitor.obj",9,pos,rot,0.09, 0.4);
	
	rot[0] = 0;
	rot[1] = 180; //orientation
	rot[2] = 0;
	pos[0]= 40;
	pos[1]= 0.8;
	pos[2]= 10.2;
	addFurnitureToRooms(false,"keyboard.obj",9,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 180;  //orientation of desk
	pos[0]=40;
	pos[1]=0;
	pos[2]=10;
	addFurnitureToRooms(obstacles,"desk.obj",9,pos,rot,0.012, 0.4);  //radius is automatically calculated now
	
	rot[0] = -90;
	rot[1] = 0;  
	rot[2] = 0; //orientation?
	pos[0]=40;
	pos[1]=0;
	pos[2]=11;
	addFurnitureToRooms(obstacles,"office_chair.obj",9,pos,rot,0.01,0.4);

//Room 12: Office ----------------------------------------------------------------------------------------------------
	rot[0] = 0;
	rot[1] = 180;  //orientation
	rot[2] = 0;
	pos[0]=36;
	pos[1]=1.1;
	pos[2]=17;
	addFurnitureToRooms(false,"Monitor.obj",12,pos,rot,0.09, 0.4);
	
	rot[0] = 0;
	rot[1] = 180; //orientation
	rot[2] = 0;
	pos[0]= 36;
	pos[1]= 0.8;
	pos[2]= 17.2;
	addFurnitureToRooms(false,"keyboard.obj",12,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 180;  //orientation of desk
	pos[0]=36;
	pos[1]=0;
	pos[2]=17;
	addFurnitureToRooms(obstacles,"desk.obj",12,pos,rot,0.012, 0.4);  //radius is automatically calculated now
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=36;
	pos[1]=0;
	pos[2]=18;
	addFurnitureToRooms(obstacles,"office_chair.obj",12,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 90; //orientation
	pos[0]= 35.4;
	pos[1]= 0.85;
	pos[2]= 16.9;
	addFurnitureToRooms(false,"desklamp.obj",12,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]= 34;
	pos[1]= 0.05;
	pos[2]= 18.5;
	addFurnitureToRooms(obstacles,"4_drawer_file.obj",12,pos,rot,0.01,0.4);

//Room 16: Office -------------------------------------------------------------------------------------------

	rot[0] = 0;
	rot[1] = 0;  //orientation
	rot[2] = 0;
	pos[0]=17;
	pos[1]=1.1;
	pos[2]=21;
	addFurnitureToRooms(false,"Monitor.obj",14,pos,rot,0.09, 0.4);
	
	rot[0] = 0;
	rot[1] = 0; //orientation
	rot[2] = 0;
	pos[0]= 17;
	pos[1]= 0.8;
	pos[2]= 20.8;
	addFurnitureToRooms(false,"keyboard.obj",14,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;  //orientation of desk
	pos[0]=17;
	pos[1]=0;
	pos[2]=21;
	addFurnitureToRooms(obstacles,"desk.obj",14,pos,rot,0.012, 0.4);  //radius is automatically calculated now
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=17;
	pos[1]=0;
	pos[2]=20;
	addFurnitureToRooms(obstacles,"office_chair.obj",14,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = -90; //orientation
	pos[0]= 17.5;
	pos[1]= 0.85;
	pos[2]= 20.9;
	addFurnitureToRooms(false,"desklamp.obj",14,pos,rot,0.01,0.4);

	rot[0] = -90; 
	rot[1] = 0;
	rot[2] = 0;
	pos[0]= 14;
	pos[1]= 0.05;
	pos[2]= 24.7;
	addFurnitureToRooms(obstacles,"4_drawer_file.obj",14,pos,rot,0.01,0.4);

//Room 16: ---------------------------------------------------------------------------------------------------------
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=18;
	pos[1]=0.3;
	pos[2]=30.1;
	addFurnitureToRooms(obstacles,"TB01.obj",16,pos,rot,0.02,0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 180;  //orientation
	pos[0]= 16;
	pos[1]= 0.1;
	pos[2]= 30.5;
	addFurnitureToRooms(obstacles,"guest_seating.obj",16,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]= 20.2;
	pos[1]=0.7;
	pos[2]=25.8;
	addFurnitureToRooms(obstacles,"m_plnt5.obj",16,pos,rot,0.04, 0.3);
//Room 15: Big meeting room -----------------------------------------------------------------------------	

	rot[0] = 0;
	rot[1] = 90;
	rot[2] = 0;
	pos[0]= 25;
	pos[1]= 0.4;
	pos[2]= 21;
	addFurnitureToRooms(obstacles,"frntblm2.obj",15,pos,rot,0.04,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=24;
	pos[1]=0;
	pos[2]=20.5;
	addFurnitureToRooms(obstacles,"office_chair.obj",15,pos,rot,0.01,0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=24;
	pos[1]=0;
	pos[2]=21.5;
	addFurnitureToRooms(obstacles,"office_chair.obj",15,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 180;
	pos[0]=26;
	pos[1]=0;
	pos[2]=20.5;
	addFurnitureToRooms(obstacles,"office_chair.obj",15,pos,rot,0.01,0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 180;
	pos[0]=26;
	pos[1]=0;
	pos[2]=21.5;
	addFurnitureToRooms(obstacles,"office_chair.obj",15,pos,rot,0.01,0.4);

//---------------

	rot[0] = 0;
	rot[1] = 90;
	rot[2] = 0;
	pos[0]= 31;
	pos[1]= 0.4;
	pos[2]= 21;
	addFurnitureToRooms(obstacles,"frntblm2.obj",15,pos,rot,0.04,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=30;
	pos[1]=0;
	pos[2]=20.5;
	addFurnitureToRooms(obstacles,"office_chair.obj",15,pos,rot,0.01,0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=30;
	pos[1]=0;
	pos[2]=21.5;
	addFurnitureToRooms(obstacles,"office_chair.obj",15,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 180;
	pos[0]=32;
	pos[1]=0;
	pos[2]=20.5;
	addFurnitureToRooms(obstacles,"office_chair.obj",15,pos,rot,0.01,0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 180;
	pos[0]=32;
	pos[1]=0;
	pos[2]=21.5;
	addFurnitureToRooms(obstacles,"office_chair.obj",15,pos,rot,0.01,0.4);

//---------------

	rot[0] = 0;
	rot[1] = 90;
	rot[2] = 0;
	pos[0]= 37;
	pos[1]= 0.4;
	pos[2]= 21;
	addFurnitureToRooms(obstacles,"frntblm2.obj",15,pos,rot,0.04,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=36;
	pos[1]=0;
	pos[2]=20.5;
	addFurnitureToRooms(obstacles,"office_chair.obj",15,pos,rot,0.01,0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=36;
	pos[1]=0;
	pos[2]=21.5;
	addFurnitureToRooms(obstacles,"office_chair.obj",15,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 180;
	pos[0]=38;
	pos[1]=0;
	pos[2]=20.5;
	addFurnitureToRooms(obstacles,"office_chair.obj",15,pos,rot,0.01,0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 180;
	pos[0]=38;
	pos[1]=0;
	pos[2]=21.5;
	addFurnitureToRooms(obstacles,"office_chair.obj",15,pos,rot,0.01,0.4);
//Room: 17 ------------------------------------------------------------------------------------------------	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=22;
	pos[1]=0.1;
	pos[2]=25.5;
	addFurnitureToRooms(obstacles,"a3watjug.obj",17,pos,rot,0.02, 0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]= 26.5;
	pos[1]= 0.05;
	pos[2]= 25.5;
	addFurnitureToRooms(obstacles,"trashcan.obj",17,pos,rot,0.01,0.4);
	
	rot[0] = 0;
	rot[1] = 0; //orientation
	rot[2] = 0;
	pos[0]= 21;
	pos[1]= 0.05;
	pos[2]= 28;
	addFurnitureToRooms(obstacles,"longtable.obj",17,pos,rot,0.01,0.4);
//Room 18: ------------------------------------------------------------------------------------------------	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=31;
	pos[1]=0.05;
	pos[2]=38.5;
	addFurnitureToRooms(obstacles,"CH04.obj",18,pos,rot,0.01,0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=28;
	pos[1]=0.05;
	pos[2]=38.5;
	addFurnitureToRooms(obstacles,"CH04.obj",18,pos,rot,0.01,0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=29;
	pos[1]=0.1;
	pos[2]=38.5;
	addFurnitureToRooms(obstacles,"SF03.obj",18,pos,rot,0.01,0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=29.5;
	pos[1]=0.3;
	pos[2]=36.5;
	addFurnitureToRooms(obstacles,"TB01.obj",18,pos,rot,0.02,0.4);

//--------------

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0; 
	pos[0]=27.4;
	pos[1]=0.3;
	pos[2]=26.5;
	addFurnitureToRooms(obstacles,"TB01.obj",18,pos,rot,0.02,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]= 29.5;
	pos[1]=0.7;
	pos[2]=25.8;
	addFurnitureToRooms(obstacles,"m_plnt5.obj",18,pos,rot,0.04, 0.3);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = -90; //orientation
	pos[0]=27.4;
	pos[1]=0.05;
	pos[2]=28;
	addFurnitureToRooms(obstacles,"CH04.obj",18,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]= 27.7;
	pos[1]=0.7;
	pos[2]=29;
	addFurnitureToRooms(obstacles,"m_plnt5.obj",18,pos,rot,0.04, 0.3);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = -90; //orientation
	pos[0]=27.4;
	pos[1]=0.05;
	pos[2]=30;
	addFurnitureToRooms(obstacles,"CH04.obj",18,pos,rot,0.01,0.4);
//------
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=25;
	pos[1]=0.05;
	pos[2]=38.5;
	addFurnitureToRooms(obstacles,"CH04.obj",18,pos,rot,0.01,0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=22;
	pos[1]=0.05;
	pos[2]=38.5;
	addFurnitureToRooms(obstacles,"CH04.obj",18,pos,rot,0.01,0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=23;
	pos[1]=0.1;
	pos[2]=38.5;
	addFurnitureToRooms(obstacles,"SF03.obj",18,pos,rot,0.01,0.4);
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=23.5;
	pos[1]=0.3;
	pos[2]=36.5;
	addFurnitureToRooms(obstacles,"TB01.obj",18,pos,rot,0.02,0.4);
//------
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 90; //orientation
	pos[0]= 32.7;
	pos[1]=0.05;
	pos[2]=32;
	addFurnitureToRooms(obstacles,"CH04.obj",18,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 90; //orientation
	pos[0]= 32.7;
	pos[1]=0.05;
	pos[2]=34;
	addFurnitureToRooms(obstacles,"CH04.obj",18,pos,rot,0.01,0.4);

//Room 19: -----------------------------------------------------------------------------------------------	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]= 36.5;
	pos[1]= 0.1;
	pos[2]= 25.5;
	addFurnitureToRooms(obstacles,"guest_seating.obj",19,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]= 40;
	pos[1]=0.7;
	pos[2]=25.8;
	addFurnitureToRooms(obstacles,"m_plnt5.obj",19,pos,rot,0.04, 0.3);

	rot[0] = 0;
	rot[1] = 180;  //orientation
	rot[2] = 0;
	pos[0]=40;
	pos[1]=1.1;
	pos[2]=30;
	addFurnitureToRooms(false,"Monitor.obj",19,pos,rot,0.09, 0.4);
	
	rot[0] = 0;
	rot[1] = 180; //orientation
	rot[2] = 0;
	pos[0]= 40;
	pos[1]= 0.8;
	pos[2]= 30.2;
	addFurnitureToRooms(false,"keyboard.obj",19,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 180;  //orientation of desk
	pos[0]=40;
	pos[1]=0;
	pos[2]=30;
	addFurnitureToRooms(obstacles,"desk.obj",19,pos,rot,0.012, 0.4);  //radius is automatically calculated now
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 180; //orientation
	pos[0]=40;
	pos[1]=0;
	pos[2]=31;
	addFurnitureToRooms(obstacles,"office_chair.obj",19,pos,rot,0.01,0.4);

//Room 22: Office -----------------------------------------------------------------------------------------------
	rot[0] = 0;
	rot[1] = 180;  //orientation
	rot[2] = 0;
	pos[0]=36;
	pos[1]=1.1;
	pos[2]=37;
	addFurnitureToRooms(false,"Monitor.obj",22,pos,rot,0.09, 0.4);
	
	rot[0] = 0;
	rot[1] = 180; //orientation
	rot[2] = 0;
	pos[0]= 36;
	pos[1]= 0.8;
	pos[2]= 37.2;
	addFurnitureToRooms(false,"keyboard.obj",22,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 180;  //orientation of desk
	pos[0]=36;
	pos[1]=0;
	pos[2]=37;
	addFurnitureToRooms(obstacles,"desk.obj",22,pos,rot,0.012, 0.4);  //radius is automatically calculated now
	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]=36;
	pos[1]=0;
	pos[2]=38;
	addFurnitureToRooms(obstacles,"office_chair.obj",22,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 90; //orientation
	pos[0]= 35.4;
	pos[1]= 0.85;
	pos[2]= 36.9;
	addFurnitureToRooms(false,"desklamp.obj",22,pos,rot,0.01,0.4);

	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]= 34;
	pos[1]= 0.05;
	pos[2]= 38.5;
	addFurnitureToRooms(obstacles,"4_drawer_file.obj",22,pos,rot,0.01,0.4);

//---------------------------------------------------------------------------------------------------------	
	rot[0] = -90;
	rot[1] = 0;
	rot[2] = 0;
	pos[0]= 11;
	pos[1]= 2.5;
	pos[2]= 38.5;
	addFurnitureToRooms(false,"exit_sign.obj",5,pos,rot,0.01,0.4);

//---------------------------------------------------------------------------------------------------------	
	//rot[0] = 0;
	//rot[1] = 0;
	//rot[2] = 0;
	//pos[0]=25;
	//pos[1]=0;
	//pos[2]=15;
	//addFurnitureToRooms(obstacles,"roundTable.obj",8,pos,rot,0.145, 0.4);
	//rot[0] = -90;
	//rot[1] = 0;
	//rot[2] = 0;
	//pos[0]=28;
	//pos[1]=0.05;
	//pos[2]=18;
	//addFurnitureToRooms(obstacles,"CH04.obj",8,pos,rot,0.01,0.4);
	//rot[0] = -90;
	//rot[1] = 0;
	//rot[2] = 0;
	//pos[0]=30;
	//pos[1]=0.1;
	//pos[2]=15;
	//addFurnitureToRooms(obstacles,"SF03.obj",8,pos,rot,0.01,0.4);
	//rot[0] = -90;
	//rot[1] = 0;
	//rot[2] = 0;
	//pos[0]=31;
	//pos[1]=0.3;
	//pos[2]=12;
	//addFurnitureToRooms(obstacles,"TB01.obj",8,pos,rot,0.02,0.4);
	//rot[0] = -90;
	//rot[1] = 0;
	//rot[2] = 0;
	//pos[0]=29;
	//pos[1]=0.1;
	//pos[2]=12;
	//addFurnitureToRooms(obstacles,"guest_seating.obj",8,pos,rot,0.01,0.4);
	//rot[0] = -90;
	//rot[1] = 0;
	//rot[2] = 0;
	//pos[0]=32;
	//pos[1]=0.1;
	//pos[2]=8;
	//addFurnitureToRooms(obstacles,"a3watjug.obj",8,pos,rot,0.02, 0.4);
	//rot[0] = -90;
	//rot[1] = 0;
	//rot[2] = 0;
	//pos[0]= 18;
	//pos[1]= 0.05;
	//pos[2]= 15;
	//addFurnitureToRooms(obstacles,"2_drawer_file.obj",6,pos,rot,0.01,0.4);
	//rot[0] = -90;
	//rot[1] = 0;
	//rot[2] = 0;
	//pos[0]= 17;
	//pos[1]= 0.05;
	//pos[2]= 15;
	//addFurnitureToRooms(obstacles,"4_drawer_file.obj",6,pos,rot,0.01,0.4);
	//rot[0] = -90;
	//rot[1] = 0;
	//rot[2] = 0;
	//pos[0]= 16;
	//pos[1]= 0.05;
	//pos[2]= 15;
	//addFurnitureToRooms(obstacles,"trashcan.obj",6,pos,rot,0.01,0.4);
	//rot[0] = 0;
	//rot[1] = 90; //orient
	//rot[2] = 0;
	//pos[0]= 14;
	//pos[1]= 0.05;
	//pos[2]= 17;
	//addFurnitureToRooms(obstacles,"controlComputer.obj",6,pos,rot,0.01,0.4);
	//rot[0] = 0;
	//rot[1] = 90; //orient
	//rot[2] = 0;
	//pos[0]= 13;
	//pos[1]= 0.05;
	//pos[2]= 17;
	//addFurnitureToRooms(obstacles,"server.obj",6,pos,rot,0.01,0.4);
	//rot[0] = -90;
	//rot[1] = 0;
	//rot[2] = 0;
	//pos[0]= 16;
	//pos[1]= 0.15;
	//pos[2]= 17;
	//addFurnitureToRooms(obstacles,"desklamp.obj",6,pos,rot,0.01,0.4);
	//rot[0] = 0;
	//rot[1] = 0;
	//rot[2] = 0;
	//pos[0]= 16;
	//pos[1]= 0.15;
	//pos[2]= 16;
	//addFurnitureToRooms(obstacles,"keyboard.obj",6,pos,rot,0.01,0.4);
	//rot[0] = 0;
	//rot[1] = 0;
	//rot[2] = 0;
	//pos[0]= 14;
	//pos[1]= 0;
	//pos[2]= 12;
	//addFurnitureToRooms(obstacles,"cup.obj",6,pos,rot,0.1,0.4);
	//rot[0] = 90;
	//rot[1] = 0;
	//rot[2] = 0;
	//pos[0]= 15;
	//pos[1]= 1;
	//pos[2]= 16;
	//addFurnitureToRooms(obstacles,"briefcase.obj",6,pos,rot,0.006,0.4);
	//rot[0] = -90;
	//rot[1] = 0;
	//rot[2] = 0;
	//pos[0]= 15;
	//pos[1]= 2.5;
	//pos[2]= 12;
	//addFurnitureToRooms(obstacles,"exit_sign.obj",6,pos,rot,0.01,0.4);
	
	//addFurnitureToRooms(obstacles,"folder.obj",1,pos,rot,0.4);
	//addFurnitureToRooms(obstacles,"notebook.obj",1,pos,rot,0.4);
	//addFurnitureToRooms(obstacles,"paper.obj",1,pos,rot,0.4);
	//addFurnitureToRooms(obstacles,"projector.obj",1,pos,rot,0.4);
	//addFurnitureToRooms(obstacles,"thermos.obj",1,pos,rot,0.4);
	//addFurnitureToRooms(obstacles,"elchncm1.obj",1,pos,rot,0.4);
	//addFurnitureToRooms(obstacles,"in_out_box.obj",1,pos,rot,0.4);
	//addFurnitureToRooms(obstacles,"marker.obj",1,pos,rot,0.4);
	//addFurnitureToRooms(obstacles,"blackpen.obj",1,pos,rot,0.4);



//	rot[1] = 20;
//	pos[0]=14;
//	pos[1]=0;
//	pos[2]=17;
//	addFurnitureToRooms(obstacles,"roundTable.obj",5,pos,rot,0.4);

//
//	rot[1] = 20;
//	pos[0]=10;
//	pos[1]=0;
//	pos[2]=17;
//	addFurnitureToRooms(obstacles,"roundTable.obj",5,pos,rot,0.4);
//
//	rot[1] = 20;
//	pos[0]=14;
//	pos[1]=0;
//	pos[2]=14;
//	addFurnitureToRooms(obstacles,"roundTable.obj",5,pos,rot,0.4);
//
////////////////////////////////////	
//	rot[1] = 20;
//	pos[0]=20;
//	pos[1]=0;
//	pos[2]=14;
//	addFurnitureToRooms(obstacles,"roundTable.obj",5,pos,rot,0.4);
//
//	rot[1] = 20;
//	pos[0]=24;
//	pos[1]=0;
//	pos[2]=17;
//	addFurnitureToRooms(obstacles,"roundTable.obj",5,pos,rot,0.4);
//
//	rot[1] = 20;
//	pos[0]=20;
//	pos[1]=0;
//	pos[2]=17;
//	addFurnitureToRooms(obstacles,"roundTable.obj",5,pos,rot,0.4);
//
//	rot[1] = 20;
//	pos[0]=24;
//	pos[1]=0;
//	pos[2]=14;
//	addFurnitureToRooms(obstacles,"roundTable.obj",5,pos,rot,0.4);

}
*/
list<sOBSTACLE> includeChairs(double* centerTable, double radio)
{
	list<sOBSTACLE> list_obst;
	
	sOBSTACLE obst;
	obst.radio = 0.2*radio;
	obst.renderBV = true; //false;
	obst.cylinder = true;
	obst.repulsion = true;		// repulsion is used to determine whether it's a normal obstacle, or is the BV of a set 
	

	obst.center[0] = centerTable[0] + 6.2;
	obst.center[1] = centerTable[1];
	obst.center[2] = centerTable[2] + 0;
	list_obst.push_back(obst);

	obst.center[0] = centerTable[0] + 3.1;
	obst.center[2] = centerTable[1] + 5.36;
	list_obst.push_back(obst);

	obst.center[0] = centerTable[0] + (-3.1);
	obst.center[2] = centerTable[1] + 5.4;
	list_obst.push_back(obst);

	obst.center[0] = centerTable[0] + (-6.2);
	obst.center[2] = centerTable[1] + 0;
	list_obst.push_back(obst);

	obst.center[0] = centerTable[0] + (-3.1);
	obst.center[2] = centerTable[1] + (-5.36);
	list_obst.push_back(obst);

	obst.center[0] = centerTable[0] + 3.1;
	obst.center[2] = centerTable[1] + (-5.4);
	list_obst.push_back(obst);
	
	return list_obst;
}


void Cbuilding::addFurnitureToRooms(bool obstacle, /*int typeObs*/const std::string &filename, int roomID, double* pos, double*rot, double scale, double radio)
{
	sOBSTACLE obst;
	copy(pos,&(obst.center[0]));
//	obst.objDL = new (CobjectsDL);
//	obst.objDL->Gen3DObjectList(typeObs, pos, rot);
	obst.staticModelInstance = new StaticModelInstance(filename, pos, rot, scale);
	//obst.radio = radio;
	ModelType *model = obst.staticModelInstance->s_models[filename];
	obst.radio = model->myRadius * scale;

	obst.renderBV = false;
	obst.cylinder = true;
	obst.repulsion = true;		// repulsion is used to determine whether it's a normal obstacle, or is the BV of a set 
								// of obstacles, and thus no collision response applies, instead we need to look within the
								// hierarchy of obstacles

	obst.isFire = false;
	
	if (filename == "roundTable.obj")  // the big round tables have chairs to include within the main obstacle
	{
		obst.repulsion = false;
		obst.list_obstacles = includeChairs(pos,radio);
	}
	

	if (obstacle)  
		getCell(roomID)->addObstacle(obst);
	else
		getCell(roomID)->addFurniture(obst);
}



// This function will read all the relevant information from the Cell and Portal Graph and create all the building
// information and all the cells with its correspon	ding lists of walls and portals.
void Cbuilding::createEnvironment(CPG *cpg)
{
	map<int,Scell*> cpg_tableCells = cpg->getTableCells();
	list<Sportal*> cpg_listPortals = cpg->getListPortals();
	list<Sportal*>::iterator iter_portals;

	map<int,int> adjCells; // each portal ID leads to a CellID. The size of this table gives the number of portals in the cell
	list<sWALL> listWalls;	// List of walls for this portal;
	list<sOBSTACLE> listObst;
	map<int,sPORTAL> tablePortals;		// List of portals for this cell. Note that the portals IDs are unique in the building
	sWALL wall;
	sOBSTACLE obst;
	sPORTAL portal;
	sSTAIR stair;
	double length;
	list<Slandmarks>::iterator iter_landm;

	int cellID;
	Scell* cpg_pcell;

	CCell* pcell;


	for (iter_portals = cpg_listPortals.begin(); iter_portals != cpg_listPortals.end(); iter_portals++)
	{
		m_tablePortals[(*iter_portals)->id].cellAid = (*iter_portals)->cellA_id;
		m_tablePortals[(*iter_portals)->id].cellBid = (*iter_portals)->cellB_id;
		m_tablePortals[(*iter_portals)->id].open = true;
		m_tablePortals[(*iter_portals)->id].m_lhumCrossing.clear();
		m_tablePortals[(*iter_portals)->id].posCenter[0] = (*iter_portals)->posA.x + ((*iter_portals)->posB.x-(*iter_portals)->posA.x)*0.5;
		m_tablePortals[(*iter_portals)->id].posCenter[1] = (*iter_portals)->posA.y + ((*iter_portals)->posB.y-(*iter_portals)->posA.y)*0.5;
		m_tablePortals[(*iter_portals)->id].posCenter[2] = (*iter_portals)->posA.z + ((*iter_portals)->posB.z-(*iter_portals)->posA.z)*0.5;
		m_tablePortals[(*iter_portals)->id].ncrossed = 0;
		m_tablePortals[(*iter_portals)->id].accDensity = 0;
		
		//printf("portal %d \t %d \t %d \n",(*iter_portals)->id,m_tablePortals[(*iter_portals)->id].cellAid ,m_tablePortals[(*iter_portals)->id].cellBid );
	}

	// we need to create one by one all the cells, then for each cell store the portal information
	// Finally add to the hash table m_building the cell id with the pointer to that new cell
	map<int,Scell*>::iterator iter_cells;
	//list<Sportal*>::iterator iter_portals;
	list<Swall>::iterator iter_walls;
	list<Sobstacle_R>::iterator iter_obst;
	map<int,tVector> landmarks;
	tVector pos;

	// go through all the cells
	for (iter_cells = cpg_tableCells.begin(); iter_cells != cpg_tableCells.end(); iter_cells++)
	{
		// Create a new CCell:
		pcell = new CCell;
		adjCells.clear();
		listWalls.clear();
		tablePortals.clear();
		listObst.clear();
		landmarks.clear();
	

		//for each cell gather all the information regarding portals and walls
		cellID = (*iter_cells).first;
		cpg_pcell  = (*iter_cells).second;


		if (cellID == 42)
				int stop = 0;

		m_building[cellID] = pcell;
		pcell->setName(cellID);

		// If it's a stair, i need to uptade also the info regarding stairs:
		if (cpg_pcell->type == 1)
		{
			stair.floorBe = cpg_pcell->stair->floorBe;
			stair.floorAb = cpg_pcell->stair->floorAb;
			stair.posCenterAb[0] = cpg_pcell->stair->posCenterAb.x;
			stair.posCenterAb[1] = cpg_pcell->stair->posCenterAb.y;
			stair.posCenterAb[2] = cpg_pcell->stair->posCenterAb.z;
			stair.posCenterBe[0] = cpg_pcell->stair->posCenterBe.x;
			stair.posCenterBe[1] = cpg_pcell->stair->posCenterBe.y;
			stair.posCenterBe[2] = cpg_pcell->stair->posCenterBe.z;
			stair.numSteps = cpg_pcell->stair->numSteps;
			pcell->setAsStair(stair);
		}


		// copy all the information about portals
		for (iter_portals = cpg_pcell->portals.begin(); iter_portals != cpg_pcell->portals.end(); iter_portals++)
		{
			portal.id = (*iter_portals)->id;
			//portal.open = true;

			if (cellID == (*iter_portals)->cellA_id)
			{
				portal.nextcellID = (*iter_portals)->cellB_id;  // if i'm in cellA then next cell is cellB				
				portal.pos[0]  = (*iter_portals)->posA.x;
				portal.pos[1]  = (*iter_portals)->posA.y;
				portal.pos[2]  = (*iter_portals)->posA.z;

				portal.doorClosed.pointA[0] = (*iter_portals)->wallDoorClosed.endA.x;
				portal.doorClosed.pointA[1] = (*iter_portals)->wallDoorClosed.endA.y;
				portal.doorClosed.pointA[2] = (*iter_portals)->wallDoorClosed.endA.z;
				portal.doorClosed.pointB[0] = (*iter_portals)->wallDoorClosed.endB.x;
				portal.doorClosed.pointB[1] = (*iter_portals)->wallDoorClosed.endB.y;
				portal.doorClosed.pointB[2] = (*iter_portals)->wallDoorClosed.endB.z;
			}
			else
			{
				portal.nextcellID = (*iter_portals)->cellA_id;  // otherwise, next cell is cellA
				portal.pos[0]  = (*iter_portals)->posB.x;
				portal.pos[1]  = (*iter_portals)->posB.y;
				portal.pos[2]  = (*iter_portals)->posB.z;

				portal.doorClosed.pointA[0] = (*iter_portals)->wallDoorClosed.endB.x;
				portal.doorClosed.pointA[1] = (*iter_portals)->wallDoorClosed.endB.y;
				portal.doorClosed.pointA[2] = (*iter_portals)->wallDoorClosed.endB.z;
				portal.doorClosed.pointB[0] = (*iter_portals)->wallDoorClosed.endA.x;
				portal.doorClosed.pointB[1] = (*iter_portals)->wallDoorClosed.endA.y;
				portal.doorClosed.pointB[2] = (*iter_portals)->wallDoorClosed.endA.z;
			}
			portal.exit = (*iter_portals)->exit;
			//printf("portal2 = %d, %f, %f, %f\n", portal.id, portal.pos[0], portal.pos[1], portal.pos[2]);
			
			// those exits with a portal towards and exit should be marked as exit rooms
			if (portal.exit == true)
			{
				exitsMaze[nexitsBuilding] = cellID;
				portal.nextcellID = -1;	// -1 means exit
				nexitsBuilding++;
			}

//			pos.x = portal.pos[0];
//			pos.y = portal.pos[1];
//			pos.z = portal.pos[2];
//			landmarks[portal.id] = pos;

//			portal.numPeopleCrossing = 0;

			tablePortals[portal.id] = portal;	// for each portal id i get all the information
			// adjCells[portal.id] = portal.nextcellID; REDUNDANT INFO!!! // With portal = portal.id  i get to cell is id = nextcellID
		}

		// copy all the information about walls
		for (iter_walls = cpg_pcell->walls.begin(); iter_walls != cpg_pcell->walls.end(); iter_walls++)
		{
			wall.pointA[0] = (*iter_walls).endA.x;
			wall.pointA[1] = (*iter_walls).endA.y;
			wall.pointA[2] = (*iter_walls).endA.z;

			wall.pointB[0] = (*iter_walls).endB.x;
			wall.pointB[1] = (*iter_walls).endB.y;
			wall.pointB[2] = (*iter_walls).endB.z;

			wall.thikness = (*iter_walls).thikness;

			// Wall normal is the cross product: UPx(B-A), where UP={0,1,0}
			length = sqrt((wall.pointB[2] - wall.pointA[2])*(wall.pointB[2] - wall.pointA[2]) + (wall.pointB[0] - wall.pointA[0])*(wall.pointB[0] - wall.pointA[0]));
			wall.normal[0] = (wall.pointB[2] - wall.pointA[2]) / length ;
			wall.normal[1] = 0.0;
			wall.normal[2] = -(wall.pointB[0] - wall.pointA[0]) / length ;
			wall.D = - (wall.pointA[0]*wall.normal[0]+wall.pointA[1]*wall.normal[1]+wall.pointA[2]*wall.normal[2]);

			obst.radio = wall.thikness/3.0; //0.05 ;//wall.thikness-0.25;			// OJO i reduced the radio of this obstacles to avoid agents getting stuck in the door frame! 14May07
			obst.renderBV = true;
			obst.cylinder = true;
			obst.center[0] = wall.pointA[0];
			obst.center[1] = wall.pointA[1];
			obst.center[2] = wall.pointA[2];
			listObst.push_back(obst);
			obst.center[0] = wall.pointB[0];
			obst.center[1] = wall.pointB[1];
			obst.center[2] = wall.pointB[2];
			listObst.push_back(obst);


			listWalls.push_back(wall);
			
		}

		for (iter_obst = cpg_pcell->obst_R.begin(); iter_obst != cpg_pcell->obst_R.end(); iter_obst++)
		{
			obst.center[0] = (*iter_obst).center.x;
			obst.center[1] = (*iter_obst).center.y;
			obst.center[2] = (*iter_obst).center.z;
			obst.radio     = (*iter_obst).radio;
			obst.renderBV  = true;
			obst.cylinder = true;
//			obst.objDL     = NULL;
			obst.staticModelInstance = NULL;
			listObst.push_back(obst);
		}

		
		for (iter_landm = cpg_pcell->listLandmarks.begin(); iter_landm != cpg_pcell->listLandmarks.end(); iter_landm++)
		{
			pos.x = (*iter_landm).pos.x;
			pos.y = (*iter_landm).pos.y;
			pos.z = (*iter_landm).pos.z;
			landmarks[(*iter_landm).id] = pos; 
		}

		
//		pcell->setAdjCells(adjCells);	 NOT ANYMORE, IT WAS REDUNDANT INFO
		pcell->setListWalls(listWalls);
		pcell->setListObstacles(listObst);
		pcell->setTablePortals(tablePortals);
		pcell->setListLandmarks(landmarks);
		pcell->setEnvironment(this);
	}

	// Now that i know the total number of exits in the building:
	for (iter_cells = cpg_tableCells.begin(); iter_cells != cpg_tableCells.end(); iter_cells++)
	{
		// PRUEBA
		if ((*iter_cells).first < 0)
			int n=0;
		m_building[(*iter_cells).first]->initPathsExits(nexitsBuilding);
	}
	
	createPaths();

	
	// DEBUG:
	list<pairIDs>::iterator iter_path;
	FILE* fd;
	fopen_s(&fd, "pathsBuilding.txt","w");

	for (int d=0; d<nexitsBuilding; d++)
	{
		fprintf(fd,"EXIT: %d",d);
		for (iter_cells = cpg_tableCells.begin(); iter_cells != cpg_tableCells.end(); iter_cells++)
		{
			path2exit path;

			
			path = m_building[(*iter_cells).first]->getPath(d);

			fprintf(fd,"\n");
			fprintf(fd, "Cell id= %3d, length=%4f",(*iter_cells).first,path.length);
			
			for (iter_path = path.path.begin(); iter_path != path.path.end(); iter_path++)
			{
				fprintf(fd,"p: %3d , c: %3d ;     ",(*iter_path).idNextPortal,(*iter_path).idCell);
			}
		}
		fprintf(fd,"\n\n\n");
	}
	fclose(fd);
    
	// END DEBUG
	
};

	
// This function creates the tiles and assigns the  values from the CPG, which are 0 if there's a wall, -1 if there's a door
// and a positive integer if it's inside a room. That positive integer corresponds to the room name
void Cbuilding::createTiles(CPG *cpg)
{
//	int f,x,z;

	nfloors = cpg->getNumFoors();
	sizeX = cpg->getSizeX();
	sizeZ = cpg->getSizeZ();

	m_tiles = new int** [nfloors];
	for (int f=0; f<nfloors; f++)
	{
		m_tiles[f] = new int* [sizeX];
		for (int x=0; x<sizeX; x++)
		{
			m_tiles[f][x] = new int [sizeZ];
			for (int z=0; z<sizeZ; z++)
			{
				m_tiles[f][x][z] = cpg->getAuxGridVal(x,z,f);  
			}

		}
	}
};

void drawNormal(sWALL wall)
{
	double med[3];

	for (int i=0; i<3; i++)
		med[i] = wall.pointA[i] + (wall.pointB[i] - wall.pointA[i])*0.5;

	glLineWidth(1);	
//	glBegin(GL_LINES);
	glVertex3f(med[0],med[1],med[2]);
	glColor3f(1.0, 1.0, 1.0); // Ending in White, so i can see the orientation of the vector	
	glVertex3f(med[0]+wall.normal[0],med[1]+wall.normal[1],med[2]+wall.normal[2]);
//	glEnd();
}

void Cbuilding::draw2D()
{
	map<int,CCell*>::iterator iter_cells;
	CCell* pcell;
//	sWALL* pwall;
	list<sWALL>	walls, laux;
	list<sWALL>::iterator iter_wall;	
	map<int,sPORTAL>::iterator iter_portal;
	map<int,sPORTAL> portal;
//	double pos[3];
	

	double red[] = {1.0, 0.0, 0.0};
	double green[] = {0.0, 1.0, 0.0};
	double blue[] = {0.0, 0.0, 1.0};
	double rb[] = {1.0, 0.0, 1.0};
	double rg[] = {1.0, 1.0, 0.0};
	double bg[] = {0.0, 1.0, 1.0};

	double* color[] = {red,green,blue,rg,rb,bg};

	int x=0;
	double linesize = 2;
	
	for (iter_cells = m_building.begin(); iter_cells != m_building.end(); iter_cells++)
	{
		pcell = (*iter_cells).second;
		glLineWidth(2);	
		//printf("cell -> %d",(*iter_cells).first);
		if (pcell->getHazard()->getType())
		{
			tVector step = getRandomPosInCell((*iter_cells).first);
			pcell->getHazard()->draw(step); 
		}

		
		//glColor3f(color[x][0],color[x][1],color[x][2]);
		x++;
		x = x % 6;
		
		walls = pcell->getListWalls();
		laux = pcell->getListWeakWalls();
		walls.splice(walls.end(),laux);
		glLineWidth(linesize);	
		glBegin(GL_LINES);
		for (iter_wall = walls.begin(); iter_wall != walls.end(); iter_wall++)
		{
				glColor3f(color[x][0],color[x][1],color[x][2]);
				glVertex3f((*iter_wall).pointA[0],-linesize,(*iter_wall).pointA[2]);
				glVertex3f((*iter_wall).pointB[0],-linesize,(*iter_wall).pointB[2]);

				//drawNormal((*iter_wall));
		}
		glEnd();


		#ifdef DEBUG_CROSSING_DOORS
		// This piece of code is for when i want to render the humans with the color of the cell, so that i know when they change color. 
		list<Chumanoid*> lhums = pcell->getListHums();
		list<Chumanoid*>::iterator iter_hum;
		tVector pos;
		
		
		for (iter_hum = lhums.begin(); iter_hum != lhums.end(); iter_hum++)
		{
			glBegin(GL_POINTS);
			pos = (*iter_hum)->getPosition();
			switch ((*iter_hum)->getCrossingState())
			{
				case NOT_CROSSING:
					glColor3f(1.0,1.0,1.0);
					break;
				case REACHING_A:
					glColor3f(1.0,1.0,0.0);
					break;
				case CLOSE_TO_A:
					glColor3f(0.0,1.0,1.0);
					break;
				case LEAVING_B:
					glColor3f(0.0,0.0,0.0);
					break;

			};
			pos = (*iter_hum)->getPosition();
			glVertex3f(pos.x,pos.y+2.5,pos.z);
			glEnd();	

			glColor3f(color[x][0],color[x][1],color[x][2]);
			(*iter_hum)->draw();
		}
		#endif
		
		portal = pcell->getTablePortals();		
		int portalID;
		for (iter_portal = portal.begin(); iter_portal != portal.end(); iter_portal++)
		{
			portalID = (*iter_portal).first;
			//if (!(*iter_portal).second.open)
			if (!(m_tablePortals[portalID].open))
			{
				glPointSize(8);
				glBegin(GL_POINTS);
					glColor3f(1.0, 0.0, 0.0);
					glPointSize(6);
					glVertex3f((*iter_portal).second.pos[0], (*iter_portal).second.pos[1], (*iter_portal).second.pos[2]);
				glEnd();
			}
			else
			{
				glPointSize(3);
				glBegin(GL_POINTS);
					glColor3f(0.0, 0.0, 0.0);
					glVertex3f((*iter_portal).second.pos[0], (*iter_portal).second.pos[1], (*iter_portal).second.pos[2]);
				glEnd();
			}
		}

		// draw Obstacles:
		pcell->drawObstacles();
	}
};

void Cbuilding::draw3D()
{
	map<int,CCell*>::iterator iter_cells;
	CCell* pcell;
//	sWALL* pwall;
	list<sWALL>	walls;
	list<sWALL>::iterator iter_wall;	
	map<int,sPORTAL>::iterator iter_portal;
	map<int,sPORTAL> portal;
//	double pos[3];
	map<int,PORTAL_INFO>::iterator m_portalInfo;
	

	double red[] = {1.0, 0.0, 0.0};
	double green[] = {0.0, 1.0, 0.0};
	double blue[] = {0.0, 0.0, 1.0};
	double rb[] = {1.0, 0.0, 1.0};
	double rg[] = {1.0, 1.0, 0.0};
	double bg[] = {0.0, 1.0, 1.0};
	double* color[] = {red,green,blue,rg,rb,bg};
	int x=0;
	double linesize = 2;

//	glDisable(GL_LIGHTING);
	m_building3D.draw();
//	glEnable(GL_LIGHTING);

	for (iter_cells = m_building.begin(); iter_cells != m_building.end(); iter_cells++)
	{
		if ((*iter_cells).second != NULL)
		{
			pcell = (*iter_cells).second;
			glLineWidth(2);	
			//printf("cell -> %d",(*iter_cells).first);
			if (pcell->getHazard()->getType())
			{
				tVector step = getRandomPosInCell((*iter_cells).first);
				pcell->getHazard()->draw(step); 
			}
	/*
			glLineWidth(linesize);	
			glBegin(GL_LINES);
			//glColor3f(color[x][0],color[x][1],color[x][2]);
			x++;
			x = x % 6;
			
			walls = pcell->getListWalls();
			for (iter_wall = walls.begin(); iter_wall != walls.end(); iter_wall++)
			{
				//glVertex3f((*iter_wall).endA.x,(*iter_wall).endA.y,(*iter_wall).endA.z);
				//glVertex3f((*iter_wall).endB.x,(*iter_wall).endB.y,(*iter_wall).endB.z);
				glColor3f(color[x][0],color[x][1],color[x][2]);
				glVertex3f((*iter_wall).pointA[0],-linesize,(*iter_wall).pointA[2]);
				glVertex3f((*iter_wall).pointB[0],-linesize,(*iter_wall).pointB[2]);
				//pos = (*iter_wall).endA;
				//pos = (*iter_wall).endB;

				drawNormal((*iter_wall));

			}
			glEnd();
			//linesize -= 1.5;
			//if (linesize <= 1.6)
			//	linesize = 5;
	*/
			portal = pcell->getTablePortals();	
			
			for (iter_portal = portal.begin(); iter_portal != portal.end(); iter_portal++)
			{
	//			if (!(*iter_portal).second.open)
				if (!m_tablePortals[(*iter_portal).first].open)
				{
					Cbuilding3D building;
					
					glPushMatrix();
						glTranslatef(m_tablePortals[(*iter_portal).first].posCenter[0],m_tablePortals[(*iter_portal).first].posCenter[1]+HEIGHT_CELL*0.5,m_tablePortals[(*iter_portal).first].posCenter[2]);
						building.drawClosedDoor();
					glPopMatrix();
//					glPointSize(8);
//					glBegin(GL_POINTS);
//						glColor3f(1.0, 0.0, 0.0);
//						glPointSize(6);
//						glVertex3f((*iter_portal).second.pos[0], (*iter_portal).second.pos[1], (*iter_portal).second.pos[2]);
//					glEnd();
				}
//				else
//				{
//					glPointSize(3);
//					glBegin(GL_POINTS);
//						glColor3f(0.0, 0.0, 0.0);
///						glVertex3f((*iter_portal).second.pos[0], (*iter_portal).second.pos[1], (*iter_portal).second.pos[2]);
//					glEnd();
//				}
			}

			glDisable(GL_COLOR_MATERIAL);
			pcell->drawFurniture();
//			glEnable(GL_COLOR_MATERIAL);

			// draw Obstacles:
			//pcell->drawObstacles();
			#ifdef DEBUG_CROSSING_DOORS
			// draw Yellow color points on top of the humans crossing a portal:
			list<Chumanoid*>::iterator iter_lhums;
			tVector pos;
			for (m_portalInfo = m_tablePortals.begin(); m_portalInfo != m_tablePortals.end(); m_portalInfo++)
			{
				glPointSize(5);
				glBegin(GL_POINTS);
				//glColor3f(0.0,1.0,1.0);
				for (iter_lhums = (*m_portalInfo).second.m_lhumCrossing.begin(); iter_lhums != (*m_portalInfo).second.m_lhumCrossing.end(); iter_lhums++)
				{
					switch ((*iter_lhums)->getCrossingState())
					{
						case NOT_CROSSING:
							glColor3f(1.0,1.0,1.0);
							break;
						case REACHING_A:
							glColor3f(1.0,1.0,0.0);
							break;
						case CLOSE_TO_A:
							glColor3f(0.0,1.0,1.0);
							break;
						case LEAVING_B:
							glColor3f(0.0,0.0,0.0);
							break;
					};

					pos = (*iter_lhums)->getPosition();
					glVertex3f(pos.x,pos.y+2.5,pos.z);
				}
				glEnd();	
			}
			#endif
			glEnable(GL_COLOR_MATERIAL);

		}
	}
};


int Cbuilding::getNumRooms()
{
	return m_building.size();
};


CCell* Cbuilding::getCell(int cellID)
{
	// PRUEBA
		if (cellID == -1)
			return NULL;
	
		if (cellID == 0)
			int n=0;
		else if (cellID>=m_building.size())
			int n=0;

	return m_building[cellID];
};

void Cbuilding::addHazardInCell(int type, int IDroom)
{
	m_building[IDroom]->addHazard(type);
};

int Cbuilding::getNumExits()
{
	return nexitsBuilding;
};


void Cbuilding::deleteInfoCell()
{
	// Delete the information regarding the decisions taken.
	map<int,CCell*>::iterator iter;

	for (iter = m_building.begin(); iter != m_building.end(); iter++)
	{
		(*iter).second->resetDecision();
	}
};


tVector Cbuilding::getRandomPosInCell(int cellID)
{
	int f,x,z;
	tVector step;
	bool exit = false;
	int tile;
	int ntiles = 0;
	double error;
	bool done=false;
	bool inObst=false;

	if (WTC)
		tile = rand() % 3000;
	else
		tile = rand() % (sizeX * sizeZ)/2.0; //Jan changed to distribute better //15;

	for (f=0; f<nfloors; f++)
	{
		for (x=0; x<sizeX; x++)
		{
			for (z=0; z<sizeZ; z++)
			{
				if (m_tiles[f][x][z] == cellID)
				{
					ntiles ++;
					done = true;
					error = 0.3 * pow(-1.0,(rand()%2)+1);
					step.x = x*SIZE_CELL + SIZE_CELL*0.5 + error;
					step.y = f*HEIGHT_CELL;
					step.z = z*SIZE_CELL + SIZE_CELL*0.5 + error;

					if (ntiles == tile)
					{		
						if (!getCell(cellID)->intersectsObst(step))
							return (step);
						else
							inObst=true;
					}
				}	
			}
		}
	}

	// this will happen if the person is in the stairs
	if ((!done) || (inObst))
	{
		step.x = m_building[cellID]->getTablePortals().begin()->second.pos[0];
		step.y = m_building[cellID]->getTablePortals().begin()->second.pos[1];
		step.z = m_building[cellID]->getTablePortals().begin()->second.pos[2];
	}
	return (step);
};

int Cbuilding::getNumPortals()
{
	return m_tablePortals.size();
}

PORTAL_INFO Cbuilding::getPortalInfo(int portalID)
{
	if ((portalID>0) && (portalID<=m_tablePortals.size()))
		return m_tablePortals[portalID];
}

void Cbuilding::addHumCrossed(int portalID)
{
	if ((portalID>0) && (portalID<=m_tablePortals.size()))
		m_tablePortals[portalID].num_humsCrossed++;
	else
		int i=0;
}

int Cbuilding::getHumCrossed(int portalID)
{
	if ((portalID>0) && (portalID<=m_tablePortals.size()))
	{
		int a = m_tablePortals[portalID].num_humsCrossed;
		int b = m_tablePortals[portalID].num_humsCrossedprev;
		int diff = m_tablePortals[portalID].num_humsCrossed - m_tablePortals[portalID].num_humsCrossedprev;
		m_tablePortals[portalID].num_humsCrossedprev = m_tablePortals[portalID].num_humsCrossed;
		return diff;
	}
	else return 0;
}

void Cbuilding::addDensity(int portalID, double d)
{
	if ((portalID>0) && (portalID<=m_tablePortals.size()))
	{
		m_tablePortals[portalID].accDensity+=d; 
	}
}

double Cbuilding::getAccDensity(int portalID)
{
	if ((portalID>0) && (portalID<=m_tablePortals.size()))
	{
		return m_tablePortals[portalID].accDensity; 
	}
	return 0;
}

void Cbuilding::resetAccDensity(int portalID)
{
	if ((portalID>0) && (portalID<=m_tablePortals.size()))
	{
		m_tablePortals[portalID].accDensity = 0; 
	}
}

void Cbuilding::createPaths()
{
	int i, nportals;
	map<int,sPORTAL> tablePortals;
	map<int,sPORTAL>::iterator iterPortals;
	list<int> listCells;
	int currentCell,nextCell;
	bool change;

	bool *maze_discovered = new bool [m_building.size()];

	for (i=0; i<nexitsBuilding; i++)
	{
		nportals = m_building[exitsMaze[i]]->getNumPortals();
		tablePortals = m_building[exitsMaze[i]]->getTablePortals();
		
		// initiate the values:
		for (int k=1; k<=m_building.size(); k++)
			maze_discovered[k] = false;

		listCells.push_back(exitsMaze[i]);

		//add first the exit cell
		// find out the portal id that leads to the exit:
		for (iterPortals=tablePortals.begin(); iterPortals!=tablePortals.end(); iterPortals++) 	
			if ((*iterPortals).second.exit) // if the portal is an exit... then we add that 0 path
				m_building[exitsMaze[i]]->addCellPath(i,-1,(*iterPortals).first);

		while (!listCells.empty())
		{
			currentCell = listCells.front();
			listCells.pop_front();
			
			maze_discovered[currentCell] = true;

			nportals = m_building[currentCell]->getNumPortals();
			tablePortals = m_building[currentCell]->getTablePortals();

			// for each portal of the current cell calculate the length of the next cell
			for (iterPortals=tablePortals.begin(); iterPortals!=tablePortals.end(); iterPortals++) 	
			{
				// check to which cell the portal leads to and whether it's been discovered or not
				nextCell = ((*iterPortals).second).nextcellID;
				if ((!maze_discovered[nextCell]) && !((*iterPortals).second.exit)) // if it hasn't been discovered and the portal is not an exit
				{
					change = m_building[nextCell]->addCellPath(i,currentCell,(*iterPortals).first);
					//maze_discovered[nextCell] = true;
					if (change) // if change, it means it had 2 doors, and therefore it's already in the list
						listCells.push_back(nextCell);	
				}
			}
		}
	}
};

// Finds the portal that joins cellA with cellB
int  Cbuilding::findPortal(int cellA, int cellB)
{
	map<int,sPORTAL>::iterator iter_portals;
	map<int,sPORTAL> table_portals;

	if (cellA != -1)
	{
		table_portals = m_building[cellA]->getTablePortals();
		for (iter_portals = table_portals.begin(); iter_portals != table_portals.end(); iter_portals++)
			if ((*iter_portals).second.nextcellID == cellB)
				return (*iter_portals).first;
		// if no portal joining those 2 cells was found, then it returns -1
	}
	else
	{
		table_portals = m_building[cellB]->getTablePortals();
		for (iter_portals = table_portals.begin(); iter_portals != table_portals.end(); iter_portals++)
			if ((*iter_portals).second.nextcellID == -1)
				return (*iter_portals).first;
		
	}
	return -1;
};

void Cbuilding::printListHums(int portalID)
{
	list<Chumanoid*>::iterator iter;
	list<Chumanoid*> list;

	list = m_tablePortals[portalID].m_lhumCrossing;
	for (iter=list.begin(); iter != list.end(); iter++)
		cout << (*iter)->gethumID() << " ";
	cout << endl;
	
}

void Cbuilding::resetNumPeopleCrossingPortals()
{
	map<int,CCell*>::iterator iter_cells;

	for (iter_cells = m_building.begin(); iter_cells != m_building.end(); iter_cells++)
	{
		if ((*iter_cells).first != -1)
			if ((*iter_cells).second != NULL)
				(*iter_cells).second->updateTotalNumPeopleCrossing();
	}
}

/*
void Cbuilding::computeLandMarks()
{
	list<sCOORD> explore;
	map<sCOORD,bool> discovered;
	int nCells = m_building.size();
	map<sCOORD,sLANDMARKS> pathCoord;
	sLANDMARKS lm;
	int cellID;
	CCell *pcell;
	map<int,sPORTAL> tablePortals;

	map<int,sPORTAL>::iterator iter_p1, iter_p2, iter_aux;
	sCOORD p1, p2, pnext, pcurrent;
	tVector s1,s2;

	// m_cell_name will appear in those tiles that belong to this cell

	for (cellID=1; cellID<nCells; cellID++)
	{
		pcell = m_building[cellID];
		tablePortals = pcell->getTablePortals();
		for (iter_p1 = tablePortals.begin(); iter_p1 != tablePortals.end(); iter_p1++)
		{
			iter_aux = iter_p1;
			iter_aux++;
			for (iter_p2 = iter_aux; iter_p2 != tablePortals.end(); iter_p2++)
			{
				// TODO: IF THERE IS VISIBILITY BETWEEN P1 AND P2 THEN I DON'T NEED TO DO ANYTHING!!!!!
				if (!pcell->visible((*iter_p1).second.pos,(*iter_p1).second.pos))
				{
					p1.x = (*iter_p1).second.pos[0] / SIZE_CELL;
					p1.y = (*iter_p1).second.pos[1] / SIZE_CELL;
					p1.z = (*iter_p1).second.pos[2] / SIZE_CELL;
					p2.x = (*iter_p2).second.pos[0] / SIZE_CELL;
					p2.y = (*iter_p2).second.pos[1] / SIZE_CELL;
					p2.z = (*iter_p2).second.pos[2] / SIZE_CELL;

					// Check in the 4neighbourhood of the door for the tile that is inside cell i
					pnext = p1;
					if (p1.x>0)
					{
						if (m_tiles[p1.y][p1.x-1][p1.z] == cellID)
							pnext.x-=1;
					}
					if (p1.x<sizeX-1)
					{	
						if (m_tiles[p1.y][p1.x+1][p1.z] == cellID)
							pnext.x+=1;
					}
					if (p1.z>0)
					{
						if (m_tiles[p1.y][p1.x][p1.z-1] == cellID)
							pnext.z-=1;
					}
					if (p1.z<sizeZ-1)
					{	
						if (m_tiles[p1.y][p1.x][p1.z+1] == cellID)
							pnext.z+=1;
					}
					explore.push_back(pnext);
					
					s1.x = p1.x;
					s1.y = p1.y;
					s1.z = p1.x;
					lm.landMarks.push_back(s1);
					pathCoord[pnext] = lm;

					while (!explore.empty())
					{
						pcurrent = explore.front();
						explore.pop_front();

						//discovered.push_back(pcurrent);
						discovered[pcurrent] = true;

						// check the 4 neighbours to see which ones needs to be extended
						if (pcurrent.x>0)
						{
							if (m_tiles[pcurrent.y][pcurrent.x-1][pcurrent.z] == cellID)
							{
								pnext = pcurrent;
								pnext.x-=1;
								// if the cell hasn't been discovered yet
								if (discovered.find(pnext) == discovered.end())
								{
									explore.push_back(pnext);
									if (pathCoord[pnext].landMarks.size()+1 < pathCoord[pcurrent].landMarks.size())
									{
										s1.x = pcurrent.x;
										s1.y = pcurrent.y;
										s1.z = pcurrent.x;
										lm = pathCoord[pcurrent];
										lm.landMarks.push_back(s1);
										pathCoord[pnext] = lm;
									}
								}
								else // if it hasn't been discovered yet, then give it a path
								{
									s1.x = pcurrent.x;
									s1.y = pcurrent.y;
									s1.z = pcurrent.x;
									lm = pathCoord[pcurrent];
									lm.landMarks.push_back(s1);
									pathCoord[pnext] = lm;
								}
							}
						}
						if (pcurrent.x<sizeX-1)
						{	
							if (m_tiles[pcurrent.y][pcurrent.x+1][pcurrent.z] == cellID)
							{
								pnext = pcurrent;
								pnext.x+=1;
								if (discovered.find(pnext) == discovered.end())
								{
									explore.push_back(pnext);
									if (pathCoord[pnext].landMarks.size()+1 < pathCoord[pcurrent].landMarks.size())
									{
										s1.x = pcurrent.x;
										s1.y = pcurrent.y;
										s1.z = pcurrent.x;
										lm = pathCoord[pcurrent];
										lm.landMarks.push_back(s1);
										pathCoord[pnext] = lm;
									}
								}
								else // if it hasn't been discovered yet, then give it a path
								{
									s1.x = pcurrent.x;
									s1.y = pcurrent.y;
									s1.z = pcurrent.x;
									lm = pathCoord[pcurrent];
									lm.landMarks.push_back(s1);
									pathCoord[pnext] = lm;
								}
							}
						}
						if (pcurrent.z>0)
						{
							if (m_tiles[pcurrent.y][pcurrent.x][pcurrent.z-1] == cellID)
							{
								pnext = pcurrent;	
								pnext.z-=1;
								if (discovered.find(pnext) == discovered.end())
								{
									explore.push_back(pnext);
									if (pathCoord[pnext].landMarks.size()+1 < pathCoord[pcurrent].landMarks.size())
									{
										s1.x = pcurrent.x;
										s1.y = pcurrent.y;
										s1.z = pcurrent.x;
										lm = pathCoord[pcurrent];
										lm.landMarks.push_back(s1);
										pathCoord[pnext] = lm;
									}
								}
								else // if it hasn't been discovered yet, then give it a path
								{
									s1.x = pcurrent.x;
									s1.y = pcurrent.y;
									s1.z = pcurrent.x;
									lm = pathCoord[pcurrent];
									lm.landMarks.push_back(s1);
									pathCoord[pnext] = lm;
								}
							}
						}
						if (pcurrent.z<sizeZ-1)
						{	
							if (m_tiles[pcurrent.y][pcurrent.x][pcurrent.z+1] == cellID)
							{
								pnext = pcurrent;
								pnext.z+=1;
								if (discovered.find(pnext) == discovered.end())
								{
									explore.push_back(pnext);
									if (pathCoord[pnext].landMarks.size()+1 < pathCoord[pcurrent].landMarks.size())
									{
										s1.x = pcurrent.x;
										s1.y = pcurrent.y;
										s1.z = pcurrent.x;
										lm = pathCoord[pcurrent];
										lm.landMarks.push_back(s1);
										pathCoord[pnext] = lm;
									}
								}
								else // if it hasn't been discovered yet, then give it a path
								{
									s1.x = pcurrent.x;
									s1.y = pcurrent.y;
									s1.z = pcurrent.x;
									lm = pathCoord[pcurrent];
									lm.landMarks.push_back(s1);
									pathCoord[pnext] = lm;
								}
							}
						}
					}
				}					
			}
		}
	}
}
*/

void Cbuilding::resetPeopleCrossed()
{
	map<int, PORTAL_INFO>::iterator iter_portal;
	
	for (iter_portal = m_tablePortals.begin(); iter_portal != m_tablePortals.end(); iter_portal++)
	{
		(*iter_portal).second.ncrossed = 0;
	}
}

void Cbuilding::loadObjs()
{
	//Declares some variables for later;
	int nfloors, x, z;
	char c;
	
	//cout << "Filename is: " << filename; 
	fstream filestr(filename, fstream::in | fstream::out);
	
	//Building size variables;
	filestr >> x;
	filestr >> z;
	filestr >> nfloors;

	char*** building;
	building = new char** [nfloors];
	for (int i=0; i<nfloors; i++)
	{
		building[i] = new char* [x];
		for (int j=0; j<x; j++)
		{
			building[i][j] = new char [z];
			for (int k=0; k<z; k++)
			{
				building[i][j][k] = ' ';  
			}

		}
	}

	for (int i=nfloors-1; i>=0; i--)
	{
		for (int j=0; j<z; j++)
		{
			for (int k=0; k<x; k++)
			{
				c=filestr.get();
				if (c=='\n') c=filestr.get();
				building[i][k][j]=c;
			}
		}
	}
	
	filestr.get();
	
	char objs[8] = "Objects";
	char matchedstring[8] ="";
	char cur;
	bool obstacle;
	double roomID;
	double pos[3];
	double rot[3];
	double scale;
	double radio;
	bool found = false;
	string name("");

	while(!found && !filestr.eof())
	{
		cur=filestr.get();
		if (cur=='\n') cur=filestr.get();
		if (cur!=objs[0]) filestr.ignore(10000, '\n');
		else
		{
			filestr.unget();
			filestr.getline(matchedstring, 8, '\n');
			if (strcmp(objs,matchedstring)==0) found=true;
		}
	}

	bool complete = false;
	if (filestr.eof())
		complete = true;

	while (!complete)
	{ 
		{
			filestr >> name;
			if (name.compare("Exit")==0) complete=true;
			if (!complete)
			{
				if (name.compare("0.4")==0) cin.get();
				filestr >> obstacle >> roomID >> pos[0] >> pos[1] >> pos[2] >> rot[0] >> rot[1] >> rot[2] >> scale >> radio;
				
				//cout << "Found: " << obstacle << " " << name << "Room: " << roomID << " Position: " << pos[0] << "," << pos[1] << "," <<pos[2] << " Rotation: " << rot[0] << "," << rot[1] << "," << rot[2] << " Scale: "<< scale << "+" << radio << "\n";
				addFurnitureToRooms(obstacle, name, roomID, pos, rot, scale, radio);
				filestr.get();
			}
		}
	}
	filestr.close();
	cout << "Loading of all models complete." << endl;
}