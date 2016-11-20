//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

//****************************************************************************//
// building.h                                                        //
// Copyright (C) 2007 Nuria Pelechano                                        //
//****************************************************************************//
// This code is covered by a disclosure agreement filed by the 
// University of Pennsylvania.  Do not distribute with out permission of
// the author.  
//****************************************************************************// 

#pragma warning ( disable : 4018)

#include <list>
#include <vector>
#include <map>
using namespace std;
#include <math.h>

#include "cell.h"
#include "building3D.h"

class CPG; 

#ifndef _BUILDING_H_
	#define _BUILDING_H_


typedef struct tVector
{
	double x,y,z;

} tVector;


struct PORTAL_INFO
{
	int cellAid, cellBid;
	bool	open;		// To know whether a portal is opened or closed
	list<Chumanoid*> m_lhumCrossing;
	// Data for calibration purposes
	int num_humsCrossed;
	int num_humsCrossedprev;
	double	posCenter[3]; 	// position of the center of the portal
	int		ncrossed;	// number of people that crossed this portal since last time. It's set to 0 everytime that the flow rate is calculated
	double	accDensity; // accumulated density. We will sum the average densities every time step for the last SECS seconds.
};

class Cbuilding
{
public:
	Cbuilding();
	~Cbuilding();
	void createCPG(CPG *cpg);
	void draw2D();
	void draw3D();
	int getNumRooms();
//	CCell getRoom(int id);
	int getNumExits();
	CCell* getCell(int cellID);
	void addHazardInCell(int type, int IDroom);
	void deleteInfoCell();	// Delete previous decisions taken by leaders. I think it doesn't make sense with the new animation step
	void reset();
	tVector getRandomPosInCell(int cellID);
	int getNumPortals();
	PORTAL_INFO getPortalInfo(int portalID);
	void addHumCrossed(int portalID);
	int getHumCrossed(int portalID);
	void addDensity(int portalID, double d);
	double getAccDensity(int portalID);
	void resetAccDensity(int portalID);
	int  findPortal(int cellA, int cellB);
	inline void closePortal(int portalID){if ((portalID > 0)&&(portalID<m_tablePortals.size()))m_tablePortals[portalID].open = false;};
	inline void openPortal(int portalID){if ((portalID > 0)&&(portalID<m_tablePortals.size()))m_tablePortals[portalID].open = true;};
	inline void insertHum(int portalID, Chumanoid* phum) {m_tablePortals[portalID].m_lhumCrossing.push_back(phum);};
	inline void removeHum(int portalID, Chumanoid* phum)  {m_tablePortals[portalID].m_lhumCrossing.remove(phum);};
	void printListHums(int portalID);
	void resetNumPeopleCrossingPortals();
	void addFurnitureToRooms(bool obstacle, const std::string &filename, int roomID, double* pos, double*rot, double scale, double radio);
	void addFurniture(bool obstacles);
	void loadObjs();
	
	inline map<int,PORTAL_INFO> getTablePortals(){return m_tablePortals;};
	inline void increasePeopleCrossed(int portalID){m_tablePortals[portalID].ncrossed++;};
	void resetPeopleCrossed();
	map<int,CCell*> m_building; // hash table containing all the rooms of the building
//
private:
	void createEnvironment(CPG *cpg);
	void createTiles(CPG *cpg);
	void createPaths();
	void computeLandMarks();
	
	//void calculateLength(int idCurrentCell, int numexit, int length); NOT ANYMORE
//	bool propagateExtraPaths(CCell* currCell, int nexit);
	
	char *filename;
	int nexitsBuilding;		// total number of exits in the building
	int exitsMaze[100];		// array of IDs of the exit cells not sure i need it...
	
	int	***m_tiles;		// Low level tiles. each tile has either a 0 if it's a wall, a -1 if it's a door, or a positive
						// integer which corresponds to the room name. So far i only need it to initialize humans inside
	                    // rooms, but it may be important in the future for collision detection or mesuremets within rooms
						// m_tiles[floor][x][z]
	int nfloors, sizeX, sizeZ; // info for m_tiles
	map<int,PORTAL_INFO> m_tablePortals;		// For each portal i'll have access to the IDs of the two cells being connected

	Cbuilding3D m_building3D;
};

inline bool operator!= (tVector A, tVector B)
{
	return ((A.x != B.x) || (A.y != B.y) || (A.z != B.z));
};

inline bool operator== (tVector A, tVector B)
{
	return ((A.x == B.x) && (A.y == B.y) && (A.z == B.z));
};

inline tVector operator+ (tVector A, tVector B)
{
	tVector step;
	step.x = A.x + B.x;
	step.y = A.y + B.y;
	step.z = A.z + B.z;
	return step;
};

inline tVector operator- (tVector A, tVector B)
{
	tVector step;
	step.x = A.x - B.x;
	step.y = A.y - B.y;
	step.z = A.z - B.z;
	return step;
};

inline tVector operator/ (tVector A, double f)
{
	tVector step;
	step.x = A.x / f;
	step.y = A.y / f;
	step.z = A.z / f;
	return step;
};

inline tVector operator* (tVector A, double f)
{
	tVector step;
	step.x = A.x * f;
	step.y = A.y * f;
	step.z = A.z * f;
	return step;
};

inline bool operator< (sCOORD A, sCOORD B)
{
	if (A.x == B.x)
	{
		if (A.y == B.y)
		{
			return (A.z < B.z);
		}
		else return (A.y < B.y);	
	}
	else return (A.x < B.x);
	
};


inline void copy(double* org, double* dst)
{
	dst[0] = org[0];
	dst[1] = org[1];
	dst[2] = org[2];
}

/*
tVector operator/= (double f)
{
	tVector step;
	step.x = A.x / f;
	step.y = A.y / f;
	step.z = A.z / f;
	return step;
};
*/

inline tVector	operator *(int val, tVector step1) 
{	
	tVector step2;

	step2.x = val*step1.x;
	step2.y = val*step1.y;
	step2.z = val*step1.z;

	return step2;
}

inline tVector	operator -(tVector step1) 
{	
	tVector step_neg;

	step_neg.x = - step1.x;
	step_neg.y = - step1.y;
	step_neg.z = - step1.z;

	return step_neg;
}

inline void float2tVector(float* val, tVector* step)
{
	step->x=val[0];
	step->y=val[1];
	step->z=val[2]; 
};

inline void double2tVector(double* val, tVector* step)
{
	step->x=val[0];
	step->y=val[1];
	step->z=val[2]; 
};

inline void tVector2float(tVector step, float* val)
{
	val[0] = step.x;
	val[1] = step.y;
	val[2] = step.z;

}


inline void tVector2double(tVector step, double* val)
{
	val[0] = step.x;
	val[1] = step.y;
	val[2] = step.z;

}

static tVector cross (tVector srcA, tVector srcB)
{
	tVector dst;
	dst.x = srcA.y*srcB.z - srcA.z*srcB.y;
	dst.y = srcA.z*srcB.x - srcA.x*srcB.z;
	dst.z = srcA.x*srcB.y - srcA.y*srcB.x;
	return dst;
}

static double length(tVector scr)
{
	return sqrt(scr.x*scr.x + scr.y*scr.y + scr.z*scr.z);
}

static double dot (tVector srcA, tVector srcB)
{
	return (srcA.x*srcB.x + srcA.y*srcB.y + srcA.z*srcB.z);
}

// Normalize the input vector
static double normalize (tVector *vec)
{
	double aux = vec->x*vec->x + vec->y*vec->y + vec->z*vec->z;
	double Len = sqrt(aux);
	//double Len = sqrt(vec->x*vec->x + vec->y*vec->y + vec->z*vec->z);
	(Len < 0.001) ? Len = 0.001 : Len;
	
	double invLen = 1.f / (double)Len;

	vec->x *= invLen;
	vec->y *= invLen;
	vec->z *= invLen;

	return Len;
}

// Normalize the input vector
static double normalize (float vec[3])
{
	double aux = vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2];
	double Len = sqrt(aux);
	//double Len = sqrt(vec->x*vec->x + vec->y*vec->y + vec->z*vec->z);
	(Len < 0.001) ? Len = 0.001 : Len;
	
  if (Len == 0)
    std::cout << "ERROR dividing by zero in normalize" << std::endl;

	double invLen = 1.f / (double)Len;

	vec[0] *= invLen;
	vec[1] *= invLen;
	vec[2] *= invLen;

	return Len;
}



#endif 

