//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#ifndef __goalInfo_h__
#define __goalInfo_h__

#include "../../steerlib/include/SteerLib.h"

class goalInfo
{
public:

	bool m_goal;
	int m_id;
	float m_desiredSpeed;
	Util::Point m_loc;
	
	goalInfo() : m_goal(false), m_id(-1), m_desiredSpeed(0), m_loc(0,0,0)
	{}

	~goalInfo()
	{}
};

#endif
