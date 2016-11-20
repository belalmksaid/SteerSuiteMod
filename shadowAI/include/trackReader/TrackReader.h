//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#pragma once
#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>
#include <sstream>
#include "SteerLib.h"	

#include <vector>

using namespace std;
using namespace Util;

class TrackReader
{
private:
	void read(string);
	void createIterators();
	std::vector<Point>::iterator* iterator_array;
	std::vector<vector<Point>> all_trajectories;
	int num_persons;

public:
	TrackReader(string);
	~TrackReader(void);
	int num_trajectory(void) const;
	float entry_time(int) const;
	float exit_time(int) const;
	Point entry_location(int) const;
	Point exit_location(int) const;
	Point current_location(int, float);
	Vector velocity(int);
};

