//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#pragma once

#include <vector>
#include <list>

#include "astar/Environment.h"
#include "astar/AStarLiteNode.h"
#include "astar/AStarLiteOpen.h"
#include "astar/AStarLiteClose.h"
#include "oracleAI/OracleAgent.h"

#include <cassert>

class OracleAgent;	//forward declaration since both classes include each other

class AStarLite
{
public:

	AStarLite();
	~AStarLite();

	bool findPath(const Environment& env, int start, int target, bool ida, OracleAgent*);
	bool findPathA(const Environment& env, int start, int target);
	bool findPathIDA(const Environment& env, int start, int target, OracleAgent*);
	std::pair<std::list<AStarLiteNode>, float> idaStar(const Environment& env, float startCost, std::list<AStarLiteNode>& pathSoFar, float costLimit);

    const vector<int>& getPath() const;

private:

    static const int NO_NODE = -1;
	int currentTarget;

	vector<int> m_path;

};
