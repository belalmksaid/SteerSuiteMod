//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#pragma once

#include <vector>

#include "Environment.h"
#include "AStarLiteNode.h"
#include "AStarLiteOpen.h"
#include "AStarLiteClose.h"

#include <cassert>


class AStarLite
{
public:

	AStarLite(void);
	~AStarLite(void);

	bool findPath(const Environment& env, int start, int target);

    const vector<int>& getPath() const;

private:

    static const int NO_NODE = -1;

	vector<int> m_path;

};
