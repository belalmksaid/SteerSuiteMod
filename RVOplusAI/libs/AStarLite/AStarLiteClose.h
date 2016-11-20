//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#pragma once

#include "AStarLiteNode.h"

#include <map>
#include <vector>
#include <cassert>

class AStarLiteClose
{
public:
	AStarLiteClose();
	~AStarLiteClose();

	void clear(void);
	bool insert(const AStarLiteNode& node);
	bool isEmpty() const;
	int size() const;
	bool hasNode(int nodeId);
	bool remove(int nodeId);
	const AStarLiteNode* search(int nodeId);
	std::vector<int> constructPath(int start, int target);

private:
	typedef std::map<int,AStarLiteNode> NodeMap;
	typedef NodeMap::iterator NodeMapIter;

	NodeMap m_nodeMap;
};
