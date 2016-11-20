//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#pragma once

#include "AStarLiteNode.h"

#include <set>
#include <map>

class AStarLiteOpen
{
public:
	AStarLiteOpen(void);
	~AStarLiteOpen(void);

	void clear(void);
	bool insert(const AStarLiteNode& node);
	bool isEmpty() const;
	int size() const;
	AStarLiteNode pop();
	bool hasNode(int nodeId);
	bool remove(int nodeId);
	const AStarLiteNode* search(int nodeId);

private:
	typedef std::multiset<AStarLiteNode, AStarLiteNode::Compare> NodeSet;
	typedef NodeSet::iterator NodeSetIter;

	typedef std::map<int,NodeSetIter> NodeMap;
	typedef NodeMap::iterator NodeMapIter;

	NodeSet m_openSet;
	NodeMap m_nodeMap;
};
