//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

//
// THIS CODE WAS ADAPTED FROM libpath, from University of Alberta.
//

#ifndef PATHFIND_ENVIRONMENT_H
#define PATHFIND_ENVIRONMENT_H

#include <string>
#include <vector>

//-----------------------------------------------------------------------------

using namespace std;

/** Interface to search environment. */
class Environment
{
public:
	/** Information about a successor of a node in the environment. */
	class Successor
	{
	public:
		Successor();

		Successor(int target, float cost) : m_target(target), m_cost(cost) {}

		int m_target;
		float m_cost;

		bool operator== (const Successor &other) const
		{
			return (m_target == other.m_target);
		}

		bool operator< (const Successor &other) const
		{
			if(m_target < other.m_target)
				return true;
			else if(m_target == other.m_target)
				return (m_cost < other.m_cost);
			else
				return false;
		}
	};

	virtual ~Environment() { };

	virtual float getHeuristic(int start, int target) const = 0;

	virtual float getMaxCost() const = 0;

	virtual float getMinCost() const = 0;

	virtual int getNumberNodes() const = 0;

	/** Generate successor nodes for the search.
	@param nodeId Current node
	@param lastNodeId
	Can be used to prune nodes,
	(is set to NO_NODE in Search::checkPathExists).
	@param result Resulting successors.
	Passed in as a reference to avoid dynamic memory allocation.
	The vector will be cleared and filled with the successors.
	*/
	virtual void getSuccessors(int nodeId, int lastNodeId,
		vector<Successor>& result) const = 0;

	virtual bool isValidNodeId(int nodeId) const = 0;
};        

#endif
