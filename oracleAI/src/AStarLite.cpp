//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include "astar/AStarLite.h"
#include <iostream>
#include <limits>
#include <typeinfo>
#include "oracleAI/OracleEnvironment.h"

using namespace std;

AStarLite::AStarLite()
{
	m_path.clear();
}

AStarLite::~AStarLite()
{
}

bool AStarLite::findPath(const Environment& env, int start, int target, bool ida, OracleAgent* subject) {
	if(ida) {
		return findPathIDA(env, start, target, subject);
	}
	else {
		return findPathA(env, start, target);
	}
}

bool AStarLite::findPathA(const Environment& env, int start, int target)
{
	// input sanitize
	assert(env.isValidNodeId(start));
	assert(env.isValidNodeId(target));

	// open and close lists
	AStarLiteOpen openList;
	AStarLiteClose closedList;

	// initialize search
	openList.clear();
	closedList.clear();
	float heuristic = env.getHeuristic(start, target);
	vector<Environment::Successor> successors;

	// create start node
	AStarLiteNode startNode(start, NO_NODE, 0, heuristic);

	// start search
	openList.insert(startNode);

	AStarLiteNode bestHeuristicNode = startNode;

	while(! openList.isEmpty())
	{
		// get the best node from the open list
		// AStarLiteOpen automatically puts that node on top
		AStarLiteNode node = openList.pop();

		// if we found the target, wrap up search and leave
		if (node.m_id == target)
		{
			closedList.insert(node);
			m_path = closedList.constructPath(start, target);
			break;
		}

		// else, get and search down the node's successors
		env.getSuccessors(node.m_id, NO_NODE, successors);

		for (vector<Environment::Successor>::const_iterator i = successors.begin(); i != successors.end(); ++i)
		{
			// successorg: the cost to go from this node to the sucessor i
			// successorId: the successor's id
			float successorg = node.m_g + i->m_cost;
			int successorId = i->m_target;

			// check if successor is in the open or closed lists
			// if it is, and the new successor g is better than the old one, remove it from the open or closed list
			if (openList.hasNode(successorId))
			{
				if (openList.search(successorId)->m_g > successorg)
					openList.remove(successorId);
				else
					continue;
			}
			else if (closedList.hasNode(successorId))
			{
				if (closedList.search(successorId)->m_g > successorg)
					closedList.remove(successorId);
				else
					continue;
			}


			//std::cerr << " - COST to reach node " << successorId << " is " << successorg << "\n";

			// the successor is either new or has a better parent path, so add it to the open list
			float successorHeuristic = env.getHeuristic(successorId, target);

			//std::cerr << " - F + G (for " << successorId << ") is " << successorg + successorHeuristic << "\n\n";

			AStarLiteNode successorNode(successorId, node.m_id, successorg, successorHeuristic);

			openList.insert(successorNode);
		}

		// done expanding node; add to closed list
		closedList.insert(node);

		// if open list is empty at this point, we failed to find a proper solution, so construct whatever is possible.
		if (node.m_f < bestHeuristicNode.m_f) {
			bestHeuristicNode = node;
		}
		if (openList.isEmpty()) {
			m_path = closedList.constructPath(start, bestHeuristicNode.m_id);
			break;
		}
	}
	
	
	/*#ifdef _DEBUG

	cout << '\n';
	for(vector<int>::const_iterator citer = m_path.cbegin(); citer != m_path.cend(); ++citer) {
		cout << *citer << " ";
	}
	cout << '\n';
	#endif*/

	return true;
}

bool AStarLite::findPathIDA(const Environment& env, int start, int target, OracleAgent* subject)
{
	// input sanitize
	assert(env.isValidNodeId(start));
	assert(env.isValidNodeId(target));
	
	//get the heuristic of the start, this is the initial cost limit
	//set up initial values for entry into the recursive function
	float initialCostLimit = env.getHeuristic(start, target);
	
	AStarLiteNode startNode(start, NO_NODE, 0, initialCostLimit);
	list<AStarLiteNode> initialPath;
	initialPath.push_back(startNode);
	currentTarget = target;

	pair<list<AStarLiteNode>, float> searchResults;
	unsigned int counter = 0;
	bool isLongPlan = false;
	try {
		dynamic_cast<const OracleEnvironment&>(env)._totalNodesExpanded = 0;
		#ifdef _DEBUG
		//cout << "-------------------------------------------------------" << endl;
		#endif
	}
	catch(bad_cast& b) {
		isLongPlan = true;
	}
	
	do {
		if(!isLongPlan) {
			static_cast<const OracleEnvironment&>(env)._numNodesExpanded = 0;
			#ifdef _DEBUG
			//cout << "Initial Cost Limit " << counter++ << ": " << initialCostLimit << endl;
			#endif
		}
		
		if(initialCostLimit == numeric_limits<float>::infinity()) {
			return false;
		}

		//cout << '\t' << initialCostLimit <<endl;
		//subject->printStepCacheSize();
		subject->flushStepCache();
		searchResults = idaStar(env, 0.0f, initialPath, initialCostLimit);

		initialCostLimit = searchResults.second;
	
		if(!isLongPlan) {
			static_cast<const OracleEnvironment&>(env)._totalNodesExpanded += static_cast<const OracleEnvironment&>(env)._numNodesExpanded;
			#ifdef _DEBUG
			//cout << "Nodes Expanded iteration " << counter << ": " << static_cast<const OracleEnvironment&>(env)._numNodesExpanded << endl;
			#endif
		}
	
	} while(searchResults.first.empty());
	
	if(!isLongPlan) {
		static_cast<const OracleEnvironment&>(env);
		#ifdef _DEBUG
		//cout << "Nodes expanded over entire search: " << static_cast<const OracleEnvironment&>(env)._totalNodesExpanded << endl;
		//cout << "Final Cost Limit: " << initialCostLimit << endl;
		//cout << "-------------------------------------------------------" << endl;
		#endif
	}
	
	//store the path in m_path

	m_path.clear();

	while(searchResults.first.back().m_parent != NO_NODE) {
		m_path.push_back(searchResults.first.back().m_id);
		searchResults.first.pop_back();
	}
	
	m_path.push_back(start);

	#ifdef _DEBUG
	//cout << '\n';
	//for(vector<int>::const_iterator citer = m_path.cbegin(); citer != m_path.cend(); ++citer) {
	//	cout << *citer << " ";
	//}
	//cout << '\n';
	#endif

	return true;
}

pair<list<AStarLiteNode>, float> AStarLite::idaStar(const Environment& env, float startCost, list<AStarLiteNode>& pathSoFar, float costLimit) {
	pair<list<AStarLiteNode>, float> result;
	result.first = pathSoFar;
	
	AStarLiteNode currentNode = pathSoFar.back();
	float minimumCost = startCost + env.getHeuristic(currentNode.m_id, currentTarget);

	//check base recursion cases
	if(minimumCost > costLimit) {
		result.first.clear();
		result.second = minimumCost;
		return result;
	}
	else if(currentNode.m_id == currentTarget) {
		result.second = costLimit;
		return result;
	}

	//recurse over successors
	float nextCostLimit = numeric_limits<float>::infinity();
	vector<Environment::Successor> successors;
	env.getSuccessors(currentNode.m_id, NO_NODE, successors);

	for(vector<Environment::Successor>::iterator iter = successors.begin(); iter != successors.end(); ++iter) {
		float newStartCost = startCost + iter->m_cost;
		AStarLiteNode child(iter->m_target, currentNode.m_id, currentNode.m_g + iter->m_cost, env.getHeuristic(iter->m_target, currentTarget)); 
		//list<AStarLiteNode> passedList = pathSoFar;
		//passedList.push_back(child);
		pathSoFar.push_back(child);

		//pair<list<AStarLiteNode>, float> recurse = idaStar(env, newStartCost, passedList, costLimit);
		pair<list<AStarLiteNode>, float> recurse = idaStar(env, newStartCost, pathSoFar, costLimit);

		if(!recurse.first.empty()) {
			return recurse;
		}
		nextCostLimit = min(nextCostLimit, recurse.second);
		pathSoFar.pop_back();
	}

	//base case where cost needs increased
	result.first.clear();
	result.second = nextCostLimit;
	return result;
}


const vector<int>& AStarLite::getPath() const
{
	return m_path;
}
