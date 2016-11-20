// Author: MahyarKoy @Jan2016

#include "VisibilityGraph.h"
#include "SteerLib.h"
#include <vector>
#include <iostream>
#include <math.h>
#include "util/GenericException.h"
#include "GraphSearch.h"
#ifdef CUDA_ENABLED
#include <cuda_profiler_api.h>
#include <cuda.h>
#endif

using namespace std;
using namespace SpaceSyntax;
using namespace Eigen;

VisibilityGraph::VisibilityGraph(SteerLib::SpatialDataBaseInterface* spacialDB, double x1, double z1, double x2, double z2)
	: _gridPlan(x1, z1, x2, z2, 0, 10, 10), _isInit(false)
{
	_spacialDB = spacialDB;
	add_region(x1, z1, x2, z2);

}

VisibilityGraph::VisibilityGraph(float x1, float z1, float x2, float z2, float height, unsigned int gridNumX, unsigned int gridNumZ)
	: _gridPlan(x1, z1, x2, z2, height, gridNumX, gridNumZ), _isInit(false)
{

}

VisibilityGraph::~VisibilityGraph()
{

}

void VisibilityGraph::init_graph()
{
	if (_isInit)
		return;
	else
		_isInit = true;

	unsigned matSize = _nodeList.size();
	_adjMatrix = MatrixXf::Zero(matSize, matSize);
	for (unsigned i = 0; i < matSize; i++)
	{
		_nodeList[i].adjList.assign(matSize, Edge(0));
		_nodeList[i].listSize = 0;
	}
}

void VisibilityGraph::clear_nodes()
{
	_nodeList.clear();
	_queryRegion.clear();
	_refRegion.clear();
	_isInit = false;
#ifdef CUDA_ENABLED
	_gpuvg.clear();
#endif
}

void VisibilityGraph::add_node(unsigned int cellIndex)
{
	Util::Point cellPoint;
	_spacialDB->getLocationFromIndex(cellIndex, cellPoint);
	// TODO: check for duplicate nodes before push_back
	_nodeList.push_back(Node(cellPoint));
}

void VisibilityGraph::add_node(Util::Point location)
{
	unsigned int cellID;
	cellID = _spacialDB->getCellIndexFromLocation(location.x, location.z);
	add_node(cellID);
}

void VisibilityGraph::add_node_gp(Util::Point location)
{
	Util::Point cellPoint;
	unsigned int x, z;
	_gridPlan.get_gridLocation(location, x, z);
	cellPoint = _gridPlan.get_location(x, z);
	// TODO: check for duplicate nodes before push_back
	_nodeList.push_back(Node(cellPoint));
}

void VisibilityGraph::add_node_query(unsigned int gx, unsigned int gz)
{
	Util::Point cellPoint;
	unsigned int gridID;
	gridID = _gridPlan.get_gridID(gx, gz);
	auto itr = _refRegion.find(gridID);
	if (itr != _refRegion.end())
	{
		// if already in refRegion, then just set query flag
		_nodeList[itr->second].isQuery = true;
		_queryRegion.insert(std::pair<unsigned, unsigned>(gridID, itr->second));
	}
	else
	{
		// if node was already in query region, then skip
		auto ret = _queryRegion.insert(std::pair<unsigned, unsigned>(gridID, _nodeList.size()));
		if (ret.second == true)
		{
			// if it was non-existant in either regions, then add to nodeList and set query flag
			cellPoint = _gridPlan.get_location(gx, gz);
			_nodeList.push_back(Node(cellPoint));
			_nodeList.back().isQuery = true;
		}
	}
	return;
}

void VisibilityGraph::add_node_ref(unsigned int gx, unsigned int gz)
{
	Util::Point cellPoint;
	unsigned int gridID;
	gridID = _gridPlan.get_gridID(gx, gz);
	auto itr = _queryRegion.find(gridID);
	if (itr != _queryRegion.end())
	{
		// if already in queryRegion, then just set query flag
		_nodeList[itr->second].isRef = true;
		_refRegion.insert(std::pair<unsigned, unsigned>(gridID, itr->second));
	}
	else
	{
		// if node was already in ref region, then skip
		auto ret = _refRegion.insert(std::pair<unsigned, unsigned>(gridID, _nodeList.size()));
		if (ret.second == true)
		{
			// if it was non-existant in either regions, then add to nodeList and set query flag
			cellPoint = _gridPlan.get_location(gx, gz);
			_nodeList.push_back(Node(cellPoint));
			_nodeList.back().isRef = true;
		}
	}
	return;
}

void VisibilityGraph::add_node_exact(Util::Point location)
{
	_nodeList.push_back(Node(location));
}

void VisibilityGraph::add_region(double x1, double z1, double x2, double z2)
{
	unsigned int startID = _spacialDB->getCellIndexFromLocation(x1, z1);
	unsigned int endID = _spacialDB->getCellIndexFromLocation(x2, z2);
	if (startID > endID)
	{
		unsigned int tmp = startID;
		startID = endID;
		endID = tmp;
	}
	unsigned int minX, minZ, maxX, maxZ;
	_spacialDB->getGridCoordinatesFromIndex(startID, minX, minZ);
	_spacialDB->getGridCoordinatesFromIndex(endID, maxX, maxZ);

	unsigned int cellID;
	Util::Point cellPoint;
	for (unsigned i = minX; i < maxX + 1; i++)
		for (unsigned j = minZ; j < maxZ + 1; j++)
		{
			cellID = _spacialDB->getCellIndexFromGridCoords(i, j);
			add_node(cellID);
		}
}

void VisibilityGraph::add_region_gp(float x1, float z1, float x2, float z2, RegionType regionType, unsigned regionID)
{
	_isInit = false;
#ifdef CUDA_ENABLED
	_gpuvg.clear();
#endif
	Util::Point startP(x1, 0, z1) , endP(x2, 0, z2);
	unsigned int xMin, zMin, xMax, zMax;
	_gridPlan.get_gridLocation(startP, xMin, zMin);
	_gridPlan.get_gridLocation(endP, xMax, zMax);
	if (xMin > xMax)
	{
		float tmp = xMin;
		xMin = xMax;
		xMax = tmp;
	}
	if (zMin > zMax)
	{
		float tmp = zMin;
		zMin = zMax;
		zMax = tmp;
	}

	Util::Point cellPoint;
	for (unsigned i = xMin; i < xMax+1; i++)
		for (unsigned j = zMin; j < zMax+1; j++)
		{
			if (regionType == RegionType::QUERY)
				add_node_query(i, j);
			else
				add_node_ref(i, j);
		}
}

void VisibilityGraph::generate_graph()
{
	unsigned int matSize = _nodeList.size();
	_adjMatrix = MatrixXf::Ones(matSize, matSize);
	for (unsigned i = 0; i < matSize; i++)
		for (unsigned j = i + 1; j < matSize; j++)
		{
			if (_spacialDB->hasLineOfSight(_nodeList[i].location, _nodeList[j].location, NULL, NULL))
			{
				_adjMatrix(i, j) = 1;
				_adjMatrix(j, i) = 1;
			}
			else
			{
				_adjMatrix(i, j) = 0;
				_adjMatrix(j, i) = 0;
			}
		}
}

void VisibilityGraph::generate_graph_gp()
{
	generate_graph_gp_omp();

	/************ Old seuqential version *************/
	/*
	unsigned int matSize = _nodeList.size();
	// Initialize the adj containers to empty state
	_adjMatrix = MatrixXf::Ones(matSize, matSize);
	for (unsigned i = 0; i < matSize; i++)
		if (!_nodeList[i].adjList.empty())
			_nodeList[i].adjList.clear();
	
	// Fill the adj containers according to raycast
	for (unsigned i = 0; i < matSize; i++)
		for (unsigned j = i + 1; j < matSize; j++)
		{
			if (_gridPlan.hasLineOfSight(_nodeList[i].location, _nodeList[j].location))
			{
				_adjMatrix(i, j) = 1;
				_adjMatrix(j, i) = 1;
				_nodeList[i].adjList.push_back(Edge(j));
				_nodeList[j].adjList.push_back(Edge(i));
			}
			else
			{
				_adjMatrix(i, j) = 0;
				_adjMatrix(j, i) = 0;
			}
			if (i == 34)
			{
				IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
				std::cout << _adjMatrix.row(i).format(CleanFmt) << std::endl;
				std::cout << "PROBLEM:" << _adjMatrix(34, 35) << std::endl << _nodeList[35].location << std::endl;
			}
		}*/
}

void VisibilityGraph::generate_graph_gp_omp()
{
	init_graph();
	unsigned int matSize = _nodeList.size();
	// Initialize the adj containers to empty state
	_adjMatrix.setZero();
	for (unsigned i = 0; i < matSize; i++)
		_nodeList[i].listSize = 0;

		// Fill the adj containers according to raycast
#pragma omp parallel
	{
		//std::cout << "NUM THREADS >>>> " << omp_get_num_threads() << std::endl;
#pragma omp for ordered schedule(dynamic)
		for (int i = 0; i < matSize; i++)
			for (int j = i; j < matSize; j++)
				//for (int j = 0; j < matSize; j++)
			{
				if (_gridPlan.hasLineOfSight(_nodeList[i].location, _nodeList[j].location))
				{
					_adjMatrix(i, j) = 1;
					_adjMatrix(j, i) = 1;
					//#pragma omp critical(dataupdateI)
					//_nodeList[i].adjList.push_back(Edge(j));
					//#pragma omp critical(dataupdateJ)
					//_nodeList[j].adjList.push_back(Edge(i));
				}
				else
				{
					_adjMatrix(i, j) = 0;
					_adjMatrix(j, i) = 0;
				}
			}
	}

	// Fill the adjacency list
	for (int i = 0; i < matSize; i++)
	{
		for (int j = i + 1; j < matSize; j++)
		{
			if (_adjMatrix(i, j) == 1)
			{
				unsigned ix, jx;
				ix = _nodeList[i].listSize;
				jx = _nodeList[j].listSize;
				_nodeList[i].adjList[ix] = Edge(j);
				_nodeList[j].adjList[jx] = Edge(i);
				// move indices forward on the adjLists
				_nodeList[i].listSize++;
				_nodeList[j].listSize++;
			}
		}
	}

}

void VisibilityGraph::generate_graph_gp_omp_v2()
{
	init_graph();
	unsigned int matSize = _nodeList.size();
	// Initialize the adj containers to empty state
	_adjMatrix.setOnes();
	for (unsigned i = 0; i < matSize; i++)
		_nodeList[i].listSize = 0;
	
	float ox1, oz1, ox2, oz2, tilt = -10, pretilt = -10;
	float ox1r, oz1r, ox2r, oz2r;
	std::vector<float> pointX(matSize, 0);
	std::vector<float> pointZ(matSize, 0);
	for (auto itr = _gridPlan._obstacleList.begin(); itr != _gridPlan._obstacleList.end(); itr++)
	{
		if (itr->get_Type() == ObstacleType::SHADOW)
			continue;
		ox1 = itr->get_pMin().x;
		oz1 = itr->get_pMin().z;
		ox2 = itr->get_pMax().x;
		oz2 = itr->get_pMax().z;
		tilt = abs(itr->get_tilt());
		if (tilt < M_PI_2 + 0.001 && tilt > M_PI_2 - 0.001)
			tilt = 0;
		else
			tilt = itr->get_tilt();

		// if similar tilt to previous
		if (tilt == pretilt)
		{
			// Do axis rotation to get axis aligned obstacle, no need to rotate the nodes
			_gridPlan.rotate_axis(ox1r, oz1r, tilt, ox1, oz1);
			_gridPlan.rotate_axis(ox2r, oz2r, tilt, ox2, oz2);
		}
		else
		{
			// Do axis rotation to get axis aligned obstacle
			pretilt = tilt;
			_gridPlan.rotate_axis(ox1r, oz1r, tilt, ox1, oz1);
			_gridPlan.rotate_axis(ox2r, oz2r, tilt, ox2, oz2);
			// Do axis rotation on all the nodes
#pragma omp parallel for
			for (int i = 0; i < matSize; i++)
				_gridPlan.rotate_axis(pointX[i], pointZ[i], tilt, _nodeList[i].location.x, _nodeList[i].location.z);
		}

		// Fill the adj containers according to raycast
#pragma omp parallel
		{
			//std::cout << "NUM THREADS >>>> " << omp_get_num_threads() << std::endl;
#pragma omp for ordered schedule(dynamic)
			for (int i = 0; i < matSize; i++)
				for (int j = i; j < matSize; j++)
				{
					if (_adjMatrix(i, j) == 0)
						continue;
					if (!_gridPlan.ray_cast_v2(pointX[i], pointZ[i], pointX[j], pointZ[j], ox1r, oz1r, ox2r, oz2r))
					{
						_adjMatrix(i, j) = 0;
						_adjMatrix(j, i) = 0;
					}
				}
		}
	}

	// Fill the adjacency list
	for (int i = 0; i < matSize; i++)
	{
		for (int j = i + 1; j < matSize; j++)
		{
			if (_adjMatrix(i, j) == 1)
			{
				unsigned ix, jx;
				ix = _nodeList[i].listSize;
				jx = _nodeList[j].listSize;
				_nodeList[i].adjList[ix] = Edge(j);
				_nodeList[j].adjList[jx] = Edge(i);
				// move indices forward on the adjLists
				_nodeList[i].listSize++;
				_nodeList[j].listSize++;
			}
		}
	}

}

void VisibilityGraph::print_graph()
{

}

MatrixXf VisibilityGraph::get_adjMatrix()
{
	return _adjMatrix;
}

vector<Node> VisibilityGraph::get_nodeList()
{
	return _nodeList;
}

vector<Node> VisibilityGraph::get_queryList()
{
	vector<Node> queryList;
	for (auto itr = _queryRegion.begin(); itr != _queryRegion.end(); itr++)
	{
		queryList.push_back(_nodeList[itr->second]);
	}
	return queryList;
}

vector<Node> VisibilityGraph::get_refList()
{
	vector<Node> refList;
	for (auto itr = _refRegion.begin(); itr != _refRegion.end(); itr++)
	{
		refList.push_back(_nodeList[itr->second]);
	}
	return refList;
}

float VisibilityGraph::get_nodeDegree(unsigned int index)
{
	float degree = 0;
	for (auto i = 0; i < _nodeList[index].listSize; i++)
	{
		if (_nodeList[_nodeList[index].adjList[i].nodeID].isRef)
			degree++;
	}
	return degree;
}

float VisibilityGraph::get_nodeDegree(Util::Point target)
{
	auto ret = get_nodeIndex(target);
	if (ret.second == true)
		return get_nodeDegree(ret.first);
	else
		return 0; // must throw GenericException
}

unsigned VisibilityGraph::get_gridID(Util::Point target)
{
	unsigned gx, gz;
	_gridPlan.get_gridLocation(target, gx, gz);
	return _gridPlan.get_gridID(gx, gz);
}

float VisibilityGraph::get_meanDegree()
{
	float sum = 0;
	for (auto itr = _queryRegion.begin(); itr != _queryRegion.end(); itr++)
	{
		sum += get_nodeDegree(itr->second);
	}
	return sum / _queryRegion.size();
}

void VisibilityGraph::setup_gridPlan(float x1, float z1, float x2, float z2, float height, unsigned int gridNumX, unsigned int gridNumZ)
{
	_gridPlan.set_gridNumX(gridNumX);
	_gridPlan.set_gridNumZ(gridNumZ);
	_gridPlan.set_height(height);
	_gridPlan.set_gridSize(x1, z1, x2, z2);
}

pair<unsigned,bool> VisibilityGraph::get_nodeIndex(Util::Point target)
{
	unsigned gridID = get_gridID(target);
	auto qitr = _queryRegion.find(gridID);
	if (qitr != _queryRegion.end())
		return pair<unsigned, bool> (qitr->second, true);
	auto ritr = _refRegion.find(gridID);
	if (ritr != _refRegion.end())
		return pair<unsigned, bool>(ritr->second, true);
	return pair<unsigned, bool>(0, false);
}

int VisibilityGraph::get_nodeIndex_exact(Util::Point location)
{
	for (auto i = 0; i < _nodeList.size(); i++)
	{
		if (_nodeList[i].location.x == location.x && _nodeList[i].location.z == location.z)
			return i;
	}
	return -1;
}

std::vector<Util::Point> VisibilityGraph::get_cellPoints(unsigned int index)
{
	return get_cellPoints(_nodeList[index].location);
}

std::vector<Util::Point> VisibilityGraph::get_cellPoints(Util::Point target)
{
	float gridSizeX = _gridPlan.get_gridSizeX(), gridSizeZ = _gridPlan.get_gridSizeZ();
	unsigned int gx, gz;
	Util::Vector southEast(-gridSizeX/2, 0, gridSizeZ/2), northEast(gridSizeX/2, 0, gridSizeZ/2);
	_gridPlan.get_gridLocation(target, gx, gz);
	Util::Point center = _gridPlan.get_location(gx, gz);
	std::vector<Util::Point> cellPoints;
	cellPoints.push_back(center - southEast);
	cellPoints.push_back(center - northEast);
	cellPoints.push_back(center + southEast);
	cellPoints.push_back(center + northEast);
	return cellPoints;
}

Tree::Tree(const std::vector<Node> &nodeList, const Eigen::MatrixXf &adjMatrix, unsigned int origin, bool full) :
	_parentList(nodeList.size(), NodeType::EMPTY), _origin(origin)
{
	_generate(nodeList, adjMatrix, full);
}

void Tree::_generate(const std::vector<Node> &nodeList, const Eigen::MatrixXf &adjMatrix, bool full)
{
	_parentList[_origin] = NodeType::ROOT;
	unsigned int childCountNow = 1, childCountNext = 0, totalCount = 1, skipCount = 0;
	vector<int> childListNext;

	// Breadth search
	queue<unsigned int> openList;
	unsigned int head;
	openList.push(_origin);
	while (openList.size() > 0)
	{
		head = openList.front();
		openList.pop();
		childCountNow--;

		// expand head adj matrix version
		/*
		for (auto j = 0; j < nodeList.size(); j++)
		{
		if (adjMatrix(head, j) == 0)
		continue;
		if (_parentList[j] != NodeType::EMPTY)
		continue;
		_parentList[j] = head;
		openList.push(j);
		}*/

		//expand head adj list version - newer
		for (unsigned j = 0; j < nodeList[head].listSize; j++)
		{
			unsigned int id;
			id = nodeList[head].adjList[j].nodeID;
			if (_parentList[id] != NodeType::EMPTY)
				continue;
			if (!full && !nodeList[id].isRef)
			{
				_parentList[id] = NodeType::SKIP;
				skipCount++;
				continue;
			}
			_parentList[id] = head;
			openList.push(id);
			childCountNext++;
			childListNext.push_back(id);
			totalCount++;
		}

		// Check if next level is reached
		if (childCountNow == 0 && childCountNext > 0)
		{
			_depthList.push_back(childListNext);
			childCountNow = childCountNext;
			childCountNext = 0;
			childListNext.clear();
		}

		// Prune if all nodes are visited
		if (totalCount >= _parentList.size() || (!full && totalCount+skipCount >= _parentList.size()) )
		{
			if (childCountNext > 0)
			{
				_depthList.push_back(childListNext);
				head = openList.front();
			}
			break;
		}
	}
	_tail = head;
	_size = totalCount;
}

Tree* VisibilityGraph::get_tree(unsigned int origin)
{
	return new Tree(_nodeList, _adjMatrix, origin); //MEMORY LEAK CAUTION
	/*std::vector<int> parentList;
	for (auto i = 0; i < _nodeList.size(); i++)
		parentList.push_back(NodeType::EMPTY);
	parentList[origin] = NodeType::ROOT;

	// Breadth search
	std::queue<unsigned int> openList;
	unsigned int head;
	openList.push(origin);
	while (openList.size() > 0)
	{
		head = openList.front();
		openList.pop();
		//expand head
		for (auto j = 0; j < _nodeList.size(); j++)
		{
			if (_adjMatrix(head, j) == 0)
				continue;
			if (parentList[j] != NodeType::EMPTY)
				continue;
			parentList[j] = head;
			openList.push(j);
		}
	}
	return parentList;*/
}

std::vector<int> VisibilityGraph::get_treeParentList(unsigned int origin)
{
	Tree graphTree(_nodeList, _adjMatrix, origin);
	return graphTree.get_parentList();
}

float VisibilityGraph::get_treeDepth(unsigned int origin)
{
	Tree graphTree(_nodeList, _adjMatrix, origin);
	return graphTree.get_depth();
}

float VisibilityGraph::get_treeEntropy(unsigned int origin)
{
	Tree graphTree(_nodeList, _adjMatrix, origin);
	return graphTree.get_entropy();
}

float VisibilityGraph::get_treeDepth(Util::Point target)
{
	auto ret = get_nodeIndex(target);
	if (ret.second == true)
		return get_treeDepth(ret.first);
	else
		throw GenericException("Tree generation failed: node not found in the graph!");
}

float VisibilityGraph::get_treeEntropy(Util::Point target)
{
	auto ret = get_nodeIndex(target);
	if (ret.second == true)
		return get_treeEntropy(ret.first);
	else
		throw GenericException("Tree generation failed: node not found in the graph!");
}

pair<float, float> VisibilityGraph::get_treeProp(unsigned int origin)
{
	Tree graphTree(_nodeList, _adjMatrix, origin);
	return pair<float, float>(graphTree.get_depth(), graphTree.get_entropy());
}

pair<float, float> VisibilityGraph::get_treeProp(Util::Point target)
{
	auto ret = get_nodeIndex(target);
	if (ret.second == true)
		return get_treeProp(ret.first);
	else
		throw GenericException("Tree generation failed: node not found in the graph!");
}

float VisibilityGraph::get_meanTreeDepth()
{
	float sum = 0;
	for (auto itr = _queryRegion.begin(); itr != _queryRegion.end(); itr++)
	{
		sum += get_treeDepth(itr->second);
	}
	return sum / _queryRegion.size();
}

float VisibilityGraph::get_meanTreeEntropy()
{
	float sum = 0;
	for (auto itr = _queryRegion.begin(); itr != _queryRegion.end(); itr++)
	{
		sum += get_treeEntropy(itr->second);
	}
	return sum / _queryRegion.size();
}

pair<float, float> VisibilityGraph::get_meanTreeProp()
{
	float dsum = 0, esum = 0;
	vector<float> depthList(_queryRegion.size(), 0.0f);
	vector<float> entropyList(_queryRegion.size(), 0.0f);
	vector<unsigned int> queryList;
	// populate queryList with node IDs of the query nodes
	for (auto itr = _queryRegion.begin(); itr != _queryRegion.end(); itr++)
	{
		queryList.push_back(itr->second);
	}
	//std::cout << "LIST SIZES" << queryList.size() << "_" << depthList.size() << "_" << entropyList.size() << endl;

#pragma omp parallel
	{
#pragma omp for ordered schedule(dynamic)
		for (int i = 0; i < queryList.size(); i++)
		{
			pair<float, float> treeProp;
			treeProp = get_treeProp(queryList[i]);
			depthList[i] = treeProp.first;
			entropyList[i] = treeProp.second;
		}
	}

	float maxVal = 0;
	for (int i = 0; i < queryList.size(); i++)
	{
		if (depthList[i] > maxVal)
			maxVal = depthList[i];
	}
	for (int i = 0; i < queryList.size(); i++)
	{
		if (depthList[i] == 0)
			dsum += maxVal;
		else
			dsum += depthList[i];
		esum += entropyList[i];
	}
	return pair<float, float>(dsum / queryList.size(), esum / queryList.size());
}

float Tree::get_entropy()
{
	float sum = 0, entropy = 0;
	for (unsigned i = 0; i < _depthList.size(); i++)
		sum += _depthList[i].size();
	for (unsigned i = 0; i < _depthList.size(); i++)
	{
		float prob;
		prob = _depthList[i].size() / sum;
		entropy += -prob*log2f(prob);
	}
	return entropy;
}

bool VisibilityGraph::check_connection()
{
	unsigned int root;
	// find a root placed freely, not covered by an obstacle
	for (root = 0; root < _nodeList.size(); root++)
	{
		if (_adjMatrix(root, root) == 1)
			break;
	}

	// if no free root, return false
	if (root > _nodeList.size())
		return false;

	// make a tree, and check if it covers all the nodes in the graph
	Tree graphTree(_nodeList, _adjMatrix, root, true);
	int disCount = _adjMatrix.trace() - graphTree.get_size();
	return disCount > 0 ? false : true;
}

#ifdef CUDA_ENABLED
void VisibilityGraph::generate_graph_cuda()
{
	init_graph();

	// Initialize the adj containers to empty state
	unsigned matSize = _nodeList.size();
	_adjMatrix.setZero();
	for (unsigned i = 0; i < matSize; i++)
		_nodeList[i].listSize = 0;

	bool gpuAvailable = selectBestGPU();
	if (!gpuAvailable)
		throw GenericException("No CUDA enabled device available!");

	// Set obstacles
	vector<Obstacle> obstacles = _gridPlan.get_obstacles();
	vector<Obstacle_Struct> obsList;
	for (int i = 0; i < obstacles.size(); i++)
	{
		Util::Point tmp;
		Point_Struct obsP1, obsP2;
		tmp = obstacles[i].get_pMin();
		obsP1.x = tmp.x; obsP1.y = tmp.y; obsP1.z = tmp.z;
		tmp = obstacles[i].get_pMax();
		obsP2.x = tmp.x; obsP2.y = tmp.y; obsP2.z = tmp.z;
		obsList.push_back(Obstacle_Struct(obsP1, obsP2, obstacles[i].get_Type(), obstacles[i].get_tilt()));
	}

	int totalNodes = _nodeList.size();

	if (!_gpuvg.is_init())
	{
		// Initialize grid with PointStructs. Location is given by row,column position in grid.
		std::vector<Point_Struct> gpuPoints;
		for (int i = 0; i < totalNodes; i++)
		{
			Util::Point tmp;
			tmp = _nodeList[i].location;
			gpuPoints.push_back(Point_Struct(tmp.x, tmp.y, tmp.z, _nodeList[i].isRef, _nodeList[i].isQuery));
		}
		_gpuvg.init(gpuPoints, obsList.size());
	}

	_gpuvg.generate_graph(obsList);
}


void VisibilityGraph::fetch_graph_cuda()
{
	unsigned matSize = _nodeList.size();
	bool* adjMatrix;
	adjMatrix = _gpuvg.get_adjMatrix();
	//cout << "Elapsed Time: " << (clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << "ms" << endl;
	
#pragma omp parallel for
	// Fill the adjacency matrix
	for (int i = 0; i < matSize; i++)
		for (int j = i; j < matSize; j++)
		{
			_adjMatrix(i, j) = adjMatrix[i*matSize + j] == true ? 1 : 0;
			_adjMatrix(j, i) = _adjMatrix(i, j);
		}
		
	// Fill the adjacency list
	for (int i = 0; i < matSize; i++)
	{
		for (int j = i + 1; j < matSize; j++)
		{
			if (_adjMatrix(i, j) == 1)
			{
				unsigned ix, jx;
				ix = _nodeList[i].listSize;
				jx = _nodeList[j].listSize;
				_nodeList[i].adjList[ix] = Edge(j);
				_nodeList[j].adjList[jx] = Edge(i);
				// move indices forward on the adjLists
				_nodeList[i].listSize++;
				_nodeList[j].listSize++;
			}
		}
	}
	free(adjMatrix);
}

void VisibilityGraph::get_output_cuda(float& mDegree, float& mDepth, float& mEntropy)
{
	_gpuvg.generate_forest();
	int* gpu_output;
	gpu_output = _gpuvg.get_output();
	mDegree = (float)gpu_output[0] / _queryRegion.size();
	mDepth = (float)gpu_output[1] / _queryRegion.size();
	mEntropy = (float)*((float *)&gpu_output[2]) / _queryRegion.size();
	//std::cout << "GPU>>>>>>>>>>>>" << gpuK << "_" << gpuD << "_" << gpuE << std::endl;
	free(gpu_output);
}

float VisibilityGraph::get_meanDegree_cuda()
{
	int* gpu_output;
	float meanDegree;
	gpu_output = _gpuvg.get_degree();
	meanDegree = (float) *gpu_output / _queryRegion.size();
	free(gpu_output);
	return meanDegree;
}

void VisibilityGraph::generate_forest_cuda()
{
	_gpuvg.generate_forest();
}

int* VisibilityGraph::get_nodesDegree_cuda() { return _gpuvg.get_nodesDegree(); }
int* VisibilityGraph::get_nodesDepth_cuda() { return _gpuvg.get_nodesDepth(); }
float* VisibilityGraph::get_nodesEntropy_cuda() { return _gpuvg.get_nodesEntropy(); }
#endif

