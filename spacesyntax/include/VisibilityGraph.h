// Author: MahyarKoy @Jan2016

#ifndef _VISIBILITY_GRAPH_H_
#define _VISIBILITY_GRAPH_H_

#include <vector>
#include <iostream>
#include <map>
// #include "SteerLib.h"
#include "util/Geometry.h"
#include "interfaces/SpatialDataBaseInterface.h"
#include "GridPlan.h"
#include <Eigen3.2.7/Dense>
#include "GraphSearch.h"
#include "Structs.h"

#ifdef _WIN32
// on win32, there is an unfortunate conflict between exporting symbols for a
// dynamic/shared library and STL code.  A good document describing the problem
// in detail is http://www.unknownroad.com/rtfm/VisualStudio/warningC4251.html
// the "least evil" solution is just to simply ignore this warning.
#pragma warning( push )
#pragma warning( disable : 4251 )
#endif

namespace SpaceSyntax
{
	enum SPACESYNTAX_API NodeType : int
	{
		SKIP = -3,
		ROOT = -2,
		EMPTY = -1
	};

	enum SPACESYNTAX_API RegionType : unsigned int
	{
		QUERY = 0,
		REF = 1
	};

	class SPACESYNTAX_API Edge
	{
	public:
		Edge(unsigned int nodeIDi) : nodeID(nodeIDi), cost(1) {}
		unsigned int nodeID;
		float cost;
	};

	class SPACESYNTAX_API Node
	{
	public:
		inline Node(Util::Point locationIn) : location(locationIn), parent(NULL), isRef(false), isQuery(false) {}
		Util::Point location;
		std::vector<Edge> adjList;
		Node* parent;
		bool isRef, isQuery;
		unsigned listSize;
	};

	class SPACESYNTAX_API Tree
	{
	public:
		Tree(const std::vector<Node> &nodeList, const Eigen::MatrixXf &adjMatrix, unsigned int origin, bool full = false);
		std::vector<int> get_parentList() { return _parentList; }
		int get_parent(unsigned int index) { return _parentList[index]; }
		unsigned int get_origin() { return _origin; }
		unsigned int get_tail() { return _tail; }
		unsigned int get_depth() { return _depthList.size(); }
		unsigned int get_size() { return _size; }
		std::vector<std::vector<int>> get_depthList() { return _depthList; }
		float get_entropy();

	private:
		std::vector<int> _parentList;
		std::vector<std::vector<int>> _depthList;
		unsigned int _origin, _tail;
		unsigned int _size;
		void _generate(const std::vector<Node> &nodeList, const Eigen::MatrixXf &adjMatrix, bool full = false);
	};

	class SPACESYNTAX_API VisibilityGraph
	{
	public:
		// TODO: make spatialDB usage roboust
		VisibilityGraph(SteerLib::SpatialDataBaseInterface* spacialDB, double x1, double z1, double x2, double z2);
		VisibilityGraph(float x1, float z1, float x2, float z2, float height, unsigned int gridNumX, unsigned int gridNumZ);
		~VisibilityGraph();
		
		// Initializes the adjMatrix and adjLists, only call after all nodes are added, before generate
		void init_graph();
		void add_node(Util::Point location);
		void add_region(double x1, double z1, double x2, double z2);
		void generate_graph();
		void add_node_gp(Util::Point location);
		void add_node_exact(Util::Point location);
		void add_region_gp(float x1, float z1, float x2, float z2, RegionType regionType, unsigned regionID = 0);
		// Must run generate to update the graph
		void generate_graph_gp();
		void generate_graph_gp_omp();
		void generate_graph_gp_omp_v2();
		void generate_graph_cuda();
		void generate_forest_cuda();
		void print_graph();
#ifdef CUDA_ENABLED
		void fetch_graph_cuda();
		void get_output_cuda(float& mDegree, float& mDepth, float& mEntropy);
		float get_meanDegree_cuda();
		int* get_nodesDegree_cuda();
		int* get_nodesDepth_cuda();
		float* get_nodesEntropy_cuda();
#endif

		// Graph handling
		void clear_nodes();
		void setup_gridPlan(float x1, float z1, float x2, float z2, float height, unsigned int gridNumX, unsigned int gridNumZ);
		std::pair<unsigned, bool> get_nodeIndex(Util::Point target);
		int get_nodeIndex_exact(Util::Point location);
		// return the 4 points of a cell containing the point specified by index (topLeft, botLeft, botRight, topRight)
		virtual std::vector<Util::Point> get_cellPoints(unsigned int index);
		// return the 4 points of a cell containing the point specified by location
		virtual std::vector<Util::Point> get_cellPoints(Util::Point target);

		// Obstacle handling
		void clear_obstacles() { _gridPlan.clear_obstacles(); }
		void remove_obstacle(unsigned int index) { _gridPlan.remove_obstacle(index); }
		void modify_obstacle(unsigned int index, Obstacle obstacle) { _gridPlan.modify_obstacle(index, obstacle); }
		void add_obstacles(std::vector<Obstacle> obstacleList) { _gridPlan.add_obstacles(obstacleList); }
		void add_obstacle(Obstacle obstacle) { _gridPlan.add_obstacle(obstacle); }
		std::vector<Obstacle> get_obstacles() { return _gridPlan.get_obstacles(); }

		// Output results
		Eigen::MatrixXf get_adjMatrix();
		std::vector<Node> get_nodeList();
		std::vector<Node> get_queryList();
		std::vector<Node> get_refList();
		std::map<unsigned int, unsigned int> get_queryRegion() { return _queryRegion; }
		std::map<unsigned int, unsigned int> get_refRegion() { return _refRegion; }
		unsigned get_gridID(Util::Point target);
		float get_nodeDegree(unsigned int index);
		float get_nodeDegree(Util::Point target);
		float get_meanDegree();
		float get_adjMatrix(unsigned int index1, unsigned int index2) { return _adjMatrix(index1, index2); } // not robust
		Tree* get_tree(unsigned int origin);
		std::vector<int> get_treeParentList(unsigned int origin);
		float get_treeDepth(unsigned int origin); //delete
		float get_treeDepth(Util::Point target); //delete
		float get_meanTreeDepth(); //delete
		float get_treeEntropy(unsigned int origin); //delete
		float get_treeEntropy(Util::Point target); //delete
		float get_meanTreeEntropy(); //delete
		std::pair<float, float> get_treeProp(unsigned int origin);
		std::pair<float, float> get_treeProp(Util::Point target);
		std::pair<float, float> get_meanTreeProp();
		bool check_connection();
	
		// TODO: get_node_byIndex and get_node_byLocation
	private:
		bool _isInit;
		void add_node(unsigned int index);
		void add_node_query(unsigned int gx, unsigned int gz);
		void add_node_ref(unsigned int gx, unsigned int gz);
		std::vector<Node> _nodeList;
		Eigen::MatrixXf _adjMatrix;
		SteerLib::SpatialDataBaseInterface* _spacialDB;
		GridPlan _gridPlan;
		std::map<unsigned int, unsigned int> _queryRegion, _refRegion;
#ifdef CUDA_ENABLED
		gpuVG _gpuvg;
#endif
	};

}

#endif
