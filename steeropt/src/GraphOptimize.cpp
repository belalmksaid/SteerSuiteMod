/*
 * GraphOptimize.cpp
 *
 *  Created on: 2015-12-08
 *      Author: gberseth
 */

#include "GraphOptimize.h"
#include "Graph.h"
#include "GraphFunction.h"
#include "GraphOptimizePrivate.h"

namespace Graphing {

GraphOptimize::GraphOptimize(Graph g, OptimizationParameters params) {
	// TODO Auto-generated constructor stub
	this->_graph = g;
	this->_params = params;
	this->_best_f = 1000000000.0;
	this->_best_graph = g;
}

GraphOptimize::~GraphOptimize() {
	// TODO Auto-generated destructor stub
}

void GraphOptimize::optimize()
{

	try
	{
		//
		// allocate and use the engine driver
		//
		const int dim = this->_params.size(); // problem dimensions.
		double * lbounds = new double[dim];
		double * ubounds = new double[dim]; // arrays for lower and upper parameter bounds, respectively
		// (2.09597,0,3.88671)

		std::vector<double> x0; // (dim, 7.0);
		for (size_t p=0; p < dim; p++)
		{
			x0.push_back(this->_params._parameters.at(p)._x0);
			lbounds[p] = this->_params._parameters.at(p)._lb;
			ubounds[p] = this->_params._parameters.at(p)._ub;
		}
		std::cout.setf( std::ios_base::scientific );
		std::cout.precision( 10 );


		double sigma = 0.13;
		libcmaes::GenoPheno<libcmaes::pwqBoundStrategy> gp(lbounds,ubounds,dim);
		libcmaes::CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy> > cmaparams(x0, sigma, -1, 0, gp);
		cmaparams.set_algo(aCMAES);
		cmaparams.set_max_fevals(5000);
		// cmaparams.set_restarts(3);
		std::cout << "Starting a steerOpt Optimization" << std::endl;
		// GraphOptimizePrivate * cmaOP = new GraphOptimizePrivate(this->_graph, this->_params);


		this->_cmasols = libcmaes::cmaes<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy> >(this->eval, cmaparams);
		std::cout << "best solution: " << this->_cmasols << std::endl;
		std::cout << "best parameters: " << this->_cmasols.best_candidate().get_x_dvec() << std::endl;
		std::cout << "best function value: " << this->_cmasols.best_candidate().get_fvalue() << std::endl;
		std::cout << "optimization took " << this->_cmasols.elapsed_time() / 1000.0 << " seconds\n";
		delete lbounds;
		delete ubounds;
	}
	catch (std::exception &e)
	{

		std::cerr << "\nERROR: exception caught in main:\n" << e.what() << "\n";

		// there is a chance that cerr was re-directed.  If this is true, then also echo
		// the error to the original cerr.
	}

}

double GraphOptimize::update(const double *x, const int N)
{
	GraphFunction gf;
	Graph tmp_graph;
	tmp_graph.initFrom(this->_graph); // duplicate
	std::cout << "Length of graph: " << gf.graphLength(tmp_graph) << std::endl;
	for (size_t p=0; p < N; p++)
	{
		// std::cout << "x[" << p << "] = " << x[p] << std::endl;
		for (size_t ns=0; ns < this->_params._parameters[p]._node_ids.size(); ns++ )
		{	/// Assuing the dimensions are equal to the number of nodes
			OptimizationParameter param = this->_params._parameters[p];
			Eigen::Vector3d pos = tmp_graph.nodes[this->_params._parameters[p]._node_ids[ns]];
			tmp_graph.nodes[this->_params._parameters[p]._node_ids[ns]] += param.getUpdatedPos(x[p], pos);
		}
	}


	// std::cout << "Length of graph: " << gf.graphLength(tmp_graph) << std::endl;


	double length = gf.graphLength(tmp_graph);
	if (length < this->_best_f)
	{
		_best_x.clear();
		std::cout << "Found better graph: " << length << std::endl;
		this->_best_f = length;
		this->_best_graph = tmp_graph;
		for (size_t v=0; v < tmp_graph.nodes.size(); v++)
		{
			std::cout << tmp_graph.nodes[v] << std::endl;
		}
		for (size_t p=0; p < N; p++)
		{
			std::cout << "x[" << p << "] = " << x[p] << std::endl;
			_best_x.push_back(x[p]);
		}

	}
	return length;

}

} /* namespace Graphing */
