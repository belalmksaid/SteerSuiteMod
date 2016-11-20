//RockstarOptimize.cpp

#include "RockstarOptimize.h"
#include "CMAOptimizePrivate.h"
#include <iostream>
#include <fstream>

namespace SteerOpt {

RockstarOptimize::RockstarOptimize(SteerSuite::SimWorld * world, SteerSuite::SteerSuite * steersuite) :
	SteerSimOptimize(world, steersuite) 
{
	// TODO Auto-generated constructor stub

}
RockstarOptimize::~RockstarOptimize() {
	// TODO Auto-generated destructor stub
}

/*
*	Objective of this function is to find a control policy
*	 parameter vector that minimizes the expected cost.
*/

void RockstarOptimize::optimize()
{
	try
	{
		int n_paramter  = 2;

		std::ofstream myfile;
  		myfile.open ("rockstar_output.txt");

		/*** initial Parameter Policy: initial_theta is a row vector. e.g. = zeros(1,10) ***/
		Eigen::VectorXd initial_theta = Eigen::VectorXd::Zero(3.26924,0.98588);
		//initial_theta(0) = 3.26924;
		//initial_theta(1) = 0.98588;

		/*** initial exploration standard deviation: it is a row vector. e.g. =ones(1,10) ***/
		Eigen::VectorXd Initial_StandardDeviation = Eigen::VectorXd::Ones(3.26924,0.98588) * 0.5;
		
		/*** second initial Parameter Policy: theta is a row vector. e.g. = zeros(1,10) ***/
		Eigen::VectorXd theta = Eigen::VectorXd::Zero(3.26924,0.98588);
		//theta(0) = 3.26924;
		//theta(1) = 0.98588;

		//cmaes::CMAES optimizer(initial_theta,Initial_StandardDeviation);
		// rockstar::Rockstar optimizer(initial_theta,Initial_StandardDeviation, n_paramter);

		std::cout << "Starting a steerOpt Optimization" << std::endl;

		for(int i=0; i<1000; i++){ // i.e. run for 2000 rollouts

			/*** computing next potential policy parameter vector ***/
			// optimizer.getNextTheta2Evaluate(theta);

			/*** computing cost from objective function by running simulation ***/

			// double x0[n_paramter]; // (dim, 7.0);
			// x0[0] = theta(0);
			// x0[1] = theta(1);

			// double cost = this->simulate(x0, n_paramter);			
			
			/*** Assign HIGH COST if the algorithm samples outside the optimization region ***/
			// if( (theta(0) > 5.0) || (theta(0) < 1.0) )
				// cost = cost * 10;	//boosting cost for sampling outside
			// if( (theta(1) > 2.0) || (theta(1) < -2.0) )
				// cost = cost * 10;	//boosting cost for sampling outside

			/*** setting cost of last computed policy parameter vector ***/
			// optimizer.setTheCostFromTheLastTheta(cost);

			//if(i % 30 == 0)
			 // std::cout<<optimizer.getRolloutNumber()<<"th rollout, cost: "<<cost<<", sigma: "<<optimizer.getSigma()<<", best_policy: "<<optimizer.getBestEverSeenPolicy()<<std::endl;
			 // myfile << optimizer.getRolloutNumber() <<"th rollout, cost: "<<cost<<", sigma: "<<optimizer.getSigma()<<", best_policy: "<<optimizer.getBestEverSeenPolicy()<<"\n";

			/*** Stopping criterian ***/
			// if(optimizer.isOptimizationDone())
			  // break;			
		}

		// std::cout<<"\nRollouts: "<< optimizer.getRolloutNumber()<<"\nBest Cost: "<<optimizer.getBestCost()<<"\nBest Policy: "<<optimizer.getBestEverSeenPolicy()<<std::endl;

		std::cout<<"Rockstar optimization done..."<<std::endl;
		myfile.close();
	}
	catch (std::exception &e) {

		std::cerr << "\nERROR: exception caught in main:\n" << e.what() << "\n";

		// there is a chance that cerr was re-directed.  If this is true, then also echo
		// the error to the original cerr.
	}
}

/*
*	rosenbrock - default objective function for testing
*/
double RockstarOptimize::rosen(Eigen::VectorXd x){	  
	    double cost;
	    int end = x.rows()-1;
	    Eigen::VectorXd y = x.block(0,0,end,1);
	    Eigen::VectorXd w = x.block(1,0,end,1);
	    Eigen::VectorXd q(end);
	    q.setOnes();
	    q=y-q;

	    cost = (y.cwiseProduct(y) - w).squaredNorm()*100.0 + q.squaredNorm();
	  	return cost;
	}

} /* namespace SteerOpt */
