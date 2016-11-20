//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include "EgocentricConstants.h"
#include "util/Geometry.h"
#include "util/Color.h"
// #include "EgocentricAIModule.h" // for changing parameters

using namespace Util;

class SpeedAffordance;
class Neuron;
class EgocentricMap;


// MUBBASIR TODO -- Please deprecate this class. 
//class SpeedAffordance
//{
//public:
//	int neuronId;
//	Vector direction;
//	float speed;
//};


/* EGOCENTRIC MAP NUMBERING SCHEME FOR NEURONS

			

			2	
		 3	  1
		4	R	0	
		  5	  7 
			6

*/

class EgocentricMap
{
public:

	Neuron *root;
	Neuron **neuron;

	int numberOfLevels;
	int numberOfNeuronsPerLevel;

	//TODO:: create a function setGoalPosition ... this must reinitialise the map i guess. 
	Point goalPosition;

	// MUBBASIR -- Please remove AI Life code 
	//int globalLeader;

	/*
	This stores the neuronId of the previously calculated goal position. If the goal falls outside any neuron area
	for some time, this previous goal Id is used to serve as the goal for activation computation.
	*/

	//TODO :: REMOVE THIS --> NOT NEEDED ??
	int goalId; 
	int neuronId; // Neuron that was selected by determineNeuronId... in the last runEgo.. phase

	/* This represents the initial currentAngle that populateCenterOfAllNeurons starts with. 
	 0 - 360
	*/
	float directionTheta; 

	
	/*
	LEVEL ARRAYS 
	The following are all arrays of size numberOfLevels. 
	They correspond to information for a particular level.
	They are dynamically allocated in the createMap() function.
	*/

	float *r; 
	/*
	radius of the neurons at a particular level. 
	NOTE::: This does not incorporate the weight. 
	r are CONSTANTS.
	*/
	float *R; // radius of the particular level. 
	float *weight; //weights of neurons at a particular level.


	Vector prevTargetDirection; // stores the target direction of the previous time step.
	int prevNeuronId; // stores the neuron id of the previous time step.


	//Introducing Speed Affordance;
	//SpeedAffordance sA[max_sacount+1];
	//int sACount; // number of pedestrians immediately around us .
	
	//void sAClear()
	//{
	//	sACount = 0;
	//}
	
	/* 
	Constructor::
	EgocentricMap(int numberOfNeuronsPerLevel, int numberOfLevels, OpenSteer::Vec3 pos);
	This function does everything. It calls all the functions and sets up the map.
	*/
	EgocentricMap(int numberOfNeuronsPerLevel, int numberOfLevels, 
Point centerPosition, Point goalPosition);
	
	/*
	void updateMapAtEveryMovement(OpenSteer::Vec3 pos)
	
	a)This function updates the center position of the root with pos
	b) It then calls populateCenterPositionOfAllNeurons() to readjust their positions.
	*/
	void updateMapAtEveryMovement(Point pos, float theta);

	//CHANGE 5 -- INTRODUCING ARGUMENT bool scaleMap
	// if scaleMap = true, turn on weights
	// else all weights = 1.0
	
	void initialise(int numberOfNeuronsPerLevel, int numberOfLevels,
		Point centerPosition, Point goalPosition, bool scaleMap);


	/*
	TODO::: WHAT ABOUT THE CRACKS?? 
	*/

	int getNeuronIdFromPositionVector(Point position, bool exact);

	bool isPositionWithinNeuronArea(int neuronId, Point position);

	// MUBBASIR TODO -- Pleas remove
	//float neuronArea(int neuronId); // prob dont need this.

	int getForwardNeuronId(int id);
	int getRightNeuronId(int id);
	int getLeftNeuronId(int id);
	int getBackwardNeuronId(int id);


	/*
	STEP 1 :: Estimate r[]
	void estimater()
	This function is used to determine the values of r[]. 
	Note:: r[] are constant and independant of w.
	*/
	void estimateArrayr();

	/* 
	STEP 2 :: Estimate weight[]
	void estimateArrayweight(OpenSteer::Vec3 goalPosition);
	This function adjusts the weights of the ESM such that the goal position falls within the area of the map.
	It also readjusts the r[] to accomodate the weights.
	*/

	//CHANGE 5.3 
	//void estimateArrayweight(OpenSteer::Vec3 goalPosition);
	void estimateArrayweight(Point goalPosition, bool scaleMap);

	/*
	STEP 3 :: Estimate R[]
	void estimateArrayR()
	This function is used to determine the values of R[]. 
	Note:: R[] is dependant on r[] and weight[]
	*/
	void estimateArrayR();

	/* 
	STEP 4:: Populate center positions of all neurons
	void populateCenterPositionOfAllNeurons()
	This function calculates the center positions all the neurons, given the center position of the root.
	*/
	void populateCenterPositionOfAllNeurons(float theta);

	/*
	void rotateMap(float theta)
	This function rotates the Egocentric map by an angle theta. 
	
	TODO:::
	theta takes an angle between 0 and 360 right now. 
	Change this to account for the form of theta used by Shawn.

	This involves cyclic adjustment of the root->neurons[0 - numberOfNeuronsPerLevel] pointers only. 
	RIGHT??
	*/
	//void rotateMap(float theta);

	/*
	void translateMap( take some arguments)

	*/

	void spreadActivation(int id);
	
	void clearActivation();

	//int determineNeuronIdToMoveTo(int neuronIdLowRange, int neurondIdHighRange);
	//CHANGE --- INTRODUCING bool useTraversability
	int determineNeuronIdToMoveTo(int neuronIdLowRange, int neurondIdHighRange, bool useTraversability);

	int determineNeuronIdWithMaximumActivationAtLevel(int level);

	/*
	target direction is the vector which is the difference between the two points:
	root->centerPosition - neuron[neuronId]->centerPosition
	*/
	//OpenSteer::Vec3 determineTargetDirection(int neuronId);
	//CHANGE -- INTRODUCING bool activation
	//bool activation == true --> neuronID determined by activation
	//	   activation == false --> neuronId determined by traversability ratio
	Vector determineTargetDirection(int neuronId,bool activation, Vector forward);


	/*
	void reset()
	This function frees up memory space of:
	a) root
	b) neuron array 
	c) w, r, R
	*/

	void reset();

	/*
	Display helper functions.
	*/
	void drawMap ( const MapDisplayChoice & choice );
	void drawNode ( const int & nodeId, const MapDisplayChoice & choice );
	Color determineNodeColor ( const int & nodeId, const MapDisplayChoice & choice  );
};



class Neuron
{
public:
	
	/* 
	  0 
	3   1
	  2
	  
	*/
		
	float traversabilityRatio;
	/* traversabilityRatio = number of free grids / total number of grids
	*/

	float activation;
	float activationPed;
	
	//float potentialObstacle;
	/*
	Range : 0 - 1.0 ?? 
	*/

	// Position of center of area covered by neuron.
	Point centerPosition;
	
	//Functions.
	
	Neuron();
	
	/*
	getIntersectedArea(float xlow, float xhigh, float zlow, float zhigh, float r);
	We need to send this function the r[i] at that level as it needs it to compute:
	nxlow etc ... 
	*/

	float getIntersectedArea(float xlow, float xhigh, float zlow, float zhigh, float r);

};

