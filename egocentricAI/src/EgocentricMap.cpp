//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include "EgocentricMap.h"
#include <math.h>
#include "util/DrawLib.h"
#include <iostream>
#include "EgocentricAIModule.h"

using namespace EgocentricGlobals;
//=================================================================================
//=================================================================================
EgocentricMap::EgocentricMap(int numberOfNeuronsPerLevel, int numberOfLevels, 
							 Point centerPosition, Point goalPosition)
{
	initialise(numberOfNeuronsPerLevel,numberOfLevels,centerPosition, goalPosition,false);

}

//=================================================================================
//=================================================================================
void EgocentricMap::updateMapAtEveryMovement(Point pos, float theta)
{
	root->centerPosition = pos;
	this->populateCenterPositionOfAllNeurons(theta);
}

//=================================================================================
//=================================================================================
void EgocentricMap::initialise(int numberOfNeuronsPerLevel, int numberOfLevels,
							   Point centerPosition, Point goalPosition, bool scaleMap)
{
	
	//TODO:::: Place minimum limit on numberOfNeuronsPerLevel [ > PI ] && numberOfLevels

	// Initializing numberOfNeuronsPerLevel and numberOfLevels
	this->numberOfLevels = numberOfLevels;
	this->numberOfNeuronsPerLevel = numberOfNeuronsPerLevel;


	// Dynamically allocating root and neuron[]
	root = (Neuron *) malloc(sizeof(Neuron));
	neuron = (Neuron **)calloc(numberOfNeuronsPerLevel*numberOfLevels,sizeof(Neuron*));

	for(int i = 0 ; i < numberOfNeuronsPerLevel*numberOfLevels ; i++)
		neuron[i] = (Neuron *)malloc(sizeof(Neuron));


	// Dynamically allocating the level arrays.
	R = (float *) malloc(numberOfLevels * sizeof(float));
	r = (float *) malloc(numberOfLevels * sizeof(float));
	weight = (float *) malloc(numberOfLevels * sizeof(float));


	this->root->centerPosition = centerPosition;
	this->goalPosition = goalPosition;
	this->directionTheta = 0.0;
	this->neuronId = -1;

	this->estimateArrayr();
	
	// CHANGE 5.2 -- INTRODCUING scaleMap
	//this->estimateArrayweight(goalPosition);
	this->estimateArrayweight(goalPosition, scaleMap);

	this->estimateArrayR();
	this->populateCenterPositionOfAllNeurons(0.0);

	//CHANGE -- INTRODUCING SPEED AFFORDANCE
	//for(int i = 0 ; i < max_sacount ; i++)
	//{
	//	this->sA[i].neuronId = -1;
	//	this->sA[i].speed = 0.0f;
	//	this->sA[i].direction = Vector(0.0f,0.0f,0.0f);
	//}

	//this->sACount = 0;

	// TO SOLVE THE PROBLEM OF VIBRATION,
	// INTRODUCING targetDirection

	prevTargetDirection.x = 0.0f;
	prevTargetDirection.y = 0.0f;
	prevTargetDirection.z = 0.0f;


}


//=================================================================================
//=================================================================================
int EgocentricMap::getNeuronIdFromPositionVector(Point position, bool exact)
{
	int nearestNeuronId = 0;
	
	// initializing distance to distance between neuron[0] and root.
	float distance = (this->neuron[0]->centerPosition - position).lengthSquared();
	float newDistance;
	for(int a = 0 ; a < numberOfLevels ; a++)
	{
		for( int b = 0 ; b < numberOfNeuronsPerLevel ; b ++)
		{
			if(isPositionWithinNeuronArea(a*numberOfNeuronsPerLevel + b, position) == true)
			{
				return (a*numberOfNeuronsPerLevel + b);

			}
			newDistance = (this->neuron[a*numberOfNeuronsPerLevel + b]->centerPosition - position).lengthSquared();
			if (distance > newDistance)
			{
				distance = newDistance;
				nearestNeuronId = a*numberOfNeuronsPerLevel + b;
			}
		}


	}
	if(exact == false)
		return nearestNeuronId;
	else
		return -1;
}

//=================================================================================
//=================================================================================
bool EgocentricMap::isPositionWithinNeuronArea(int neuronId, Point position)
{
	float centerX = this->neuron[neuronId]->centerPosition.x;
	float centerZ = this->neuron[neuronId]->centerPosition.z;

	//To get level number from neuronID
	int level = neuronId / numberOfNeuronsPerLevel;

	float xLow = centerX - r[level];
	float xHigh = centerX + r[level];
	float zLow = centerZ - r[level];
	float zHigh = centerZ + r[level];

	float x = position.x;
	float z = position.z;

	if( x > xLow && x < xHigh && z > zLow && z < zHigh) return true;
	else return false;


}

//=================================================================================
//=================================================================================
//float EgocentricMap::neuronArea(int neuronId)
//{
//	int level = neuronId / numberOfNeuronsPerLevel;
//	return pow(2*r[level], 2);
//}

//=================================================================================
//=================================================================================
int EgocentricMap::getForwardNeuronId(int id)
{
	if(id > numberOfLevels * numberOfNeuronsPerLevel ) return -1;

	int forwardId = id + numberOfNeuronsPerLevel;
	if(forwardId < numberOfLevels * numberOfNeuronsPerLevel )
		return forwardId;
	else 
		return -1;
}

//=================================================================================
//=================================================================================
int EgocentricMap::getRightNeuronId(int id)
{
	if(id > numberOfLevels * numberOfNeuronsPerLevel ) return -1;
	int rightId;

	if((id+1)%numberOfNeuronsPerLevel == 0)
		rightId = id - (numberOfNeuronsPerLevel -1);
	else rightId = id + 1;
	
	return rightId;
}

//=================================================================================
//=================================================================================
int EgocentricMap::getBackwardNeuronId(int id)
{
	if(id > numberOfLevels * numberOfNeuronsPerLevel ) return -1;

	int backwardId = id - numberOfNeuronsPerLevel;
	if(backwardId >= 0 )
		return backwardId;
	else 
		return -1;
}

//=================================================================================
//=================================================================================
int EgocentricMap::getLeftNeuronId(int id)
{
	if(id > numberOfLevels * numberOfNeuronsPerLevel ) return -1;

	int leftId;
	
	if(id %  numberOfNeuronsPerLevel != 0)
		leftId = (id - 1);
	else
		leftId = id + numberOfNeuronsPerLevel - 1;
	
	return leftId;
	
}

//=================================================================================
//=================================================================================
void EgocentricMap::populateCenterPositionOfAllNeurons(float theta)
{
	float angleStep = 360.0f / numberOfNeuronsPerLevel;
	directionTheta += theta;
	if(directionTheta > 360.0)
		directionTheta -= 360.0;

	for (int i=0; i < numberOfLevels ; i++ )
	{
		float currentAngle = directionTheta;
		for(int j=0; j < numberOfNeuronsPerLevel; j ++)
		{
			float x = root->centerPosition.x + R[i] * cosf(currentAngle * M_PI_OVER_180);
			float z = root->centerPosition.z + R[i] * sinf(currentAngle * M_PI_OVER_180);


			this->neuron[i*numberOfNeuronsPerLevel + j]->centerPosition = Point(x,0,z);

			currentAngle += angleStep;
			if(currentAngle > 360.0)
				currentAngle -= 360;


		}
	}
}

//=================================================================================
//=================================================================================
void EgocentricMap::estimateArrayr()
{

	/*
	Math:

	We know, 
	R = numberOfNeuronsPerLevel * r / PI

	Initially,
	R = r + pedestrian_radius
	numberOfNeuronsPerLevel * r / PI = r + pedestrian_radius

	r(numberOfNeuronsPerLevel/ PI - 1) = pedestrian_radius
	r = PI/(numberOfNeuronsPerLevel - PI) * pedestrian_radius

	We know,

	rnext = (numberOfNeuronsPerLevel + PI)/(numberOfNeuronsPerLevel - PI) * r


	*/

	r[0] = (float) M_PI/(numberOfNeuronsPerLevel - (float)M_PI) * (float)pedestrian_radius;
	for(int i = 1 ; i < numberOfLevels ; i ++)
	{
		r[i] = (float) ((float)numberOfNeuronsPerLevel + (float)M_PI)/((float)numberOfNeuronsPerLevel - (float)M_PI) * r[i-1];
	}

}

//=================================================================================
//=================================================================================
void EgocentricMap::estimateArrayweight(Point goalPosition, bool scaleMap)
{
	/*

	Assuming we want to put goal on center of extreme neuron.
	[centerX,centerZ]..2r[0]dash..|...2r[1]dash...|....2r[2]dash....|........2r[numberOfLevels-2]dash........|.....r[numberOfLevels-1]dash.....[x,z]

	
	Now, 

	D = sqr( pow((x - centerX),2) + pow((z - centerZ),2) ... (6)
	
	D = 2r1dash + 2r2dash + 2r3dash .... 2r(n-1)dash + rndash ... (7)

	where,

	rdash[i] = r[i] * weight[i] 

	Note: weight[0] = 1.
	
	Therefore, equation is of the form:

	a + bw[1] + cw[2] + dw[3] ..... zw[numberOfLevels - 1] = D
	bw[1] + cw[2] + dw[3] ..... zw[numberOfLevels - 1] = D - a

	
	IMPORTANT PRECONDITION::
	if D >= 2r[0] + 2r[1] + .... 2r[numberOfLevels - 2] + r[numberOfLevels -1]
		continue
	else set all weights to 1.


	*/

	float x = goalPosition.x;
	float z = goalPosition.z;
	float centerX = this->root->centerPosition.x;
	float centerZ = this->root->centerPosition.z;

	float rSum = 0.0;
	for(int i = 0 ; i < numberOfLevels -1 ; i++)
		rSum = rSum + 2 * r[i];

	rSum += r[numberOfLevels-1];
	

	float D = sqrt( pow((x - centerX),2) + pow((z - centerZ),2));
	D = D - r[0];

	
	/*
	A certain number of levels will always have weight of 1.0
	TODO:: How do we figure how many levels ?? For now, assuming 4.
	*/

	weight[0] = 1.0;
	weight[1] = 1.0;
	weight[2] = 1.0;
	weight[3] = 1.0;
	
	//Precondition.. 
	if( D < rSum || scaleMap == false)
	{
			for(int i = 1 ; i < numberOfLevels - 1 ; i++)
				weight[i] = 1.0 ;
			return;
	}
	else
	{	float Ddash =0.0;
		int initialLevelStart = 4;
		float initialIncrement = 0.1f;
		float increment = 0.1f;
		int levelStart = 4;

		do
		{
			for(int i = levelStart ; i < numberOfLevels  ; i++)
				weight[i] = weight[i-1] + increment;
			
			Ddash = 0.0;
			for(int j=1; j < numberOfLevels ; j++)
				Ddash += 2 * r[j] * weight[j];
			
			increment +=0.1f;
			levelStart ++;

			if(levelStart == numberOfLevels)
			{
				// We come here if weights still haven't been able to bring goal into area.
				initialIncrement +=0.1f;
				initialLevelStart ++;
				increment = initialIncrement ;
				levelStart = initialLevelStart;


				if(initialLevelStart == numberOfLevels)
				{
					/*
					Time to increase the number of levels and redefine the map.
					*/
					printf("resetting");
					this->reset();
					// CHANGE 5.4
					//this->initialise(this->numberOfNeuronsPerLevel, this->numberOfLevels + 1,
					//				this->root->centerPosition, this->goalPosition);
					this->initialise(this->numberOfNeuronsPerLevel, this->numberOfLevels + 1,
									this->root->centerPosition, this->goalPosition,scaleMap);
					return;
				}	
			}

		}
		while(Ddash < D);

	}


	/* 
	Now we need to update r[i] to accomodate these weights.
	*/

	for(int i=0;i<numberOfLevels; i ++)
		r[i] = r[i] * weight[i];





}

//=================================================================================
//=================================================================================
void EgocentricMap::estimateArrayR()
{
	/*
	We know,

	R = (PI / numberOfNeuronsPerLevel ) * r

	*/
	for(int i = 0 ; i < numberOfLevels ; i++)
	{
		R[i] = (numberOfNeuronsPerLevel / (float)M_PI) * r[i];
		//printf("R[%d] = %f, r[%d] = %f\n",i,R[i],i,r[i]);
	}


}

////=================================================================================
////=================================================================================
void EgocentricMap::spreadActivation(int id)
{

	
	//You call this function passing in goal neuron id whose activation is already initialized to MAX_ACTIVATION.
	if(id > numberOfLevels * numberOfNeuronsPerLevel  || id < 0 )
	{
		//printf("Invalid id in spreadActivation() \n");
		return;
	}

	Neuron *goal = this->neuron[id];
	int forwardId = this->getForwardNeuronId(id);
	int rightId = this->getRightNeuronId(id);
	int backwardId = this->getBackwardNeuronId(id);
	int leftId = this->getLeftNeuronId(id);

	Neuron *forward = NULL;
	Neuron *right = NULL;
	Neuron *backward = NULL;
	Neuron *left = NULL;

	if(goal->activation < 0.1 ) return;
	
	if(forwardId != -1 ) forward = this->neuron[forwardId];
	if(rightId != -1 ) right = this->neuron[rightId];
	if(backwardId != -1 )backward = this->neuron[backwardId];
	if(leftId != -1 )left = this->neuron[leftId];
	

	/* to determine the activation of these guys on the basis of 
		a) goal activation. 
		b) traversability ratio
	*/

	/*
	What if the neuron has already been traversed before? 
	Compute new activation. If greater than current activation then consider else ignore.
	*/
	
	// the jury is still out.  MAY REMOVE.
	float distanceF = 0.0f,distanceR = 0.0f ,distanceL = 0.0f ,distanceB = 0.0f;
	
	//if(id == goalId)
	//{
	//	//Right at the beginning.
	//	//introduce distance from goal into the activation decay.
	//	if(forwardId != -1)
	//		distanceF = (this->goalPosition - this->neuron[forwardId]->centerPosition).lengthSquared();
	//	if(rightId != -1)
	//		distanceR = (this->goalPosition - this->neuron[rightId]->centerPosition).lengthSquared();
	//	if(leftId != -1)
	//		distanceL = (this->goalPosition - this->neuron[leftId]->centerPosition).lengthSquared();
	//	if(backwardId != -1)
	//		distanceB = (this->goalPosition - this->neuron[backwardId]->centerPosition).lengthSquared();

	//	printf(" id %d goalID : %d FFF %f %f %f %f\n",id,goalId,distanceF,distanceR,distanceL,distanceB);
	//}

	float newLeftActivationWithDecay = goal->activation * (float)activation_decay - (distanceL * 0.1f);
	float newRightActivationWithDecay = goal->activation * (float)activation_decay - (distanceR * 0.1f);
	float newForwardActivationWithDecay = goal->activation * (float)forward_activation_decay - (distanceF * 0.1f);
	float newBackwardActivationWithDecay = goal->activation * (float)backward_activation_decay - (distanceB * 0.1f);


	if(forwardId != -1 && newForwardActivationWithDecay*forward->traversabilityRatio >= forward->activation)
	{
		//printf("doing forward\n");
		forward->activation = newForwardActivationWithDecay*forward->traversabilityRatio;
		spreadActivation(forwardId);
		//printf("after doing forward\n");
	}

	if(rightId != -1 && newRightActivationWithDecay*right->traversabilityRatio > right->activation)
	{
		//printf("doing right\n");
		right->activation = newRightActivationWithDecay*right->traversabilityRatio;
		spreadActivation(rightId);
	}

	if(backwardId != -1 && newBackwardActivationWithDecay*backward->traversabilityRatio > backward->activation)
	{
		//printf("doing back\n");
		backward->activation = newBackwardActivationWithDecay*backward->traversabilityRatio;
		spreadActivation(backwardId);
	}

	if(leftId != -1 && newLeftActivationWithDecay*left->traversabilityRatio > left->activation)
	{
		//printf("doing left\n");
		left->activation = newLeftActivationWithDecay*left->traversabilityRatio;
		spreadActivation(leftId);
	}

}

//=================================================================================
//=================================================================================
void EgocentricMap::clearActivation()
{
	for(int i = 0 ; i < numberOfLevels * numberOfNeuronsPerLevel ; i++)
	{
		this->neuron[i]->activation = 0.0f;
		this->neuron[i]->activationPed = 1.0f;
		//this->neuron[i]->potentialObstacle = 1.0f;
	}
}

//=================================================================================
//=================================================================================
int EgocentricMap::determineNeuronIdToMoveTo(int neuronIdLowRange, int neurondIdHighRange, bool useTraversability)
{
	//Adopting greedy approach for now.

	// How about maxActivation for 2 levels instead of 1?
	float maxActivation = 0.0;
	
	//There are cases when all activation may be 0.0 
	//In that case, TODO:: ?

	int neuronIdLocal=-1;
	bool flag = false;
	for(int i = neuronIdLowRange ; i != (neurondIdHighRange) ; i = (i +1) % numberOfNeuronsPerLevel )
	{
		//printf("here i:%d nL:%d nH:%d\n",i,neuronIdLowRange,neurondIdHighRange);
		int prev, next;
		int pprev, nnext;
		if(i == 0) 
			prev = numberOfNeuronsPerLevel -1;
		else
			prev = i -1;

		if(i== numberOfNeuronsPerLevel -1) next = 0;
		else
			next = i + 1;

		//CHANGE
		// INTRODUCING WINDOW SIZE OF 5. 
		if(i <= 1)
			pprev = numberOfNeuronsPerLevel - 2;
		else 
			pprev = i - 2;

		nnext = (i+2) % numberOfNeuronsPerLevel ;

		
		//CHANGE 
		if( i == getNeuronIdFromPositionVector(goalPosition,true) ) 
		{
			neuronIdLocal = i; 
			break;
		}
		//if(neuron[i]->activation + neuron[prev]->activation + neuron[next]->activation > maxActivation)

		//CHANGE : INTRODUCING WINDOW SIZE OF 5.
		if(neuron[i]->activation + neuron[prev]->activation + neuron[next]->activation + 
			neuron[nnext]->activation + neuron[pprev]->activation> maxActivation)
		//SECOND CHANGE : INTRODUCING WINDOW SIZE : 5 + 1/3 -- NEURONS OF NEXT LEVEL. 
		//if(neuron[i]->activation + neuron[prev]->activation + neuron[next]->activation + 
		//	neuron[nnext]->activation + neuron[pprev]->activation + 
		//	neuron[i + numberOfNeuronsPerLevel]->activation> maxActivation)
		{
			//maxActivation = neuron[i]->activation + neuron[prev]->activation + neuron[next]->activation; 
			//CHANGE:: INTRODUCING WINDOW SIZE OF 5.
			maxActivation = neuron[i]->activation + neuron[prev]->activation + neuron[next]->activation + 
							neuron[nnext]->activation + neuron[pprev]->activation; 
			//SECOND CHANGE : INTRODUCING WINDOW SIZE : 5 + 1/3 -- NEURONS OF NEXT LEVEL. 
			//maxActivation = neuron[i]->activation + neuron[prev]->activation + neuron[next]->activation + 
			//				neuron[nnext]->activation + neuron[pprev]->activation +
			//				neuron[i + numberOfNeuronsPerLevel]->activation; 

			neuronIdLocal = i;
			//if(neuron[prev]->activation < 0.1 || neuron[next]->activation < 0.1 
			//   || neuron[neuronId]->activation  < 0.1 )
			//	flag = false;
			//CHANGE -- CHANGING CONDITION FORM -- ALSO NOTE CHANGE IN flag initialization from true to false.
			if(neuron[prev]->activation > 0.1 && neuron[next]->activation > 0.1 
			   && neuron[neuronIdLocal]->activation  > 0.1 )
				flag = true;
		}
	}

	//if(flag == false) printf("FLAG FALSE IN DETERMINE NEURON ID !!!!! \n\n\n\n\n");
	if(flag == true || useTraversability == false) return neuronIdLocal;


	// MAJOR CHANGE --> INITIALLY ALWAYS JUST RETURING NEURONID CALCULATED PREVIOUSLY.
	// if we dont have enough activation, we merely pick a neuron with high traversabilty in the 
	// direction we were last traveling ... e->neuronId

	//determining neuronIdLowRange and neuronIdHighRange such that they are in the direction of a goal.
	// i.e choose a free neuron in the direction of the goal, if the activation does not work. 
	if(this->neuronId == -1)
	{
		// this is usually right at the beginning.
		this->neuronId = this->getNeuronIdFromPositionVector(this->goalPosition,false) % numberOfNeuronsPerLevel;
		// I basically get the first level equivalent of the goal neuron.
		neuronIdLowRange = this->neuronId - 3;
		if(neuronIdLowRange < 0) 
			neuronIdLowRange = this->numberOfNeuronsPerLevel + neuronIdLowRange;

		neurondIdHighRange = (this->neuronId + 2) % this->numberOfNeuronsPerLevel;
	}

	float maxTraversability = 0.0f;
	
	for(int i = this->neuronId /*neuronIdLowRange*/ ; i != (neurondIdHighRange) ; i = (i +1) % numberOfNeuronsPerLevel )
	{
		//printf("here i:%d nL:%d nH:%d\n",i,neuronIdLowRange,neurondIdHighRange);
		int prev, next;
		if(i == 0) 
			prev = numberOfNeuronsPerLevel -1;
		else
			prev = i -1;

		if(i== numberOfNeuronsPerLevel -1) next = 0;
		else
			next = i + 1;

		//if(neuron[i]->traversabilityRatio + neuron[prev]->traversabilityRatio + neuron[next]->traversabilityRatio > maxTraversability)
		if(neuron[i]->traversabilityRatio > 0.5 && neuron[prev]->traversabilityRatio > 0.5 && neuron[next]->traversabilityRatio > 0.5 )
		{
			maxTraversability = neuron[i]->traversabilityRatio + neuron[prev]->traversabilityRatio + neuron[next]->traversabilityRatio; 
			neuronIdLocal = i;
			//printf("TRAVERSABILITY DETERMINATION : %d  %d  %d  %d\n\n",this->neuronId,neuronIdLowRange,neurondIdHighRange,neuronIdLocal);
			return neuronIdLocal + 100;
			
			//if(neuron[prev]->activation < 0.1 && neuron[next]->activation < 0.1)
			//	flag = false;
		}
	}
	
	for(int i =  neuronIdLowRange ; i != (this->neuronId) ; i = (i +1) % numberOfNeuronsPerLevel )
	{
		//printf("here i:%d nL:%d nH:%d\n",i,neuronIdLowRange,neurondIdHighRange);
		int prev, next;
		if(i == 0) 
			prev = numberOfNeuronsPerLevel -1;
		else
			prev = i -1;

		if(i== numberOfNeuronsPerLevel -1) next = 0;
		else
			next = i + 1;

		//if(neuron[i]->traversabilityRatio + neuron[prev]->traversabilityRatio + neuron[next]->traversabilityRatio > maxTraversability)
		if(neuron[i]->traversabilityRatio > 0.5 && neuron[prev]->traversabilityRatio > 0.5 && neuron[next]->traversabilityRatio > 0.5 )
		{
			maxTraversability = neuron[i]->traversabilityRatio + neuron[prev]->traversabilityRatio + neuron[next]->traversabilityRatio; 
			neuronIdLocal = i;
			//printf("TRAVERSABILITY DETERMINATION : %d  %d  %d  %d\n\n",this->neuronId,neuronIdLowRange,neurondIdHighRange,neuronIdLocal);

			return neuronIdLocal + 100;
			
			//if(neuron[prev]->activation < 0.1 && neuron[next]->activation < 0.1)
			//	flag = false;
		}
	}

	/*TODO::: WHAT TO DO? */ return neuronId;

	//TODO :: STILL IF WE HAVE NOWHERE TO GO..
	// RETURN -1 ... AND PUT SPEED = 0 AND aimForTargetDirection = false 
	//else return -1; 
}




//=================================================================================
//bool activation == true --> neuronID determined by activation
//	 activation == false --> neuronId determined by traversability ratio
//=================================================================================
Vector EgocentricMap::determineTargetDirection(int neuronId, bool activation, Vector forward)
{


	Vector targetDirection = neuron[neuronId]->centerPosition - root->centerPosition;
	
	// EXPERIMENT NO 1 -- JUST THE CENTER POSITION.
	//return targetDirection;

	Vector positionToGoTo;
	
	//printf("rx : %f rz %f nx : %f nz : %f\n",root->centerPosition.x,root->centerPosition.z,
	//		neuron[neuronId]->centerPosition.x,neuron[neuronId]->centerPosition.z);
	

	//---------------------------------------------------------------------------
	// MAJOR CHANGE -- WE DONT NECESSARILY HAVE TO GOTO CENTER OF ONE NEURON. 
	// INSTEAD TAKE WEIGHED SUM OF WINDOW OF NEURON CENTERS (3/5). 
	int prev, next;
	if(neuronId == 0) 
		prev = numberOfNeuronsPerLevel -1;
	else
		prev = neuronId -1;

	if(neuronId== numberOfNeuronsPerLevel -1) next = 0;
	else
		next = neuronId + 1;

	//CHANGE
	// INTRODUCING WINDOW SIZE OF 5. -- NOT INCOROPORATED YET .. DO WE NEED ? NO.
	//if(neuronId <= 1)
	//	pprev = numberOfNeuronsPerLevel - 2;
	//else 
	//	pprev = neuronId - 2;

	//nnext = (neuronId + 2) % numberOfNeuronsPerLevel ;

	
	//positionToGoTo.x = ( neuron[neuronId]->centerPosition.x * neuron[neuronId]->activation +
	//					 neuron[prev]->centerPosition.x * neuron[prev]->activation +
	//					 neuron[next]->centerPosition.x * neuron[next]->activation ) 
	//					 / ( neuron[neuronId]->activation + neuron[prev]->activation + neuron[next]->activation ) ;

	//positionToGoTo.z = ( neuron[neuronId]->centerPosition.z * neuron[neuronId]->activation +
	//					 neuron[prev]->centerPosition.z * neuron[prev]->activation +
	//					 neuron[next]->centerPosition.z * neuron[next]->activation ) 
	//					 / ( neuron[neuronId]->activation + neuron[prev]->activation + neuron[next]->activation ) ;

	//positionToGoTo.y = 0.0f;

	//CHANGE -- INSTEAD OF TAKING BOTH PREV AND NEXT, JUST CHOOSE THE MAX ONE.?
	//CHANGE -- INTRODUCING bool activation

	int other;
	if(neuron[prev]->activation > neuron[next]->activation)
		other = prev;
	else 
		other = next;

	if(activation == true && (neuron[prev]->activation + neuron[other]->activation) > 0.0f )
	{


		positionToGoTo.x = ( neuron[neuronId]->centerPosition.x * neuron[neuronId]->activation +
							 neuron[other]->centerPosition.x * neuron[other]->activation ) 
							 / ( neuron[neuronId]->activation + neuron[other]->activation) ;

		positionToGoTo.z = ( neuron[neuronId]->centerPosition.z * neuron[neuronId]->activation +
							neuron[other]->centerPosition.z * neuron[other]->activation ) 
							/ ( neuron[neuronId]->activation + neuron[other]->activation) ;

		positionToGoTo.y = 0.0f;
	}
	else
	{


		positionToGoTo.x = ( neuron[neuronId]->centerPosition.x * neuron[neuronId]->traversabilityRatio +
							 neuron[other]->centerPosition.x * neuron[other]->traversabilityRatio ) 
							 / ( neuron[neuronId]->traversabilityRatio + neuron[other]->traversabilityRatio ) ;

		positionToGoTo.z = ( neuron[neuronId]->centerPosition.z * neuron[neuronId]->traversabilityRatio +
							neuron[other]->centerPosition.z * neuron[other]->traversabilityRatio ) 
							/ ( neuron[neuronId]->traversabilityRatio + neuron[other]->traversabilityRatio) ;

		positionToGoTo.y = 0.0f;
	}


	// IVE ADDED THIS

	//neuron[neuronId]->activation = 1.0f;
	//neuron[prev]->activation = 1.0f;
	//neuron[next]->activation = 0.0f;

	if(activation == true)
	{
		//printf("activation values : %f %f %f\n",neuron[neuronId]->activation,neuron[prev]->activation,neuron[next]->activation);

		positionToGoTo = ( neuron[neuronId]->activation * ( neuron[neuronId]->centerPosition - root->centerPosition) 
			+ neuron[prev]->activation * ( neuron[prev]->centerPosition - root->centerPosition) 
			+ neuron[next]->activation * ( neuron[next]->centerPosition - root->centerPosition) ) 
			/ ( neuron[neuronId]->activation + neuron[prev]->activation + neuron[next]->activation );
	}
	else
	{
		//printf("using travesa\n");
		//printf("activation values : %f %f %f\n",neuron[neuronId]->activation,neuron[prev]->activation,neuron[next]->activation);

		positionToGoTo = ( neuron[neuronId]->traversabilityRatio * ( neuron[neuronId]->centerPosition - root->centerPosition) 
			+ neuron[prev]->traversabilityRatio * ( neuron[prev]->centerPosition - root->centerPosition) 
			+ neuron[next]->traversabilityRatio * ( neuron[next]->centerPosition - root->centerPosition) ) 
			/ ( neuron[neuronId]->traversabilityRatio + neuron[prev]->traversabilityRatio + neuron[next]->traversabilityRatio );
	}






	// IVE CHANGED THIS.
	targetDirection = positionToGoTo ;/*- root->centerPosition;*/


	//printf("TARGET DIRECTION !!! %f %f ID's %d %d %d\n\n",targetDirection.x,targetDirection.z,neuronId,next,prev);
	//-----------------------------------------------------------------------------------

	
	//OpenSteer::Vec3 v1 = this->prevTargetDirection;
	//OpenSteer::Vec3 v2 = targetDirection;
	//float w1 = this->neuron[this->prevNeuronId]->activation;
	//float w2 = this->neuron[neuronId]->activation;

	//w1  = 0.5f; w2 = 0.5f;

	//printf("prev td : %f %f %f\n",targetDirection.x,targetDirection.y,targetDirection.z);
	//targetDirection.x = (v1.x * w1 + v2.x * w2) / (w1 + w2);
	//targetDirection.y = (v1.y * w1 + v2.y * w2) / (w1 + w2);
	//targetDirection.z = (v1.z * w1 + v2.z * w2) / (w1 + w2);

	//targetDirection.normalize();
	//printf("new td : %f %f %f\n",targetDirection.x,targetDirection.y,targetDirection.z);

	return targetDirection;
}

//=================================================================================
//=================================================================================
int EgocentricMap::determineNeuronIdWithMaximumActivationAtLevel(int level)
{
	//Adopting greedy approach for now.
	float maxActivation = 0.0;
	
	//There are cases when all activation may be 0.0 --> when goal doesn't fall in map.
	// In that case, this will return -1.

	int neuronId=-1;
	for(int i = level * numberOfNeuronsPerLevel ; i < level * numberOfNeuronsPerLevel + numberOfNeuronsPerLevel ; i++)
		if(neuron[i]->activation > maxActivation)
		{
			maxActivation = neuron[i]->activation;
			neuronId = i;
		}

	return neuronId;
	//THIS WILL RETURN -1 IF THERE IS NO ACTIVATION!
}

//=================================================================================
//=================================================================================
void EgocentricMap::reset()
{
	free(this->root);
	free(this->neuron);
	free(this->weight);
	free(this->r);
	free(this->R);
		
}

void 
EgocentricMap::drawMap ( const MapDisplayChoice & choice )
{
	if ( choice == MapDisplayChoice_NoDisplay ) 
		return;

	for ( int i = 0 ; i < numberOfLevels * numberOfNeuronsPerLevel ; i++ )
	{
		drawNode ( i, choice );
	}
}
	
void 
EgocentricMap::drawNode ( const int & nodeId, const MapDisplayChoice & choice )
{
	Point a,b,c,d;

	Point centerPosition = neuron[nodeId]->centerPosition;
	float radius = r[ (int) (nodeId / numberOfNeuronsPerLevel) ];

	a.x = centerPosition.x - radius;
	a.y = 0.1f;
	a.z = centerPosition.z - radius;

	b.x = centerPosition.x + radius;
	b.y = 0.1f;
	b.z = centerPosition.z -radius;

	c.x = centerPosition.x + radius;
	c.y = 0.1f;
	c.z = centerPosition.z + radius;

	d.x = centerPosition.x - radius;
	d.y = 0.1f;
	d.z = centerPosition.z + radius;


	//Util::DrawLib::glColor(determineNodeColor (nodeId, choice));
	DrawLib::drawQuad ( d, c ,b,a );


}
	
Color 
EgocentricMap::determineNodeColor ( const int & nodeId, const MapDisplayChoice & choice  )
{
	if ( nodeId == goalId ) 
		return Color ( 0.0f, 0.0f, 0.0f );

	if ( nodeId == this->neuronId ) 
		return Color ( 1.0f, 1.0f, 1.0f );

	float determinationFactor ;
	Color color (0.0f,0.0f,0.0f);

	switch ( choice )
	{
	case MapDisplayChoice_Activation :
		if ( nodeId == determineNeuronIdWithMaximumActivationAtLevel ( nodeId / numberOfNeuronsPerLevel ) )
			return gBlack;

		determinationFactor = neuron[nodeId]->activation;
		break;

	case MapDisplayChoice_Traversability :
		determinationFactor = neuron[nodeId]->traversabilityRatio;
		break;

	case MapDisplayChoice_ActivationPed :
		determinationFactor = neuron[nodeId]->activationPed;
		break;

	case MapDisplayChoice_NodeNumber :

		switch( nodeId % 4 )
		{
			case 0 : return Color ( 1.0f, 0.0f, 0.0f ); break;
			case 1 : return Color ( 0.0f, 1.0f, 0.0f ); break;
			case 2 : return Color ( 0.0f, 0.0f, 1.0f ); break;
			case 3 : return Color ( 0.0f, 0.0f, 0.0f ); break;
		}
		break;

	default:
		determinationFactor = neuron[nodeId]->traversabilityRatio;
		break;

	}

	if(determinationFactor < 0.3f)
	{
		//Lets use RED for low determinationFactor
		color.r = determinationFactor + 0.3f;
		color.g = 0.0f;
		color.b = 0.0f;
	}
	else if (determinationFactor < 0.6f)
	{

		//Lets use GREEN for medium determinationFactor
		color.r = 0.0f;
		color.g = determinationFactor * 1.0f/0.6f;
		color.b = 0.0f;

	}

	else
	{
		//Lets use BLUE for high determinationFactor
		color.r = 0.0f;
		color.g = 0.0f;
		color.b = determinationFactor * 1.0f/0.6f; 
	}

	return color;
}
