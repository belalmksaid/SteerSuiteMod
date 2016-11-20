//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#pragma once
#include <iostream>
#include <vector>
#include <string>
#include "statistics.h"
#include "globalvbles.h"


#define SUSCEPTIBLE 1
#define INFECTED 2
#define REMOVED 3

#define DOSE_MEAN		1
#define DOSE_VARIANCE	0.433	

#define RIGHT	1
#define	LEFT	-1


//////////////////////////////////////////
// Ocean personality model.
// Stored as value -10 to 10
// and standard deviation.
// Essentially a distribution
// openness, conscientiousness,
// extroversion, agreeableness, and neurotisism
struct FFM {  //five factor model of personality
	double  value;
	double std;
}; 

enum PERSONALITY_TYPE {
	OPEN, CONSCIENT, EXTRO, AGREE, NEURO
};


class CTrait
{
public:
	CTrait(void);
	~CTrait(void);

private:

	FFM		mPersonality[5];	// the personality of the agent
	bool	mFriendship;

	//CONTAGION
	int		mStatus;
	double	mDose;
	double	mDoseThreshold;
	double	mEmpathy;	//depends on the personality factors

	float	mColorHairPersonality[4];


	int		mImpatience;		// Scale from 0 to 10, where 0 means very patient and 10 complete impatient
	int		mPanic;				// Scale from 0 to 10, where 0 means calm and 10 complete panic
	float	mImbalance;		// how much the person can mantain the equilibrium, 4=fall; 1..3 consecutive pushes. 0=standing

	float	mPersonalSpace;

	float	mWaitingRadius;

	int		mWaitingTimer;		// Waiting timer that the user defines as MIN, MED or MAX

	int		mRightPreference; //gets a value in the range [1..5]

	int		mTaskCount; //number of tasks that an agent must perform - used to test openness to experience

	int		mGroupId; //Group index that the person belongs to

	bool	mIsVictim;
	bool	mLynching;
	


public:
	
	// Set and get the personality characteristics

	void	SetFFM(int ind, double value, double std);
	void	SetFFMValue(int ind, double value){mPersonality[ind].value = value;}
	void	SetFFMStd(int ind, double std){mPersonality[ind].std = std;}
	double	GetFFM(int ind);

	void	SetFriendship(bool friendship){mFriendship = friendship;}
	bool	GetFriendship(){return mFriendship;}



	//CONTAGION
	void	SetStatus(int status);
	int		GetStatus(void){return mStatus;};
	void	SetDose(double dose){mDose = dose;};
	double	GetDose(void){return mDose;};
	void	AddDose();
	void	SetDoseThreshold();
	double	GetDoseThreshold(){return mDoseThreshold;};

	void	SetEmpathy(double empathy){mEmpathy = empathy;}
	double	GetEmpathy(){return mEmpathy;}

	void	SetHairColorPersonality(float r, float g, float b, float a); 
	float*  GetHairColorPersonality() {return mColorHairPersonality;}

	int		GetPanicLevel(){return mPanic;}
	void	SetPanicLevel(int panicLevel);
	void	IncreasePanic(int level);


	void	SetImpatience(int pat){mImpatience = pat;}
	int		GetImpatience() {return mImpatience;}

	void	SetImbalance(float equil){mImbalance = equil;}
	float	GetImbalance(){return mImbalance;}
	void	IncreaseImbalance(float e)	{mImbalance+=e;}

	float	GetPersonalSpace() {return mPersonalSpace;}
	void	SetPersonalSpace(int personalSpace);

	void	SetWaitingRadius(int radius);
	float	GetWaitingRadius(){return mWaitingRadius;};

	void	SetWaitingTimer(int timer);
	int		GetWaitingTimer(){return mWaitingTimer;};
	

	void	SetRightPreference(int val){mRightPreference = val;}
	int		ComputeRightPreference();

	void	SetTaskCount(int val) {mTaskCount = val;}
	int		GetTaskCount() {return mTaskCount;}

	void	SetGroupId(int groupId) {mGroupId = groupId;};
	int		GetGroupId() {return mGroupId;};

	
	void	SetIsVictim(bool val){mIsVictim = val;}
	bool	GetIsVictim(){return mIsVictim;}

	void	SetLynching(bool val){mLynching = val;}
	bool	GetLynching(){return mLynching;}



	




	
	
};
