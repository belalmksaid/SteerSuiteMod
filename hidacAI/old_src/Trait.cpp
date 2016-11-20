//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include "Trait.h"




CTrait::CTrait(void)
{
	mFriendship = false;
	
	//Contagion
	SetDose(0);
	SetStatus(SUSCEPTIBLE);		
	SetDoseThreshold();
	mEmpathy = 0; //initially no contagion

	mPanic = 0;
	mImpatience = 0;
	mImbalance = 0;

	SetPersonalSpace(MED);
	SetWaitingRadius(MED);

	
	mWaitingTimer = 5 + rand()%5;


	mLynching = false;


}

CTrait::~CTrait(void)
{
}



////////////////////////////////////////////////
// Set and get the personality characteristics

void CTrait::SetFFM(int ind, double value, double std)
{
	mPersonality[ind].value = value;
	mPersonality[ind].std = std;
	
}


///////////////////////////////////////
// sampling of the distributions

double  CTrait::GetFFM(int ind)
{
	double val = normalDist(mPersonality[ind].value, mPersonality[ind].std);
	//truncate to -10 or 10
	if(val < -10)
		val = -10;
	if(val > 10)
		val = 10;
	return val;
}




//////////////////////////////////////////////////////////
//Contagion Model
//////////////////////////////////////////////////////////

void CTrait::SetDoseThreshold()
{
	mDoseThreshold =  logNormalDist(580,DOSE_VARIANCE);
}

void CTrait::AddDose()
{
	float dose;
	dose = logNormalDist(DOSE_MEAN,DOSE_VARIANCE);

	//take some percentage of the normal dose depending on empathy
	mDose += dose*mEmpathy;

	if(mDose > mDoseThreshold)
		SetStatus(INFECTED);
	else
		SetStatus(SUSCEPTIBLE);
}

void CTrait::SetStatus(int status)
{
	mStatus = status;

	if(status == INFECTED)
	{
		if(mFriendship)
			SetHairColorPersonality(1.0,0,0,1.0);
		mDose = mDoseThreshold;
	}
	
	else if(status == SUSCEPTIBLE && !mFriendship)
		SetHairColorPersonality(mDose/mDoseThreshold,0,0,1.0);
	else if(!GetFriendship())
		SetHairColorPersonality(0,0,1.0,1.0);
}


void CTrait::SetHairColorPersonality(float r, float g, float b, float a)
{
	mColorHairPersonality[0] = r; 
	mColorHairPersonality[1] = g; 
	mColorHairPersonality[2] = b; 
	mColorHairPersonality[3] = a;
}


void CTrait::SetPanicLevel(int panicLevel)
{
	mPanic = panicLevel;
	//funda
//	m_colorHairOriginal[0]=1.0;
//	m_colorHairOriginal[1] = 0.0;
//	m_colorHairOriginal[2]=0.0;
};

void CTrait::IncreasePanic(int level)
{
	if (mPanic < 10) 
		mPanic+=level;
};

//average 0.6 
void CTrait::SetPersonalSpace(int personalSpace)
{
	switch (personalSpace)
	{
		case MIN: mPersonalSpace = 0;//0.05; //intimate space (add this to 0.5 human offset)
				 break;
		case MED: mPersonalSpace = 0;//0.1;//0.2;  
				  break;
		case MAX: mPersonalSpace = 0.3;
			  break;
	};
}


void CTrait::SetWaitingRadius(int radious)
{
	switch (radious)
	{
		case MIN: mWaitingRadius = 0.25;
				  break;
		case MED: mWaitingRadius = 0.45;  
				  break;
		case MAX: mWaitingRadius = 0.65;
				  break;
	};
}

void CTrait::SetWaitingTimer(int timer)
{
	
	switch (timer)
	{
		case MIN: mWaitingTimer = 1 + rand()%5;
			break;
		case MED: mWaitingTimer = 5 + rand()%5;
			break;
		case MAX: mWaitingTimer = 50 + rand()%5;
			break;
	};
}
//
// by mRightPreference/5 probability, right will be selected
//
int CTrait::ComputeRightPreference()
{
	int val = (rand()% 5 + 1)/5.0;


	if( mRightPreference >= val) 
		return RIGHT;
	else
		return LEFT; 

}

