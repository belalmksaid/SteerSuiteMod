//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include "EgocentricMap.h"

Neuron::Neuron(void)
{
	traversabilityRatio = 1.0; // Initially the neuron is fully traversable. 
	activation = 0.0; 
	activationPed = 1.0;
}

float Neuron::getIntersectedArea(float xlow, float xhigh, float zlow, float zhigh, float r)
{
	float nxlow , nxhigh , nzlow , nzhigh ;
	nxlow = this->centerPosition.x - r;
	nxhigh = this->centerPosition.x + r;
	nzlow = this->centerPosition.z - r;
	nzhigh = this->centerPosition.z + r ;

	float area = 0.0f;

	if (nxhigh <= xlow || nzlow >= zhigh 
		|| nxlow >= xhigh || nzlow > zhigh) 
	{

		// 1) whole building is completely outside the neuron. do nothing.
		// test case float nxlow = 11.0, nxhigh = 22.0, nzlow = 1.0, nzhigh = 2.0;

	} 
	else if (nxlow >= xlow && nxhigh <= xhigh 
		&& nzlow >= zlow && nzhigh <= zhigh) 
	{
		// 2) neuron is inside obstacle
		// test case float nxlow = -10.0, nxhigh = 10.0, nzlow = -10.0, nzhigh = 10.0;
		// test case float nxlow = -9.0, nxhigh = 9.0, nzlow = -9.0, nzhigh = 9.0;
		area = fabsf(nxhigh - nxlow) * fabsf(nzhigh - nzlow); 

	} 
	else if (nxlow <= xlow && nxhigh >= xhigh 
		&& nzlow <= zlow && nzhigh >= zhigh)  
	{
			// 3) neuron is outside obstacle
			// test case float nxlow = -11.0, nxhigh = 11.0, nzlow = -11.0, nzhigh = 11.0;
			area = fabsf(xhigh - xlow) * fabsf(zhigh - zlow); 

	} 
	else if (nxlow <= xlow && nxhigh <= xhigh
		&& nzlow <= zlow && nzhigh >= zhigh) 
	{
			// 4) obstacle cuts neuron to the right 
			area = fabsf(nxhigh - xlow) * fabsf(zhigh-zlow); 
	} 
	else if (nxhigh <= xhigh && nxlow <= xlow
		&& nzhigh >= zhigh && nzlow >= zlow) 
	{
		// 5) obstacle cuts neuron lower right corner
		// test case float nxlow = -11.0, nxhigh = 0.0, nzlow = 0.0, nzhigh = 11.0;
		area = fabsf(nxhigh - xlow) * fabsf(zhigh - nzlow);
	} 
	else if (nxhigh >= xhigh && nxlow <= xlow 
		&& (nzlow < zhigh && nzhigh >= zhigh) && nzlow >= zlow) 
	{
		// 6) obstacle cuts neuron to the bottom
		// test case float nxlow = -11.0, nxhigh = 11.0, nzlow = 0.0, nzhigh = 11.0;
		area = fabsf(xhigh-xlow) * fabsf(zhigh - nzlow);
	} 
	else if (nxlow < xhigh && nxhigh >= xhigh 
		&& nzlow >= zlow && nzhigh >= zhigh ) 
	{
		// 7) obstacle cuts neuron lower left corner
		// test case float nxlow = 0.0, nxhigh = 11.0, nzlow = 0.0, nzhigh = 11.0;
		area = fabsf(xhigh - nxlow) * fabsf(zhigh - nzlow);
	} 
	else if (nxlow >= xlow && nxhigh >= xhigh 
		&& nzhigh >= zhigh && nzlow <= zlow) 
	{  
		// 8) obstacle cuts neuron left
		// test case float nxlow = 0.0, nxhigh = 11.0, nzlow = -11.0, nzhigh = 11.0;
		area = fabsf(xhigh - nxlow) * fabsf(zhigh - zlow);
	} 
	else if ((nxlow < xhigh && nxlow > xlow) && nxhigh >= xhigh
		&& nzhigh > zlow && nzlow <= zlow) 
	{
		// 9) obstacle cuts neuron upper left corner
		// test case float nxlow = 0.0, nxhigh = 11.0, nzlow = -11.0, nzhigh = 0.0;
		area = fabsf(xhigh - nxlow) * fabsf(nzhigh -zlow);
	} 
	else if (nxlow <= xlow && nxhigh >= xhigh
		&& nzhigh > zlow && nzlow < zlow) 
	{
		// 10) obstacle cuts neuron at the top
		// test case float nxlow = -11.0, nxhigh = 11.0, nzlow = -11.0, nzhigh = 0.0;
		area = fabsf(xhigh - xlow) * fabsf(nzhigh - zlow);
	} 
	else if (nxlow <= xlow && (nxhigh > xlow && nxhigh < xhigh)
		&& nzhigh > zlow && nzlow <= zlow) 
	{
		// 11) obstacle cuts neuron upper right corner
		// test case float nxlow = -11.0, nxhigh = 0.0, nzlow = -11.0, nzhigh = 0.0;
		area = fabsf(nxhigh - xlow) * fabsf(nzhigh - zlow);
	} 
	else if (nxlow >= xlow && nxhigh <= xhigh
		&& (nzhigh > zlow && nzhigh <= zhigh )&& nzlow <= zlow) 
	{
		// 12) obstacle cuts neuron upper whole
		//test case float nxlow = -5.0, nxhigh = 5.0, nzlow = -11.0, nzhigh = 0.0;
		area = fabsf(nxhigh - nxlow) * fabsf(nzhigh - zlow);
	} 
	else if (nxlow <= xlow && (nxhigh>xlow && nxhigh<=xhigh)
		&& nzhigh <= zhigh && nzlow >= zlow) 
	{
		// 13) obstacle cuts neuron right whole
		// test case float nxlow = -11.0, nxhigh = 0.0, nzlow = -5.0, nzhigh = 5.0;
		area = fabsf(nxhigh - xlow) * fabsf(nzhigh - nzlow);
	} else if (nxlow >= xlow && nxhigh <= xhigh 
		&& (nzlow < zhigh && nzhigh > zhigh) && nzlow >= zlow) 
	{
		// 14) obstacle cuts neuron bottom whole
		//test case float nxlow = -5.0, nxhigh = 5.0, nzlow = 0, nzhigh = 11.0;
		area = fabsf(nxhigh - nxlow) * fabsf(zhigh - nzlow);
	} 
	else if ((nxlow >= xlow && nxlow < xhigh) && nxhigh >= xhigh 
		&& nzhigh <= zhigh && nzlow >= zlow) 
	{
		// 15) obstacle cuts neuron left whole
		// test case float nxlow = 0.0, nxhigh = 11.0, nzlow = -5, nzhigh = 5.0;
		area = fabsf(nxlow - xhigh) * fabsf(nzhigh - nzlow);
	} else if (nxlow >= xlow && nxhigh <= xhigh 
		&& nzlow <= zlow && nzhigh >= zhigh) 
	{
			// 16) cross obstacle middle hor
			// test case float nxlow = -5.0, nxhigh = 5.0, nzlow = -11.0, nzhigh =11.0;
			area = fabsf(nxlow - nxhigh) * fabsf(zhigh - zlow);
	} else if (nxlow <= xlow && nxhigh >= xhigh 
		&& nzlow >= zlow && nzhigh <= zhigh) 
	{
		// 17) cross obstacle middle vert
		// test case float nxlow = -11.0, nxhigh = 11.0, nzlow = -5.0, nzhigh =5.0;
		area = fabsf(xlow - xhigh) * fabsf(nzhigh - nzlow);
	}

	return area;
}

