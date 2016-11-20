#ifndef _Structs_H_
#define _Structs_H_

#define SHADOW_OBSTACLE 0
#define BLOCK_OBSTACLE 1


struct Point_Struct {
	float x;
	float y;
	float z;
	bool isRef, isQ;

	Point_Struct(float p_x, float p_y, float p_z, bool ref, bool isq)
	{
		x = p_x;
		y = p_y;
		z = p_z;
		isRef = ref;
		isQ = isq;
	}

	Point_Struct() : x(0), y(0), z(0), isRef(false), isQ(false) {}
};

struct Obstacle_Struct {

	Point_Struct pMin;
	Point_Struct pMax;
	int type;
	float tiltDegree;

	Obstacle_Struct(Point_Struct min, Point_Struct max, int obsType, float tilt)
	{
		pMin = min;
		pMax = max;
		type = obsType;
		tiltDegree = tilt;
	}
};

#endif