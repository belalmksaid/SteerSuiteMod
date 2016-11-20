// steerplug.cpp : Defines the exported functions for the DLL application.
//

#include "steerplug.h"
#include <iostream>


// This is an example of an exported variable
STEERPLUG_API int nsteerplug=0;

// This is an example of an exported function.
STEERPLUG_API int fnsteerplug(void)
{
    return 42;
}

extern double optimize(double num_games)
{

	// This is part of the DLL, so we can call any function we want
	// in the C++. The parameters can have any names we want to give
	// them and they don't need to match the extern declaration.
	return 3 * num_games;
}

extern double optimize2(double const* points, size_t point_count, size_t const * faces, size_t face_count)
{
	std::cout << "Points: " << std::endl;
	for (size_t i = 0; i < point_count; i += 3)
	{
		std::cout << "Point: " << points[i] << ", " << points[i + 1] << ", " << points[i + 2] << std::endl;
	}

	std::cout << "Faces: " << std::endl;

	for (size_t i = 0; i < face_count; i += 3)
	{
		std::cout << "Point: " << faces[i] << ", " << faces[i + 1] << ", " << faces[i + 2] << std::endl;
	}
	return 1.1;
}

// This is the constructor of a class that has been exported.
// see steerplug.h for the class definition
Csteerplug::Csteerplug()
{
    return;
}
