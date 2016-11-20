//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#ifndef __GLFW_ENGINE_DRIVER_ACCLEMSH_H__
#define __GLFW_ENGINE_DRIVER_ACCLEMSH_H__

/// @file GLFWEngineDriver.h
/// @brief Declares the GLFWEngineDriver class


//
// GLFWEngineDriver object:  receives an exisiting SteerLib::SimulationEngine, and runs it properly.
//

#ifdef ENABLE_GUI
#ifdef ENABLE_GLFW

#include "SteerLib.h"
#include "glfw/include/GL/glfw.h"
// #include "core/SimAntTweakBar.h"
#include "util/FrameSaver.h"
#include "core/GLFWEngineDriver.h"

/**
 * @brief A GUI back-end that controls a SteerLib::SimulationEngine.
 *
 * This back-end is used to visualize the contents of a simulation interactively.
 *
 * This class uses GLFW, which is more recent than the traditional GLUT.  Like GLUT, 
 * GLFW is pure C, so it can only invoke callbacks trhough static non-member wrappers.
 * Because of this, this class uses a singleton design pattern.
 *
 * @see
 *   - CommandLineEngineDriver to control a SimulationEngine without a GUI.
 */
class GLFWEngineDriverACCLMesh : public GLFWEngineDriver
{
public:

	// void processWindowResizedEvent(int width, int height);
	// void processKeyPressEvent(int key, int action);
	void processMouseMovementEvent(int x, int y);
	// void processMouseButtonEvent(int button, int action);
	// void processMouseWheelEvent(int pos);
	// int processWindowCloseEvent();

protected:
	/// Use static function GLFWEngineDriver::getInstance() instead.
	GLFWEngineDriverACCLMesh();
	~GLFWEngineDriverACCLMesh() {}

};


#endif // ifdef ENABLE_GLFW
#endif // ifdef ENABLE_GUI

#endif // end of __GLFW_ENGINE_DRIVER_ACCLEMSH_H__
