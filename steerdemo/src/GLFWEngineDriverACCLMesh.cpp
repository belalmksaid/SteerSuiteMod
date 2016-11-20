//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//
//
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//
//
// Copyright (c) 2009-2010 Shawn Singh, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

/// @file GLFWEngineDriver.cpp
/// @brief Implements the GLFWEngineDriver functionality.
///
/// @todo
///   - update documentation in this file
///

#ifdef ENABLE_GUI
#ifdef ENABLE_GLFW

#include <iostream>
#include "SteerLib.h"
#include "core/GLFWEngineDriverACCLMesh.h"

#include "glfw/include/GL/glfw.h"

// #include "AntTweakBar/include/AntTweakBar.h"

using namespace std;
using namespace SteerLib;
using namespace Util;

#define MULTISAMPLE_ARB 0x809D

//
// callback wrappers for GLFW
//
// static void GLFWCALL processWindowResizedEvent(int width, int height) { GLFWEngineDriver::getInstance()->processWindowResizedEvent(width,height); }
// static void GLFWCALL processKeyPressEvent(int key, int action) { GLFWEngineDriver::getInstance()->processKeyPressEvent(key, action); }
// static void GLFWCALL processMouseButtonEvent(int button, int action) { GLFWEngineDriver::getInstance()->processMouseButtonEvent(button, action); }
static void GLFWCALL processMouseMovementEvent(int x, int y) { GLFWEngineDriver::getInstance()->processMouseMovementEvent(x,y); }
// static void GLFWCALL processMouseWheelEvent(int pos) { GLFWEngineDriver::getInstance()->processMouseWheelEvent(pos); }
// static int GLFWCALL processWindowCloseEvent() { return GLFWEngineDriver::getInstance()->processWindowCloseEvent(); }


//
// getInstance()
//
// Singleton trick:  with the static instance in this function, we are guaranteed that the 
// constructor will be called before the instance is ever retrieved.
//
GLFWEngineDriverACCLMesh::GLFWEngineDriverACCLMesh() :
		GLFWEngineDriver()
{

}



void GLFWEngineDriver::processMouseMovementEvent(int x, int y)
{

	// TwEventMousePosGLFW(x, y);
	// get mouse changes
	int deltaX = x - _mouseX;
	int deltaY = y - _mouseY;

	// update mouse position
	_mouseX = x;
	_mouseY = y;

	// camera rotate
	if(_rotateCameraOnMouseMotion)
	{
		float xAdjust = -deltaX * _options->guiOptions.mouseRotationFactor;
		float yAdjust = deltaY * _options->guiOptions.mouseRotationFactor;

		_engine->getCamera().nudgeRotate(yAdjust, xAdjust);
	}

	// camera zoom
	if(_zoomCameraOnMouseMotion)
	{
		float yAdjust = deltaY * _options->guiOptions.mouseZoomFactor;
		_engine->getCamera().nudgeZoom(yAdjust);
	}

	// camera move
	if(_moveCameraOnMouseMotion)
	{
		float xAdjust = deltaX * _options->guiOptions.mouseMovementFactor;
		float yAdjust = deltaY * _options->guiOptions.mouseMovementFactor;

		_engine->getCamera().nudgePosition(xAdjust, yAdjust);
	}

	_engine->processMouseMovementEvent(deltaX, deltaY);
}


#endif // ifdef ENABLE_GLFW
#endif // ifdef ENABLE_GUI
