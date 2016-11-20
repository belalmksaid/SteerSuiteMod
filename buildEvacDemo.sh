#!/bin/bash

#
# Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
# See license.txt for complete license.
#

./build/bin/steersim  -ai rvo2dAI -config ./build/config_unix.xml -testcase ../testcases/office-building.xml -numFrames 2500

./build/bin/steersim  -ai rvo2dAI -config ./build/config_unix-timeopt.xml -testcase ../testcases/office-building.xml -numFrames 2500
