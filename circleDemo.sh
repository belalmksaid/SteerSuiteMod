#!/bin/bash

#
# Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
# See license.txt for complete license.
#

./build/bin/steersim -ai rvo2dAI -testcase ../testcases/concentric-circles_500v2.xml -config ./build/config_unix.xml -numFrames 2500
./build/bin/steersim -ai rvo2dAI -testcase ../testcases/concentric-circles_500v2.xml -config ./build/config_unix-timeopt-circle.xml -numFrames 2000
