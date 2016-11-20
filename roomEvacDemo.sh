#!/bin/bash
#
# Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
# See license.txt for complete license.
#

./build/bin/steersim -ai sfAI -testcase ../testcases/bottleneck-evac2.xml -config ./build/config_unix.xml -numFrames 2000

./build/bin/steersim -ai sfAI -testcase ../testcases/bottleneck-evac2.xml -config ./build/config_win-pleopt-room.xml -numFrames 2000
