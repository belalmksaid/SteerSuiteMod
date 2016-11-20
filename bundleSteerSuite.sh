#!/bin/bash

#
# Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
# See license.txt for complete license
#

# this sciprt is designed to package a steersuite for release.


# example usage
# ./tarSteerSuiteExec.sh linux
os="linux"
tarOptions="--exclude='.svn/' --exclude-vcs"

if [ "$os" == "linux" ]
then
	tarName="steersuite.tar"
	rm $tarName # if old files
#	tar $tarOptions -cvf $tarName ./build/win32/
	tar $tarOptions -cvf $tarName ./build/build.sh
	tar $tarOptions -rvf $tarName ./build/buildall
	tar $tarOptions -rvf $tarName ./build/clean.sh
	tar $tarOptions -rvf $tarName ./build/cleanall
	tar $tarOptions -rvf $tarName ./build/premake4.lua
	tar $tarOptions -rvf $tarName ./build/generate_visual_studio.bat
	tar $tarOptions -rvf $tarName ./build/config_unix.xml
	tar $tarOptions -rvf $tarName ./build/config_win.xml
	# concatenate other files/folders
	tar $tarOptions -rvf $tarName ./steerlib/
	tar $tarOptions -rvf $tarName ./socialForcesAI/
#	tar $tarOptions -rvf $tarName ./ccAI/
	tar $tarOptions -rvf $tarName ./documentation/
# 	tar $tarOptions -rvf $tarName ./external/AntTweakBar
	tar $tarOptions -rvf $tarName ./external/recastnavigation
	tar $tarOptions -rvf $tarName ./external/glfw
	tar $tarOptions -rvf $tarName ./external/mersenne
	tar $tarOptions -rvf $tarName ./external/tinyxml
#	tar $tarOptions -rvf $tarName ./FrameSaver/
	tar $tarOptions -rvf $tarName ./pprAI/
	tar $tarOptions -rvf $tarName ./reactiveAI
	tar $tarOptions -rvf $tarName ./RecFileIO
	tar $tarOptions -rvf $tarName ./rvo2AI
	tar $tarOptions -rvf $tarName ./kdtree
	tar $tarOptions -rvf $tarName ./navmeshBuilder
#	tar $tarOptions -rvf $tarName ./scenario
	tar $tarOptions -rvf $tarName ./simpleAI
	tar $tarOptions -rvf $tarName ./steerbench
#	tar $tarOptions -rvf $tarName ./steergen
#	tar $tarOptions -rvf $tarName ./steermine
	tar $tarOptions -rvf $tarName ./simpleAI
	tar $tarOptions -rvf $tarName ./steersim
	tar $tarOptions -rvf $tarName ./steersimlib
	tar $tarOptions -rvf $tarName ./steertool
#	tar $tarOptions -rvf $tarName ./steertrain
	tar $tarOptions -rvf $tarName ./testcases
	tar $tarOptions -rvf $tarName ./util
	tar $tarOptions -rvf $tarName ./simpleAI
#	tar $tarOptions -rvf $tarName ./roomEvacDemo.sh
#	tar $tarOptions -rvf $tarName ./roomEvacDemo.bat
#	tar $tarOptions -rvf $tarName ./buildEvacDemo.bat
#	tar $tarOptions -rvf $tarName ./buildEvacDemo.sh
#	tar $tarOptions -rvf $tarName ./paramBlendingDemo.sh
#	tar $tarOptions -rvf $tarName ./paramBlendingDemo.bat
#	tar $tarOptions -rvf $tarName ./parameterDemo.sh
#	tar $tarOptions -rvf $tarName ./parameterDemo.bat
	tar $tarOptions -rvf $tarName ./README.md
#	tar $tarOptions -rvf $tarName ./roomEvacDemo.bat
#	tar $tarOptions -rvf $tarName ./roomEvacDemo.sh
#	tar $tarOptions -rvf $tarName ./ParamOpt_README.pdf
	tar $tarOptions -rvf $tarName ./license.txt

	# tar $tarOptions -rvf $tarName paramBlendingDemo.sh
	gzip -f --best $tarName
else
	echo "Please specify os...."
	echo "linux/osx/win"
	exit

fi
