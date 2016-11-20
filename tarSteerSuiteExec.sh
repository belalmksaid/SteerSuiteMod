#!/bin/bash
#
# Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
# See license.txt for complete license.
#
# example usage
# ./tarSteerSuiteExec.sh linux
os=$1

if [ "$os" == "linux" ]
then
	tarName="paramOpt_Linux.tar"
	rm $tarName
	tar -cvf $tarName ../crowd-analysis/build/bin
	# concatenate other files/folders
	tar -rvf $tarName ../crowd-analysis/build/lib
	tar -rvf $tarName ../crowd-analysis/build/modules
	tar -rvf $tarName ../crowd-analysis/ParamOpt_README.pdf
	tar -rvf $tarName ../crowd-analysis/build/config_unix.xml
	tar -rvf $tarName ../crowd-analysis/build/config_unix-timeopt.xml
	tar -rvf $tarName ../crowd-analysis/build/config_unix-timeopt-circle.xml
	tar -rvf $tarName ../crowd-analysis/build/config_unix-pleopt-room.xml
	tar -rvf $tarName ../crowd-analysis/testcases
	tar -rvf $tarName ../crowd-analysis/parameterDemo.sh
	tar -rvf $tarName ../crowd-analysis/circleDemo.sh
	tar -rvf $tarName ../crowd-analysis/buildEvacDemo.sh
	tar -rvf $tarName ../crowd-analysis/roomEvacDemo.sh
	# tar -rvf $tarName paramBlendingDemo.sh
	gzip -f --best $tarName

elif [ "$os" == "osx" ]
then

	tarName="paramOpt_OSX.tar"
	rm $tarName
	tar -cvf $tarName ../crowd-analysis/build/bin
	# concatenate other files/folders
	tar -rvf $tarName ../crowd-analysis/build/lib
	tar -rvf $tarName ../crowd-analysis/build/modules
	tar -rvf $tarName ../crowd-analysis/ParamOpt_README.pdf
	tar -rvf $tarName ../crowd-analysis/build/config_unix.xml
	tar -rvf $tarName ../crowd-analysis/build/config_unix-timeopt.xml
	tar -rvf $tarName ../crowd-analysis/build/config_unix-timeopt-circle.xml
	tar -rvf $tarName ../crowd-analysis/build/config_unix-pleopt-room.xml
	tar -rvf $tarName ../crowd-analysis/testcases
	tar -rvf $tarName ../crowd-analysis/parameterDemo.sh
	tar -rvf $tarName ../crowd-analysis/circleDemo.sh
	tar -rvf $tarName ../crowd-analysis/buildEvacDemo.sh
	tar -rvf $tarName ../crowd-analysis/roomEvacDemo.sh
	# tar -rvf $tarName paramBlendingDemo.sh
	gzip -f --best $tarName
	

elif [ "$os" == "win" ]
then
# Windows..... 


	tarName="paramOpt_Win.tar"
	rm $tarName
	tar -cvf $tarName ../crowd-analysis/build/win32/Release
	# concatenate other files/folders
	tar -rvf $tarName ../crowd-analysis/build/config_win.xml
	tar -rvf $tarName ../crowd-analysis/ParamOpt_README.pdf
	tar -rvf $tarName ../crowd-analysis/build/config_win-timeopt.xml
	tar -rvf $tarName ../crowd-analysis/build/config_win-timeopt-circle.xml
	tar -rvf $tarName ../crowd-analysis/build/config_win-pleopt-room.xml
	tar -rvf $tarName ../crowd-analysis/testcases
	tar -rvf $tarName ../crowd-analysis/parameterDemo.bat
	tar -rvf $tarName ../crowd-analysis/circleDemo.bat
	tar -rvf $tarName ../crowd-analysis/roomEvacDemo.bat
	# tar -rvf $tarName paramBlendingDemo.bat
	tar -rvf $tarName ../crowd-analysis/buildEvacDemo.bat
	gzip -f --best $tarName
	
else
	echo "Please specify os...."
	echo "linux/osx/win"
	exit

fi
