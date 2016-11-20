Put FrameSaver.h in steerlib\include\util and add to the project and put FrameSaver.cpp in steerlib\src and add to project

To use:
Include: #include "util/FrameSaver.h"
Initialize object: 
Util::FrameSaver framesaver;	
framesaver.StartRecord(1024);// resolution
Call the function 'framesaver.DumpPPM(1024, 768);' before the frame you have to dump
typically in the 'preprocessFrame' function

Use IrfanView or any other to view the ppm files 
you need to have mpeg_encode.exe and a .param file eg default.param in the same location as the ppm files
then run the command mpeg_encode.exe default.param. This param file has all the movie options