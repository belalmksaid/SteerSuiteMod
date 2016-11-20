#!/bin/bash

# swig -Wall -v -debug-classes -c++ -csharp -dllimport "D:\\\playground\\\SteerSuite\\\build\\\bin\\\steerSuiteAdapter.dll" -o $1steerSuiteAdapter.cpp $1steerSuiteAdapter.swig
swig -Wall -v -debug-classes -c++ -csharp -o $1steerSuiteAdapter.cpp $1steerSuiteAdapter.swig
# swig3.0 -Wall -v -debug-classes -c++ -csharp -o $1steerSuiteAdapter.cpp $1steerSuiteAdapter.swig
