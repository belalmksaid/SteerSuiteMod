//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


/// @file steersim/src/Main.cpp
/// @brief Entry point of SteerSim.

#include <exception>
#include "SteerLib.h"
#include "core/CommandLineEngineDriver.h"
#include "core/QtEngineDriver.h"
#include "SimulationPlugin.h"
#include "core/SteerSim.h"

#include "AntTweakBar/include/AntTweakBar.h"

#include <iomanip>
#include <cctype>
#include <algorithm>
#include "../include/core/GLFWEngineDriverACCLMesh.h"


using namespace std;
using namespace SteerLib;
using namespace Util;

const char * steersuite_init(int argc, char ** argv);
LogData * init_steersuite(int argc, char ** argv);



int main(int argc, char **argv)
{
	// save the original cerr streambuf, so that we can restore it on an exception.
	std::streambuf * cerrOriginalStreambuf = std::cerr.rdbuf();

	std::ofstream coutRedirection;
	std::ofstream cerrRedirection;
	std::ofstream clogRedirection;

	try {

		SimulationOptions simulationOptions;
		initializeOptionsFromCommandLine( argc, argv, simulationOptions );


		//
		// after initializing from command line,
		// the simulationOptions data structure is properly initialized
		// and is the only source of options from now on.
		//


		// re-direct cout, cerr, and clog, if the user specified it through options.
		if (simulationOptions.globalOptions.coutRedirectionFilename != "") {
			coutRedirection.open(simulationOptions.globalOptions.coutRedirectionFilename.c_str());
			std::cout.rdbuf( coutRedirection.rdbuf() );
		}
		if (simulationOptions.globalOptions.cerrRedirectionFilename != "") {
			cerrRedirection.open(simulationOptions.globalOptions.cerrRedirectionFilename.c_str());
			std::cerr.rdbuf( cerrRedirection.rdbuf() );
		}
		if (simulationOptions.globalOptions.clogRedirectionFilename != "") {
			clogRedirection.open(simulationOptions.globalOptions.clogRedirectionFilename.c_str());
			std::clog.rdbuf( clogRedirection.rdbuf() );
		}

		//
		// allocate and use the engine driver
		//
		if (simulationOptions.globalOptions.engineDriver == "commandline") {
			CommandLineEngineDriver * cmd = new CommandLineEngineDriver();
			cmd->init(&simulationOptions);
			cmd->run();
			cmd->finish();
		}
		else if (simulationOptions.globalOptions.engineDriver == "glfw") {
#ifdef ENABLE_GUI
#ifdef ENABLE_GLFW
			GLFWEngineDriver * driver = GLFWEngineDriver::getInstance();
			CommandLineEngineDriver * cmd = new CommandLineEngineDriver();

			cmd->init(&simulationOptions);
			driver->init(&simulationOptions);

			driver->run();
			driver->finish();
			cmd->run();

			cmd->finish();
#else
			throw GenericException("GLFW functionality is not compiled into this version of SteerSim.");
#endif
#else
			throw GenericException("GUI functionality is not compiled into this version of SteerSim. Use the -commandline option.");
#endif // ifdef ENABLE_GUI
		}
		else if (simulationOptions.globalOptions.engineDriver == "qt") {
#ifdef ENABLE_GUI
#ifdef ENABLE_QT
			SteerSimQt::QtEngineDriver * driver = SteerSimQt::QtEngineDriver::getInstance();
			driver->init(options);
			driver->run();
			driver->finish();
#else
			throw GenericException("Qt functionality is not compiled into this version of SteerSim.");
#endif
#else
			throw GenericException("GUI functionality is not compiled into this version of SteerSim. Use the -commandline option.");
#endif
		}
	}
	catch (std::exception &e) {

		std::cerr << "\nERROR: exception caught in main:\n" << e.what() << "\n";

		// there is a chance that cerr was re-directed.  If this is true, then also echo 
		// the error to the original cerr.
		if (std::cerr.rdbuf() != cerrOriginalStreambuf) {
			std::cerr.rdbuf(cerrOriginalStreambuf);
			std::cerr << "\nERROR: exception caught in main:\n" << e.what() << "\n";
		}

		if (coutRedirection.is_open()) coutRedirection.close();
		if (cerrRedirection.is_open()) cerrRedirection.close();
		if (clogRedirection.is_open()) clogRedirection.close();

		return EXIT_FAILURE;
	}

	if (coutRedirection.is_open()) coutRedirection.close();
	if (cerrRedirection.is_open()) cerrRedirection.close();
	if (clogRedirection.is_open()) clogRedirection.close();

	return EXIT_SUCCESS;
}

/*
 * A duplicate of the main function. Created for calling from Python
 */
const char * steersuite_init(int argc, char ** argv)
{
	// save the original cerr streambuf, so that we can restore it on an exception.
	std::streambuf * cerrOriginalStreambuf = std::cerr.rdbuf();

	const char * outData=NULL;

	std::ofstream coutRedirection;
	std::ofstream cerrRedirection;
	std::ofstream clogRedirection;

	try {
		SimulationOptions simulationOptions;
		initializeOptionsFromCommandLine( argc, argv, simulationOptions );

		//
		// after initializing from command line,
		// the simulationOptions data structure is properly initialized
		// and is the only source of options from now on.
		//


		// re-direct cout, cerr, and clog, if the user specified it through options.
		if (simulationOptions.globalOptions.coutRedirectionFilename != "") {
			coutRedirection.open(simulationOptions.globalOptions.coutRedirectionFilename.c_str());
			std::cout.rdbuf( coutRedirection.rdbuf() );
		}
		if (simulationOptions.globalOptions.cerrRedirectionFilename != "") {
			cerrRedirection.open(simulationOptions.globalOptions.cerrRedirectionFilename.c_str());
			std::cerr.rdbuf( cerrRedirection.rdbuf() );
		}
		if (simulationOptions.globalOptions.clogRedirectionFilename != "") {
			clogRedirection.open(simulationOptions.globalOptions.clogRedirectionFilename.c_str());
			std::clog.rdbuf( clogRedirection.rdbuf() );
		}
		//
		// allocate and use the engine driver
		//
		if (simulationOptions.globalOptions.engineDriver == "commandline") {
			std::cout << "Using command line engine Driver." << std::endl;
			CommandLineEngineDriver * cmd = new CommandLineEngineDriver();
			cmd->init(&simulationOptions);
			std::cout << "Running command line engine Driver." << std::endl;
			cmd->run();
			std::cout << "Finishing command line engine Driver." << std::endl;
			outData = cmd->getData();
			cmd->finish();
			std::cout << "finished command line engine driver" << std::endl;
		}
		else if (simulationOptions.globalOptions.engineDriver == "glfw") {
#ifdef ENABLE_GUI
#ifdef ENABLE_GLFW
			GLFWEngineDriver * driver = GLFWEngineDriver::getInstance();
			driver->init(&simulationOptions);
			driver->run();
			outData = driver->getData();
			driver->finish();
#else
			throw GenericException("GLFW functionality is not compiled into this version of SteerSim.");
#endif
#else
			throw GenericException("GUI functionality is not compiled into this version of SteerSim. Use the -commandline option.");
#endif // ifdef ENABLE_GUI
		}
		else if (simulationOptions.globalOptions.engineDriver == "qt") {
#ifdef ENABLE_GUI
#ifdef ENABLE_QT
			SteerSimQt::QtEngineDriver * driver = SteerSimQt::QtEngineDriver::getInstance();
			driver->init(options);
			driver->run();
			std::cout << "finished qt engine driver" << std::endl;
			driver->finish();
#else
			throw GenericException("Qt functionality is not compiled into this version of SteerSim.");
#endif
#else
			throw GenericException("GUI functionality is not compiled into this version of SteerSim. Use the -commandline option.");
#endif
		}
	}
	catch (std::exception &e) {

		std::cerr << "\nERROR: exception caught in main:\n" << e.what() << "\n";

		// there is a chance that cerr was re-directed.  If this is true, then also echo
		// the error to the original cerr.
		if (std::cerr.rdbuf() != cerrOriginalStreambuf) {
			std::cerr.rdbuf(cerrOriginalStreambuf);
			std::cerr << "\nERROR: exception caught in main:\n" << e.what() << "\n";
		}

		if (coutRedirection.is_open()) coutRedirection.close();
		if (cerrRedirection.is_open()) cerrRedirection.close();
		if (clogRedirection.is_open()) clogRedirection.close();
		char * out = (char *) malloc(sizeof(char)*20);
		strncpy(out, "EXIT_FAILURE", 20);
		return out;
	}

	if (coutRedirection.is_open()) coutRedirection.close();
	if (cerrRedirection.is_open()) cerrRedirection.close();
	if (clogRedirection.is_open()) clogRedirection.close();

	// This needs to be left available for the Python process to use this data
	return outData;
}

/*
 * A duplicate of the main function. Created for calling from Python
 */
LogData * init_steersuite(int argc, char ** argv)
{
	// save the original cerr streambuf, so that we can restore it on an exception.
	std::streambuf * cerrOriginalStreambuf = std::cerr.rdbuf();

	LogData * outData=NULL;

	std::ofstream coutRedirection;
	std::ofstream cerrRedirection;
	std::ofstream clogRedirection;

	try {
		SimulationOptions simulationOptions;
		initializeOptionsFromCommandLine( argc, argv, simulationOptions );

		//
		// after initializing from command line,
		// the simulationOptions data structure is properly initialized
		// and is the only source of options from now on.
		//


		// re-direct cout, cerr, and clog, if the user specified it through options.
		if (simulationOptions.globalOptions.coutRedirectionFilename != "") {
			coutRedirection.open(simulationOptions.globalOptions.coutRedirectionFilename.c_str());
			std::cout.rdbuf( coutRedirection.rdbuf() );
		}
		if (simulationOptions.globalOptions.cerrRedirectionFilename != "") {
			cerrRedirection.open(simulationOptions.globalOptions.cerrRedirectionFilename.c_str());
			std::cerr.rdbuf( cerrRedirection.rdbuf() );
		}
		if (simulationOptions.globalOptions.clogRedirectionFilename != "") {
			clogRedirection.open(simulationOptions.globalOptions.clogRedirectionFilename.c_str());
			std::clog.rdbuf( clogRedirection.rdbuf() );
		}
		//
		// allocate and use the engine driver
		//
		if (simulationOptions.globalOptions.engineDriver == "commandline") {
			std::cout << "Using command line engine Driver." << std::endl;
			CommandLineEngineDriver * cmd = new CommandLineEngineDriver();
			cmd->init(&simulationOptions);
			std::cout << "Running command line engine Driver." << std::endl;
			cmd->run();
			std::cout << "Finishing command line engine Driver." << std::endl;
			outData = cmd->getLogData();
			cmd->finish();
			std::cout << "finished command line engine driver" << std::endl;
		}
		else if (simulationOptions.globalOptions.engineDriver == "glfw") {
#ifdef ENABLE_GUI
#ifdef ENABLE_GLFW
			GLFWEngineDriver * driver = GLFWEngineDriver::getInstance();
			driver->init(&simulationOptions);
			driver->run();
			outData = driver->getLogData();
			driver->finish();
#else
			throw GenericException("GLFW functionality is not compiled into this version of SteerSim.");
#endif
#else
			throw GenericException("GUI functionality is not compiled into this version of SteerSim. Use the -commandline option.");
#endif // ifdef ENABLE_GUI
		}
		else if (simulationOptions.globalOptions.engineDriver == "qt") {
#ifdef ENABLE_GUI
#ifdef ENABLE_QT
			SteerSimQt::QtEngineDriver * driver = SteerSimQt::QtEngineDriver::getInstance();
			driver->init(options);
			driver->run();
			std::cout << "finished qt engine driver" << std::endl;
			driver->finish();
#else
			throw GenericException("Qt functionality is not compiled into this version of SteerSim.");
#endif
#else
			throw GenericException("GUI functionality is not compiled into this version of SteerSim. Use the -commandline option.");
#endif
		}
	}
	catch (std::exception &e) {

		std::cerr << "\nERROR: exception caught in main:\n" << e.what() << "\n";

		// there is a chance that cerr was re-directed.  If this is true, then also echo
		// the error to the original cerr.
		if (std::cerr.rdbuf() != cerrOriginalStreambuf) {
			std::cerr.rdbuf(cerrOriginalStreambuf);
			std::cerr << "\nERROR: exception caught in main:\n" << e.what() << "\n";
		}

		if (coutRedirection.is_open()) coutRedirection.close();
		if (cerrRedirection.is_open()) cerrRedirection.close();
		if (clogRedirection.is_open()) clogRedirection.close();
		return NULL;
	}

	if (coutRedirection.is_open()) coutRedirection.close();
	if (cerrRedirection.is_open()) cerrRedirection.close();
	if (clogRedirection.is_open()) clogRedirection.close();

	// This needs to be left available for the Python process to use this data
	return outData;
}


PLUGIN_API	const char * init_steersim(int argc, char ** argv)
{
	return steersuite_init(argc, argv);
}
PLUGIN_API	LogData * init_steersim2(int argc, char ** argv)
{
	return init_steersuite(argc, argv);
}

