//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#include "SteerLib.h"
#include "datarec/DataRecIO.h"

using namespace std;

DataRecWriter::DataRecWriter(size_t numAgents, size_t numObstacles)
{
	_numAgents = numAgents;
	_numObstacles = numObstacles;
	_agentSequences = new StepDataSequence[numAgents];
	_obstacles = new ObstacleData[numObstacles];
}

DataRecWriter::~DataRecWriter()
{
	delete [] _agentSequences;
	delete [] _obstacles;
}

void DataRecWriter::writeToFile(const string& filename)
{
	if (Util::isExistingFile(filename)) {
		throw Util::GenericException("FootRecWriter::writeToFile(): file \"" + filename + "\" already exists.");
	}

	std::ofstream out;
	out.open(filename.c_str(), ios::binary);
	if (!out.is_open()) {
		throw Util::GenericException("FootRecWriter::writeToFile(): could not open file \"" + filename + "\" for writing.");
	}

	FootRecHeader header;
	header.format = SteerLib::DATA_REC;
	header.numAgents = _numAgents;
	header.numObstacles = _numObstacles;
	header.obstacleDataOffset = sizeof(header);
	header.agentTableOffset = _numObstacles * sizeof(ObstacleData) + sizeof(header);
	out.write((char*)&header, sizeof(header));

	for(unsigned int i = 0; i < _numObstacles; i++) {
		out.write((char*)&_obstacles[i], sizeof(ObstacleData));
	}

	AgentFootHeader agentHeader;
	unsigned int offsetOfThisAgentFootsteps = sizeof(header) + _numObstacles * sizeof(ObstacleData) + _numAgents * sizeof(AgentFootHeader);

	for (unsigned int i=0; i<_numAgents; i++) {
		if (i>0) {
			offsetOfThisAgentFootsteps += _agentSequences[i-1].size() * sizeof(StepData);
		}
		agentHeader.footstepTableOffset = offsetOfThisAgentFootsteps;
		agentHeader.numFootsteps = _agentSequences[i].size();
		out.write((char*)&agentHeader, sizeof(agentHeader));
	}

	unsigned int tellp = out.tellp();
	unsigned int s = sizeof(header) + _numObstacles * sizeof(ObstacleData) + _numAgents * sizeof(AgentFootHeader);
	assert( tellp == s);

	for (unsigned int i=0; i<_numAgents; i++) {
		for (unsigned int step = 0; step < _agentSequences[i].size(); step++) {
			out.write((char*)&(_agentSequences[i][step]), sizeof(StepData));
		}
	}

	out.close();

}

void DataRecWriter::addFootstep(unsigned int agentIndex, const StepData & s)
{
	_agentSequences[agentIndex].push_back(s);
}

void DataRecWriter::addFootstep(unsigned int agentIndex, const Footstep & footstep, unsigned int lookatID)
{
	StepData s;
	s.lookAtObject = lookatID;
	s.step = footstep;
	_agentSequences[agentIndex].push_back(s);
}

void DataRecWriter::addObstacleInfo(unsigned int obstacleIndex, float xmin, float xmax, float zmin, float zmax) {
	_obstacles[obstacleIndex].xmin = xmin;
	_obstacles[obstacleIndex].xmax = xmax;
	_obstacles[obstacleIndex].zmin = zmin;
	_obstacles[obstacleIndex].zmax = zmax;
}

DataRecReader::DataRecReader(const std::string & filename)
{
	_fileMap.open( filename );
	_header = (FootRecHeader*)_fileMap.getBasePointer();

	switch(_header->format) {
	case SteerLib::DATA_REC:
		{
			break;	//just pass through
		}
	case SteerLib::FOOT_REC:
		{
			throw new Util::GenericException("The version flag for " + filename + " suggests it is a footstepAI rec file, not dataAI.");
		}
	case SteerLib::SHADOW_REC:
		{
			throw new Util::GenericException("The version flag for " + filename + " suggests it is a shadowAI rec file, not dataAI.");
		}
	case SteerLib::STD_REC:
		{
			throw new Util::GenericException("The version flag for " + filename + " suggests it is a standard rec file, not dataAI.");
		}
	//format and open error values are for backwards compatibility with the original rec file format,
	//there's no reason to suspect they'd show up here, more likely to get junk values from old rec files.
	case SteerLib::FORMAT_ERROR:
	case SteerLib::OPEN_ERROR:
	default:
		{
			throw new Util::GenericException("The version flag in rec file " + filename + " is not for any recognized format.");
		}
	}
	
	_obstacles = (ObstacleData*)_fileMap.getPointerAtOffset(_header->obstacleDataOffset);

	_agents = (AgentFootHeader*)_fileMap.getPointerAtOffset(_header->agentTableOffset);

	_steps = new StepData*[ _header->numAgents ];

	char* base = (char*)_fileMap.getBasePointer();
	for (unsigned int i=0; i<_header->numAgents; i++) {
		_steps[i] = (StepData*)(base + _agents[i].footstepTableOffset);
	}

}

DataRecReader::~DataRecReader()
{
	if (_fileMap.isOpen()) _fileMap.close();
	if (_steps != NULL) delete [] _steps;
}

unsigned int DataRecReader::getNumAgents()
{
	return _header->numAgents;
}

unsigned int DataRecReader::getNumObstacles() {
	return _header->numObstacles;
}

ObstacleData& DataRecReader::getObstacle(unsigned int obstacleIndex) {
	return _obstacles[obstacleIndex];
}

unsigned int DataRecReader::getNumStepsForAgent(unsigned int agentIndex)
{
	return _agents[agentIndex].numFootsteps;
}

StepData & DataRecReader::getStepData(unsigned int agentIndex, unsigned int stepNumber)
{
	return _steps[agentIndex][stepNumber];
}