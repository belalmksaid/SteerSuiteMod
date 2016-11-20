//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//


#include "SteerLib.h"
#include "footrec/FootRecIO.h"
#include "footstepAI/FootstepAgent.h"

using namespace std;


FootRecWriter::FootRecWriter(size_t numAgents, size_t numObstacles)
{
	_numAgents = numAgents;
	_numObstacles = numObstacles;
	_agentSequences = new StepDataSequence[numAgents];
	_obstacles = new ObstacleData[numObstacles];
}

FootRecWriter::~FootRecWriter()
{
	delete [] _agentSequences;
	delete [] _obstacles;
}


void FootRecWriter::writeToFile(const std::string & filename)
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
	header.format = SteerLib::FOOT_REC;
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

void FootRecWriter::addFootstep(unsigned int agentIndex, const StepData & s)
{
	_agentSequences[agentIndex].push_back(s);
}

void FootRecWriter::addFootstep(unsigned int agentIndex, const Footstep & footstep, unsigned int lookatID)
{
	StepData s;
	s.lookAtObject = lookatID;
	s.step = footstep;
	_agentSequences[agentIndex].push_back(s);
}

void FootRecWriter::addObstacleInfo(unsigned int obstacleIndex, float xmin, float xmax, float zmin, float zmax) {
	_obstacles[obstacleIndex].xmin = xmin;
	_obstacles[obstacleIndex].xmax = xmax;
	_obstacles[obstacleIndex].zmin = zmin;
	_obstacles[obstacleIndex].zmax = zmax;
}

struct STPStep
{
	double start;
	double length;
};

void FootRecWriter::writeToStepFile(const std::string & filename)
{
	const double timeScale = 2.0;
	const double posScale = 1.0;

	/*if (Util::isExistingFile(filename)) {
		throw Util::GenericException("FootRecWriter::writeToStepFile(): file \"" + filename + "\" already exists.");
	}*/

	std::ofstream out;
	out.open(filename);
	if (!out.is_open()) {
		throw Util::GenericException("FootRecWriter::writeToStepFile(): could not open file \"" + filename + "\" for writing.");
	}

	out << "VERSION\n54\nSTEPS\n";

	out << fixed;
	out.precision(6);

	unsigned int numRleg = 0;
	unsigned int numLleg = 0;

	// TODO: Support multiple agents; write a MaxScript to load a folder of STP files possibly?
	assert(_numAgents > 0);

	out << _agentSequences[0].size() << "\n";

	for (int i = 0; i < _agentSequences[0].size(); i++)
	{
		if (_agentSequences[0][i].step.whichFoot == LEFT_FOOT)
			numLleg++;
		else
			numRleg++;

		//double orientation = _agentSequences[0][i].step.parabolaOrientationPhi;

		double orientation;

		if (i == 0)
			orientation = -FootstepAgent::determineFootstepOrientation(_agentSequences[0][i].step, _agentSequences[0][i].step);
		else
			orientation = -FootstepAgent::determineFootstepOrientation(_agentSequences[0][i].step, _agentSequences[0][i-1].step);

		out << cos(orientation) << " " << sin(orientation) << " " << 0.0 << "\n";
		out << -sin(orientation) << " " << cos(orientation) << " " << 0.0 << "\n";
		out << 0.0 << " " << 0.0 << " " << 1.0 << "\n";

		out << _agentSequences[0][i].step.footX * posScale << " " << -_agentSequences[0][i].step.footZ * posScale << " " << 0.0 << "\n";
	}

	out << "SCORE\n";

	std::vector<STPStep> steps;

	double offset = 0.0;

	double newOffset = 0.0;
	int newOffsetPos = -1;

	double addToLast = 0.0;

	for (int i = 0; i < _agentSequences[0].size(); i++)
	{
		STPStep newStep;

		if (_agentSequences[0][i].step.state == FOOTSTEP_STATE_HOPPING)
		{
			//out << int((_agentSequences[0][i].step.endTime - _agentSequences[0][i].step.startTime) * 500.0 * timeScale) << "\n";
			//out << int(_agentSequences[0][i].step.endTime * 1000.0 * timeScale) << "\n";
			newStep.start = (_agentSequences[0][i].step.startTime + (_agentSequences[0][i].step.endTime - _agentSequences[0][i].step.startTime)*0.5) * 1000.0;
		}
		else
		{
			newStep.start = _agentSequences[0][i].step.startTime * 1000.0;
		}

		if (i == newOffsetPos)
		{
			offset += newOffset;
			newOffset = 0;
			std::cout << "newoffsetPos added at " << i << std::endl;
		}
	
		newStep.start += offset;

		int j;
		for (j = i+1; j < _agentSequences[0].size() && _agentSequences[0][j].step.whichFoot != _agentSequences[0][i].step.whichFoot; j++);

		if (j < _agentSequences[0].size()-1 && _agentSequences[0][i+1].step.state == FOOTSTEP_STATE_HOPPING)
			newStep.length = (_agentSequences[0][i+1].step.startTime - _agentSequences[0][i].step.startTime) * 1000.0;
		else if (i == 0)
			newStep.length = (_agentSequences[0][i].step.endTime - _agentSequences[0][i].step.startTime) * 1200.0;
		else if (j < _agentSequences[0].size() - 1)
			//out << int((_agentSequences[0][i+1].step.startTime - _agentSequences[0][i].step.startTime)  * 1000.0) << "\n";
			newStep.length = (_agentSequences[0][i+1].step.startTime - _agentSequences[0][i].step.startTime) * 1200.0;
		else
			newStep.length = (_agentSequences[0][i].step.endTime - _agentSequences[0][i].step.startTime)  * 1200.0;

		if (j-i > 2)
		{
			std::cout << "J-I IS DIFFERENT" << std::endl;
			newStep.length *= 2;
			newOffset += newStep.length/2;
			newOffsetPos = j;
			addToLast = newStep.length/2;
		}

		if (_agentSequences[0][i].step.state == FOOTSTEP_STATE_HOPPING)
			newStep.length *= 0.5;

		if (i < _agentSequences[0].size()-1 && _agentSequences[0][i].step.whichFoot == _agentSequences[0][i+1].step.whichFoot)
		{
			offset += newStep.length;
			std::cout << "length added to offset added at " << i << std::endl;
		}

		// can only have two starting steps in a row currently
		if (i > 0 && _agentSequences[0][i].step.whichFoot == _agentSequences[0][i-1].step.whichFoot)
		{
			newStep.length += addToLast;
		}

		steps.push_back(newStep);

		/*for (j = i+1; j < _agentSequences[0].size() && _agentSequences[0][j].step.whichFoot != (bool)whichFoot; j++);

		if (j < _agentSequences[0].size()-1 && _agentSequences[0][i+1].step.state == FOOTSTEP_STATE_HOPPING)
			length = (_agentSequences[0][j-1].step.startTime - _agentSequences[0][i].step.startTime) * 1000.0;
		else if (i == 0)
			length = (_agentSequences[0][i].step.endTime - _agentSequences[0][i].step.startTime) * 1200.0;
		else if (j < _agentSequences[0].size() - 1)
			//out << int((_agentSequences[0][i+1].step.startTime - _agentSequences[0][i].step.startTime)  * 1000.0) << "\n";
			length = (_agentSequences[0][j-1].step.startTime - _agentSequences[0][i].step.startTime) * 1200.0;
		else
			length = (_agentSequences[0][i].step.endTime - _agentSequences[0][i].step.startTime)  * 1200.0;

		if (j-i > 1)
			length *= 2;

		if (_agentSequences[0][i].step.state == FOOTSTEP_STATE_HOPPING)
			length *= 0.5;*/


	}

		// exploits the fact that RIGHT_FOOT = 1 and LEFT_FOOT = 0
	for (int whichFoot = 1; whichFoot >= 0; whichFoot--)
	{
		if ((bool)whichFoot == RIGHT_FOOT)
			out << "RLEG\n" << numRleg << "\n";
		else
			out << "LLEG\n" << numLleg << "\n";

		for (int i = 0, step = 0; i < steps.size(); i++)
		{
			if (_agentSequences[0][i].step.whichFoot == (bool)whichFoot)
			{
				out << "step" << step << "\n";
				/*out << i * 2400 << "\n"; // time of step start?
				out << 2880 << "\n"; // length of step?*/

				// start time of step
				//std::cout << "STEP: " << i <<" " << _agentSequences[0][i].step.state << std::endl;
				// 
				out << int(steps[i].start * timeScale) << "\n";
				out << int(steps[i].length * timeScale) << "\n";
				out << i << "\n";

				step++;
			}
		}
	}

	out << "SCALE\n"
		<< 0.0 << "\n"			// ? 
		<< 10.0 << "\n"			// something to do with hopping/jumping/running height? lower -> higher jump
		<< 1.0 << "\n"			// seems to scale stride length
		<< DEFAULT_BASE_RADIUS << "\n"		// distance from center of biped to foot? i.e., walking parameter radius - PASS THE AGENT'S ACTUAL RADIUS TO THIS SOMEHOW!
		<< 100.0 << "\n";		// also something to do with hopping/jumping height

	// it appears that the second and last values are correlated - first*second = (a*first)*(a*second)

	out << "KEYFF\n";

	for (int i = 0; i < _agentSequences[0].size()*2; i++)
	{
		out << 0 << "\n";
	}

	// sometimes this is 0 - why???
	out << 1 << "\n";

	out << 206 << "\n" << 160 << "\n" << "END\n";

	out.close();
}

FootRecReader::FootRecReader(const std::string & filename)
{
	_fileMap.open( filename );
	_header = (FootRecHeader*)_fileMap.getBasePointer();
	switch(_header->format) {
	case SteerLib::FOOT_REC:
		{
			break;	//just pass through
		}
	case SteerLib::DATA_REC:
		{
			throw new Util::GenericException("The version flag for " + filename + " suggests it is a dataAI rec file, not footstepAI.");
		}
	case SteerLib::SHADOW_REC:
		{
			throw new Util::GenericException("The version flag for " + filename + " suggests it is a shadowAI rec file, not footstepAI.");
		}
	case SteerLib::STD_REC:
		{
			throw new Util::GenericException("The version flag for " + filename + " suggests it is a standard rec file, not footstepAI.");
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

	/*
	_obstacles = new ObstacleData[_header->numObstacles];
	char* base = (char*)_fileMap.getBasePointer();
	for(unsigned int i = 0; i < _header->numObstacles; i++) {
		_obstacles[i] = *(ObstacleData*)(base + i * sizeof(ObstacleData));
	}
	*/
	
	_obstacles = (ObstacleData*)_fileMap.getPointerAtOffset(_header->obstacleDataOffset);

	_agents = (AgentFootHeader*)_fileMap.getPointerAtOffset(_header->agentTableOffset);

	_steps = new StepData*[ _header->numAgents ];

	char* base = (char*)_fileMap.getBasePointer();
	for (unsigned int i=0; i<_header->numAgents; i++) {
		_steps[i] = (StepData*)(base + _agents[i].footstepTableOffset);
	}

}

FootRecReader::~FootRecReader()
{
	if (_fileMap.isOpen()) _fileMap.close();
	if (_steps != NULL) delete [] _steps;
}

unsigned int FootRecReader::getNumAgents()
{
	return _header->numAgents;
}

unsigned int FootRecReader::getNumObstacles()
{
	return _header->numObstacles;
}

ObstacleData& FootRecReader::getObstacle(unsigned int obstacleIndex) {
	return _obstacles[obstacleIndex];
}

unsigned int FootRecReader::getNumStepsForAgent(unsigned int agentIndex)
{
	return _agents[agentIndex].numFootsteps;
}

StepData & FootRecReader::getStepData(unsigned int agentIndex, unsigned int stepNumber)
{
	return _steps[agentIndex][stepNumber];
}
