//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include "GenEngine.h"

using namespace Util;
using namespace std;

GeneratorEngine::GeneratorEngine(StateConfig* config, unsigned int whichContext, const string& saveLocation, bool validationMode) {
	outFileRoot = saveLocation;
	contextNumber = whichContext;
	validation = validationMode;

	if(!validation) {
		//for now the .metaData file will hold a single integer, the number of contexts, assuming [0, n] contexts.
		//future expansion of the file can be used to avoid flooding the command line with more arguments, and allow flexibility with models/algorithms
		if(fileCanBeOpened(saveLocation + ".metaData")) {
			metaLog.first = false;
			ifstream metaReader(saveLocation + ".metaData");
			char buffer[100];
			metaReader.getline(buffer, 100);
			metaLog.second.push_back(buffer);

			//this bit of code may be moved elsewhere when there is more information in the file
			if(atoi(metaLog.second[0].c_str()) < whichContext) {
				stringstream converter;
				converter << whichContext;
				metaLog.second[0] = converter.str();
				metaLog.first = true;	//if metaLog.second has to be changed, that makes it dirty, simple enough
			}
		}
		else {
			stringstream converter;
			converter << whichContext;
			metaLog.second.push_back(converter.str());
			metaLog.first = true;
		}
	}
	else {
		//need to load in the .actions file so that the numbers match, otherwise you get scared by a 90% error rate -.-
		stringstream nameBuilder;
		nameBuilder << outFileRoot << "_" << contextNumber;
		ifstream leftSide(nameBuilder.str() + "l.actions");
		ifstream rightSide(nameBuilder.str() + "r.actions");
		do {
			char rawLine[100];
			leftSide.getline(rawLine, 100);
			stringstream tokenizer(rawLine, ios_base::in);
			int junk;

			stepInfo temp;
			tokenizer >> junk;
			tokenizer >> temp.phi;
			tokenizer >> temp.duration;
			tokenizer >> temp.desiredSpeed;
			tokenizer >> temp.footstepEnum;

			actionMap[0].push_back(temp);
		} while(leftSide.peek() != EOF);
		leftSide.close();

		do {
			char rawLine[100];
			rightSide.getline(rawLine, 100);
			stringstream tokenizer(rawLine, ios_base::in);
			int junk;

			stepInfo temp;
			tokenizer >> junk;
			tokenizer >> temp.phi;
			tokenizer >> temp.duration;
			tokenizer >> temp.desiredSpeed;
			tokenizer >> temp.footstepEnum;

			actionMap[1].push_back(temp);
		} while(rightSide.peek() != EOF);
		rightSide.close();
	}
	
	reader = 0;
	stateSpace = config;
	currentStep = 0;				//an external flag for now because we may want to introduce a function call for a single step being processed
	actionUsage[0].clear();			//just to be sure
	actionUsage[0].resize(30, 0);
	actionUsage[1].clear();
	actionUsage[1].resize(30, 0);
}

GeneratorEngine::~GeneratorEngine() {
	if(reader) {
		delete reader;
	}
	actionUsage[0].clear();
	actionUsage[1].clear();
	actionMap[0].clear();
	actionMap[1].clear();
}

void GeneratorEngine::loadRecFile(const string& filename) {
	if(reader) {
		delete reader;
	}
	reader = new FootRecReader(filename);
}

void GeneratorEngine::extractSamples() {
	if(reader == 0) {
		//decide on a programming paradigm separating runtime from coding errors?  perhaps use assert here?
		throw GenericException("Did not load a rec file before extraction with GeneratorEngine.");
	}
	unsigned int numSteps = reader->getNumStepsForAgent(0);	//assuming agent 0 is the subject agent of every simulation
	unsigned int numAgents = reader->getNumAgents();		//assuming this is constant for the whole simulation recording
	unsigned int numObstacles = reader->getNumObstacles();
	
	//each agent needs to be searched for the applicable footstep relative to the subject
	//store the last hit because the footstep monotonically increases
	unsigned int* lastSteps = new unsigned int[numAgents];
	for(unsigned int i = 0; i < numAgents; i++) {
		lastSteps[i] = 0;
	}
	
	//Assume that the subject has a single goal throughout this rec file, which Mubbasir says is the scenario module behavior

	//start generating data from footstep 1 because 0 is the initial condition of the agent
	for(unsigned int i = 1; i < numSteps; i++) {
		//get subject agent's data for this footstep for transformation of the other agents into local space and the corresponding action
		if(i == 1) {
			lastFrame.step = reader->getStepData(0, 0).step;
		}
		subject.step = reader->getStepData(0, i).step;

		subject.subjectGoal.x = subject.step.targetX;
		subject.subjectGoal.z = subject.step.targetZ;
		
		//the footstep decisions are made in discrete offsets of atan2f(dz, dx) of the previous step, so we want this to be 0 for conversion to local space
		subject.subjectPrevX = lastFrame.step.outputCOMState.x;
		subject.subjectPrevZ = lastFrame.step.outputCOMState.z;
		subject.theta = atan2f(lastFrame.step.outputCOMState.dz, lastFrame.step.outputCOMState.dx);
		
		//extract data for each obstacle
		vector<obstacleInfo> obstacles;
		for(unsigned int j = 0; j < numObstacles; j++) {
			obstacleInfo currObstacle;
			ObstacleData recObstacle = reader->getObstacle(j);
			currObstacle.xmax = recObstacle.xmax;
			currObstacle.xmin = recObstacle.xmin;
			currObstacle.zmax = recObstacle.zmax;
			currObstacle.zmin = recObstacle.zmin;

			//transform these min/max points into local space
			currObstacle.xmin -= subject.subjectPrevX;
			currObstacle.xmax -= subject.subjectPrevX;
			currObstacle.zmin -= subject.subjectPrevZ;
			currObstacle.zmax -= subject.subjectPrevZ;

			currObstacle.rayCorrectionAngle = subject.theta;

			obstacles.push_back(currObstacle);
		}
		//at this point all obstacles should have their data relative to the subject's local space stored in a vector

		//extract data for each agent (skipping agent 0, our subject)
		vector<agentInfo> agents;
		for(unsigned int j = 1; j < numAgents; j++) {
			
			//guarding against overflow since these are stored in unchecked arrays
			if(lastSteps[j] >= reader->getNumStepsForAgent(j)) {
				continue;
			}

			//agents are not generally on the same step, so need to find an overlapping range
			Footstep tempStep;
			Footstep prevStep;
			agentInfo tempInfo;
			unsigned int offset = 0;

			//find the applicable steps of this agent (startTime < subject.startTime and endTime >= subject.startTime)
			do {
				if(lastSteps[j] + offset == reader->getNumStepsForAgent(j)) {
					break;
				}
				tempStep = reader->getStepData(j, lastSteps[j] + offset).step;
				offset++;
			} while(!(tempStep.startTime <= subject.step.startTime && tempStep.endTime >= subject.step.startTime));

			lastSteps[j] += offset - 1;

			//sanity check, not sure if steersim drops agents from simulation asynchronously
			if(lastSteps[j] >= reader->getNumStepsForAgent(j)) {
				continue;
			}
			
			if(lastSteps[j] > 0) {
				prevStep = reader->getStepData(j, lastSteps[j] - 1).step;
			}
			else {	//apparently this is so early in the simulation that this agent is still standing
				prevStep = tempStep;
			}

			//linearly interpolate the COM between the two footsteps
			//calculate the weight to use for interpolation
			float weight = (subject.step.startTime - prevStep.endTime) / tempStep.endTime;
			tempInfo.COMx = prevStep.outputCOMState.x * weight + tempStep.outputCOMState.x * (1 - weight);
			tempInfo.COMz = prevStep.outputCOMState.z * weight + tempStep.outputCOMState.z * (1 - weight);

			//choose the velocity somehow, previous step's is probably best: 0th footstep velocity is 0
			if(lastSteps[j] > 0) {
				tempInfo.velx = prevStep.outputCOMState.dx;
				tempInfo.velz = prevStep.outputCOMState.dz;
			}
			else {
				tempInfo.velx = 0;	//Mubbasir said to assume that the first footstep has a velocity of 0
				tempInfo.velz = 0;
			}

			//transform coordinates and velocity into subject's local space WHEN THE DECISION IS MADE, so the previous step's COM data
			tempInfo.COMx -= subject.subjectPrevX;
			tempInfo.COMz -= subject.subjectPrevZ;

			//transform agent's position and velocity accordingly
			float rotX = tempInfo.COMx * cosf(-subject.theta) - tempInfo.COMz * sinf(-subject.theta);
			float rotZ = tempInfo.COMx * sinf(-subject.theta) + tempInfo.COMz * cosf(-subject.theta);

			tempInfo.COMx = rotX;
			tempInfo.COMz = rotZ;

			//reuse these variables to transform velocities as well
			rotX = tempInfo.velx * cosf(-subject.theta) - tempInfo.velz * sinf(-subject.theta);
			rotZ = tempInfo.velx * sinf(-subject.theta) + tempInfo.velz * cosf(-subject.theta);

			tempInfo.velx = rotX;
			tempInfo.velz = rotZ;

			//store the id, which is just j
			tempInfo.id = j;

			agents.push_back(tempInfo);
		}
		//at this point all the agents should have their data relative to the subject agent's local space stored

		extractSpecializedSamples(contextNumber, agents, obstacles);
		if(i < 5) {
			extractContextSamples(contextNumber, agents, obstacles);
		}

		lastFrame.step = subject.step;
	}
	delete [] lastSteps;
}

 void GeneratorEngine::printStats() const {
	if(reader == 0) {
		cerr << "Did not load a rec file first!" << endl;
		return;
	}
	if(actionMap[0].empty() && actionMap[1].empty()) {
		cerr << "Haven't extracted samples yet, no usage stats would be valid." << endl;
		return;
	}

	int leftSum = 0;
	for(int i = 0; i < actionMap[0].size(); i++) {
		leftSum += actionUsage[0][i];
	}
	if(!validation) {
		cout << "Left foot action-space usage for training context " << contextNumber << endl;
	}
	else {
		cout << "Left foot action-space usage for validating context " << contextNumber << endl;
	}
	for(int i = 0; i < actionMap[0].size(); i++) {
		cout << i << '\t' << static_cast<double>(actionUsage[0][i]) / static_cast<double>(leftSum) << endl;
	}

	int rightSum = 0;
	for(int i = 0; i < actionMap[1].size(); i++) {
		rightSum += actionUsage[1][i];
	}
	if(!validation) {
		cout << "Right foot action-space usage for training context " << contextNumber << endl;
	}
	else {
		cout << "Right foot action-space usage for validating context " << contextNumber << endl;
	}
	for(int i = 0; i < actionMap[1].size(); i++) {
		cout << i << '\t' << static_cast<double>(actionUsage[1][i]) / static_cast<double>(rightSum) << endl;
	}
}

void GeneratorEngine::shrinkage() {
	for (int foot = 0; foot <= 1; foot++){
		float sum = 0.0f;

		/* Jennie: there's a convention with for loop counters that they go i, j, k, etc. and they get reused when possible.
		   It'll make your code more intuitive (and your coding a tidbit faster because some variable names are on "auto pilot") because
		   it will be easier to know which loop layer you're in when you debugging nested loops.  I changed to loops below to reflect that.

		   I don't know that there's a convention for iterators, i don't normally have nested loops so i just name the variable "iter" or "citer"
		   depending on if it's const or not
		*/
		for (int i = 0; i < actionMap[foot].size(); i++) {
			sum += actionUsage[foot][i];
		}
		
		vector<bool> existing_enums (4, false);
		vector<pair<int, int>> change_to;

		for (int i = 0; i < actionMap[foot].size(); i++) {
			pair <int, int> current = make_pair (i, i);
			change_to.push_back(current);

			float percentage;
			percentage = static_cast<float>(actionUsage[foot][i]) / sum;
			
			if ((percentage < .001) && (existing_enums[actionMap[foot][i].footstepEnum])) {
				to_remove[foot].push_back(i);
			}
			else {
				existing_enums[actionMap[foot][i].footstepEnum] = true;
			}
		}
		for (int i = 0; i < to_remove[foot].size(); i++) {
			for (int j = 0; j < actionMap[foot].size(); j++) {
				if (j > to_remove[foot][i]){
					change_to[j].second -= 1;
				}
			}
			change_to[to_remove[foot][i]].second = -1;
		}
		for (int i = 0; i < samples[foot].size(); i++){
			if (i == samples[foot].size()) break;
			int string_size = samples[foot][i].size();
			if (samples[foot][i].at(string_size - 3) != ','){
				char current_type1 = (samples[foot][i].at(string_size - 3));
				char current_type2 = (samples[foot][i].at(string_size - 2));
				string current_type = toString(current_type1) + toString(current_type2);

				int position = atoi(current_type.c_str());

				if (change_to[position].second == -1){
					samples[foot].erase(samples[foot].begin()+i);
					i--;
				}
				else if (change_to[position].second != change_to[position].first){
					samples[foot][i].erase(string_size - 3, 3);
					samples[foot][i].append(toString(change_to[position].second));
					samples[foot][i].append(toString('\n'));
				}
			}

			else{
				string current_type = toString(samples[foot][i].at(string_size - 2));

				int position = atoi(current_type.c_str());

				if (change_to[position].second == -1){
					samples[foot].erase(samples[foot].begin()+i);
					i--;
				}
				else if (change_to[position].second != change_to[position].first){
					samples[foot][i].replace(string_size - 2, 1, (toString(change_to[position].second)));
				}
			}
		}
	}
}

void GeneratorEngine::saveSamples() {
	ofstream outl;
	ofstream outr;

	//add the attribute for the classes and indices of the action table as possible values
	stringstream specializedRoot;
	specializedRoot << outFileRoot << "_" << contextNumber;
	outl.open(specializedRoot.str() + "l.names");
	outr.open(specializedRoot.str() + "r.names");
	//attach a constant attribute name for classes we learn over
	outl << "Choices.\n";
	outr << "Choices.\n";

	//get the environmental features from StateConfig
	outl << stateSpace->toString(contextNumber);
	outr << stateSpace->toString(contextNumber);

	//add egocentric features
	/*outl << "Goal_Distance:\t" << "continuous.\n";
	outl << "Goal_Theta:\t" << "continuous.\n";
	outr << "Goal_Distance:\t" << "continuous.\n";
	outr << "Goal_Theta:\t" << "continuous.\n";
	//out << "Which_foot:\t" << "r,l.\n";
	outl << "Step_State:\t" << "0,1,2,3.\n";
	outl << "Choices:\t";
	outr << "Step_State:\t" << "0,1,2,3.\n";
	outr << "Choices:\t";*/
	outl << "Goal_Distance:\t" << "1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20.\n";
	outl << "Goal_Theta:\t" << "0,1,2,3,4,5,6,7,8,9,10,11,-1,-2,-3,-4,-5,-6,-7,-8,-9,-10,-11,12.\n";
	outr << "Goal_Distance:\t" << "1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20.\n";
	outr << "Goal_Theta:\t" << "0,1,2,3,4,5,6,7,8,9,10,11,-1,-2,-3,-4,-5,-6,-7,-8,-9,-10,-11,12.\n";
	//out << "Which_foot:\t" << "r,l.\n";
	outl << "Step_State:\t" << "0,1,2,3.\n";
	outl << "Choices:\t";
	outr << "Step_State:\t" << "0,1,2,3.\n";
	outr << "Choices:\t";

	for(unsigned int i = 0; i < (actionMap[0].size() - to_remove[0].size()); i++) {
		if(i == (actionMap[0].size() - to_remove[0].size()) - 1) {
			outl << i << ".\n";   
		}
		else {
			outl << i << ",";
		}
	}
	outl.close();

	for(unsigned int i = 0; i < (actionMap[1].size() - to_remove[1].size()); i++) {
		if(i == (actionMap[1].size()  - to_remove[1].size())- 1) {
			outr << i << ".\n";
		}
		else {
			outr << i << ",";
		}
	}
	outr.close();
	
	if(!validation) {
		//if the metaData file needed rewritten, we should rewrite this too
		ifstream nameChecker((outFileRoot + "_C.names").c_str());
		if(!nameChecker.is_open() || metaLog.first) {
			//Write the top-level classifier .names file if the context number is higher than we've seen
		
			outl.open(outFileRoot + "_C.names", ios::trunc);
			//attach a constant attribute name for classes we learn over
			outl << "Choices.\n";

			//get the environmental features from StateConfig
			outl << stateSpace->toString(-1);
			outl << "Choices:\t";
			for(unsigned int i = 0; i < contextNumber; i++) {
				outl << i << ",";
			}

			//maxContext is actually the highest LABEL, so we need to add its value at the end
			outl << contextNumber << ".\n";

			outl.close();
		}
	}

	//switch to the .data output files
	//first write the specialized classifier's data
	if(!validation) {
		outl.open(specializedRoot.str() + "l.data");
		outr.open(specializedRoot.str() + "r.data");
	}
	else {
		outl.open(specializedRoot.str() + "l.test");
		outr.open(specializedRoot.str() + "r.test");
	}

	for(vector<string>::const_iterator citer = samples[0].cbegin(); citer != samples[0].cend(); ++citer) {
		outl << *citer;
	}
	outl.close();

	for(vector<string>::const_iterator citer = samples[1].cbegin(); citer != samples[1].cend(); ++citer) {
		outr << *citer;
	}
	outr.close();

	if(!validation) {
		//then switch to the top-level classifier and write/append the data as needed
		if(fileCanBeOpened(outFileRoot + "_C.data")) {
			//set the file output to append at the end since we're adding new contexts
			outl.open(outFileRoot + "_C.data", ios::app);
		}
		else {
			//Otherwise we're creating a new data file, note that this is automatically true for specialized classifiers
			outl.open(outFileRoot + "_C.data");
		}
	}
	else {
		//then switch to the top-level classifier and write/append the data as needed
		if(fileCanBeOpened(outFileRoot + "_C.test")) {
			//set the file output to append at the end since we're adding new contexts
			outl.open(outFileRoot + "_C.test", ios::app);
		}
		else {
			//Otherwise we're creating a new data file, note that this is automatically true for specialized classifiers
			outl.open(outFileRoot + "_C.test");
		}
	}
	for(vector<string>::const_iterator citer = contextSamples.cbegin(); citer != contextSamples.cend(); ++citer) {
		outl << *citer;
	}

	outl.close();

	//switch to the .actions output file, which we've added for the action table of specialized classifiers
	outl.open(specializedRoot.str() + "l.actions");
	int j = 0;
	for(int i = 0; i < actionMap[0].size(); i++) {
		if(find(to_remove[0].begin(), to_remove[0].end(), i) == to_remove[0].end()) {
			outl << j++ << " " << actionMap[0][i].phi << " " << actionMap[0][i].duration << " " << actionMap[0][i].desiredSpeed << " " << actionMap[0][i].footstepEnum << endl;
		}
	}
	outl.close();

	j = 0;
	outr.open(specializedRoot.str() + "r.actions");
	for(int i = 0; i < actionMap[1].size(); i++) {
		if(find(to_remove[1].begin(), to_remove[1].end(), i) == to_remove[1].end()) {
			outr << j++ << " " << actionMap[1][i].phi << " " << actionMap[1][i].duration << " " << actionMap[1][i].desiredSpeed << " " << actionMap[1][i].footstepEnum << endl;
		}
	}
	outr.close();

	if(!validation) {
		//last, if the metaLog is dirty, overwrite the file with the line-by-line contents of the log
		if(metaLog.first) {
			outr.open(outFileRoot + ".metaData", ios::trunc);
			for(vector<string>::const_iterator citer = metaLog.second.cbegin(); citer != metaLog.second.cend(); ++citer) {
				outr << *citer << endl;
			}
			outr.close();
		}
	}
}

void GeneratorEngine::printRecFileContents() const {
	cout << "Agents:" << endl;
	for(unsigned int i = 0; i < reader->getNumAgents(); i++) {
		StepData curr = reader->getStepData(i, 0);
		cout << '\t' << i << ": (" << curr.step.outputCOMState.x << ", " << curr.step.outputCOMState.z << ") and velocity (" << curr.step.outputCOMState.dx << ", " << curr.step.outputCOMState.dz << ")\n";
	}

	cout << "Obstacles:" << endl;
	for(unsigned int i = 0; i < reader->getNumObstacles(); i++) {
		ObstacleData curr = reader->getObstacle(i);
		cout << '\t' << i << ": (" << curr.xmin << " to " << curr.xmax << ") on x and (" << curr.zmin << " to " << curr.zmax << ") on z\n";
	}
}

void GeneratorEngine::handleSlice(const StateConfig::slice wedge, vector<agentInfo>& agents, vector<obstacleInfo>& obstacles, stringstream& result) {
	float currNearest = numeric_limits<float>::infinity();
	float nearestSpeed;
	int nearestTheta;

	for(vector<agentInfo>::iterator agentIter = agents.begin(); agentIter != agents.end(); ++agentIter) {
		//for point-in-slice, check atan2 result against bound angles then check distance with radii bounds
		//atan2(y, x) returns signed radians [-pi, pi] need to convert to [0, 360] degrees for comparison
		float angle = atan2f(agentIter->COMz, agentIter->COMx);
		if(angle < 0) {
			angle = angle * (180.0f / static_cast<float>(M_PI)) + 360.0f;
		}
		else {
			angle = angle * (180.0f / static_cast<float>(M_PI));
		}
		
		//test rotational bounds, if true place agent in disregard list and move on, if false continue to next iteration
		if(angle >= wedge.firstBound && angle <= wedge.secondBound) {
			//test distance bounds
			float distance = sqrt(agentIter->COMz * agentIter->COMz + agentIter->COMx * agentIter->COMx);
			if(distance >= wedge.minDist && distance < currNearest && distance <= wedge.maxDist) {
				currNearest = distance;
				nearestSpeed = sqrtf(agentIter->velx * agentIter->velx + agentIter->velz * agentIter->velz);
				nearestTheta = floorf(0.5f + (12.0f * atan2f(agentIter->velz, agentIter->velx)) / M_PI);
			}
		}
	}

	if(!wedge.agentOnly) {
		for(vector<obstacleInfo>::iterator obstacleIter = obstacles.begin(); obstacleIter != obstacles.end(); ++obstacleIter) {
			//need a collision test with the obstacle to get the nearest point
		
			//3 test rays will be fired, minBound, maxBound, and bisection
			float bisectAngle = ((wedge.firstBound + wedge.secondBound) / 2.0f) / (180.0f / static_cast<float>(M_PI));
			float minAngle = wedge.firstBound / (180.0f / static_cast<float>(M_PI));
			float maxAngle = wedge.secondBound / (180.0f / static_cast<float>(M_PI));
			
			Point origin(0.0f, 0.0f, 0.0f);
			Vector direction = rotateInXZPlane(Vector(1.0f, 0.0f, 0.0f), bisectAngle + obstacleIter->rayCorrectionAngle);
			Ray ray;
			ray.dir = direction;
			ray.pos = origin;
			ray.maxt = wedge.maxDist;
			ray.mint = wedge.minDist;

			float distance;
			if(rayIntersectsBox2D(obstacleIter->xmin, obstacleIter->xmax, obstacleIter->zmin, obstacleIter->zmax, ray, distance)) {
				if(distance < currNearest) {
					//if the shortest distance is closer than the current nearest distance, record the new distance and velocities of 0
					currNearest = distance;
					nearestSpeed = 0.0f;
					nearestTheta = 0;
				}
			}
			
			direction = rotateInXZPlane(Vector(1.0f, 0.0f, 0.0f), minAngle + obstacleIter->rayCorrectionAngle);
			ray.dir = direction;
			if(rayIntersectsBox2D(obstacleIter->xmin, obstacleIter->xmax, obstacleIter->zmin, obstacleIter->zmax, ray, distance)) {
				if(distance < currNearest) {
					//if the shortest distance is closer than the current nearest distance, record the new distance and velocities of 0
					currNearest = distance;
					nearestSpeed = 0.0f;
					nearestTheta = 0;
				}
			}

			direction = rotateInXZPlane(Vector(1.0f, 0.0f, 0.0f), maxAngle + obstacleIter->rayCorrectionAngle);
			ray.dir = direction;
			if(rayIntersectsBox2D(obstacleIter->xmin, obstacleIter->xmax, obstacleIter->zmin, obstacleIter->zmax, ray, distance)) {
				if(distance < currNearest) {
					//if the shortest distance is closer than the current nearest distance, record the new distance and velocities of 0
					currNearest = distance;
					nearestSpeed = 0.0f;
					nearestTheta = 0;
				}
			}
		}
	}
	
	//if isEmpty is still true, store that all is clear in some way
	if(currNearest == numeric_limits<float>::infinity()) {
		//use distance of -1 for empty wedge
		result << -1 << "," << 0.0f << "," << 0;
	}
	else {
		if(nearestTheta == -12) {
			nearestTheta = 12;
		}
		result << (floorf(2 * currNearest + 0.5f) / 2) << "," << nearestSpeed << "," << nearestTheta;
	}
}

void GeneratorEngine::handleDensity(const StateConfig::density area, vector<agentInfo>& agents, vector<obstacleInfo>& obstacles, stringstream& result) {
	//for simplicity just use the count, all areas are the same size so it would just be a common denominator for now.
	unsigned int counter = 0;
	
	for(vector<agentInfo>::iterator agentIter = agents.begin(); agentIter != agents.end(); ++agentIter) {
		//for point-in-slice, check atan2 result against bound angles then check distance with radii bounds
		//atan2(y, x) returns signed radians [-pi, pi] need to convert to [0, 360] degrees for comparison
		float angle = atan2f(agentIter->COMz, agentIter->COMx);
		if(angle < 0) {
			angle = angle * (180.0f / static_cast<float>(M_PI)) + 360.0f;
		}
		else {
			angle = angle * (180.0f / static_cast<float>(M_PI));
		}
		
		//test rotational bounds, if true place agent in disregard list and move on, if false continue to next iteration
		if(angle >= area.firstBound && angle <= area.secondBound) {
			//test distance bounds
			float distance = sqrt(agentIter->COMz * agentIter->COMz + agentIter->COMx * agentIter->COMx);
			if(distance >= area.minDist && distance <= area.maxDist) {
				counter++;
			}
		}
	}

	if(!area.agentOnly) {
		for(vector<obstacleInfo>::iterator obstacleIter = obstacles.begin(); obstacleIter != obstacles.end(); ++obstacleIter) {
			//need a collision test with the obstacle to get the nearest point
		
			//3 test rays will be fired, minBound, maxBound, and bisection
			float bisectAngle = ((area.firstBound + area.secondBound) / 2.0f) / (180.0f / static_cast<float>(M_PI));
			float minAngle = area.firstBound / (180.0f / static_cast<float>(M_PI));
			float maxAngle = area.secondBound / (180.0f / static_cast<float>(M_PI));
			Point origin(0.0f, 0.0f, 0.0f);
			Vector direction = rotateInXZPlane(Vector(1.0f, 0.0f, 0.0f), bisectAngle + obstacleIter->rayCorrectionAngle);
			Ray ray;
			ray.dir = direction;
			ray.pos = origin;
			ray.maxt = area.maxDist;
			ray.mint = area.minDist;

			float distance;
			if(rayIntersectsBox2D(obstacleIter->xmin, obstacleIter->xmax, obstacleIter->zmin, obstacleIter->zmax, ray, distance)) {
				counter++;
			}

			direction = rotateInXZPlane(Vector(1.0f, 0.0f, 0.0f), minAngle + obstacleIter->rayCorrectionAngle);
			ray.dir = direction;
			if(rayIntersectsBox2D(obstacleIter->xmin, obstacleIter->xmax, obstacleIter->zmin, obstacleIter->zmax, ray, distance)) {
				counter++;
			}

			direction = rotateInXZPlane(Vector(1.0f, 0.0f, 0.0f), maxAngle + obstacleIter->rayCorrectionAngle);
			ray.dir = direction;
			if(rayIntersectsBox2D(obstacleIter->xmin, obstacleIter->xmax, obstacleIter->zmin, obstacleIter->zmax, ray, distance)) {
				counter++;
			}
		}
	}

	result << counter;
}

void GeneratorEngine::handleFlow(const StateConfig::flow area, vector<agentInfo>& agents, stringstream& result) {
	unsigned int counter = 0;
	float runningSpeed = 0;
	int runningTheta = 0;

	for(vector<agentInfo>::iterator agentIter = agents.begin(); agentIter != agents.end(); ++agentIter) {
		//for point-in-slice, check atan2 result against bound angles then check distance with radii bounds
		//atan2(y, x) returns signed radians [-pi, pi] need to convert to [0, 360] degrees for comparison
		float angle = atan2f(agentIter->COMz, agentIter->COMx);
		if(angle < 0) {
			angle = angle * (180.0f / static_cast<float>(M_PI)) + 360.0f;
		}
		else {
			angle = angle * (180.0f / static_cast<float>(M_PI));
		}
		
		//test rotational bounds, if true place agent in disregard list and move on, if false continue to next iteration
		if(angle >= area.firstBound && angle <= area.secondBound) {
			//test distance bounds
			float distance = sqrt(agentIter->COMz * agentIter->COMz + agentIter->COMx * agentIter->COMx);
			if(distance >= area.minDist && distance <= area.maxDist) {
				counter++;
				runningSpeed += sqrtf(agentIter->velx * agentIter->velx + agentIter->velz * agentIter->velz);
				runningTheta += floorf(0.5f + (12.0f * atan2f(agentIter->velz, agentIter->velx)) / M_PI);
			}
		}
	}

	if(counter == 0) {
		//if the area has no population, we would run into a divide by 0 error
		result << 0 << "," << 0;
	}
	else {
		int finalTheta = static_cast<int>(0.5f + (static_cast<float>(runningTheta) / static_cast<float>(counter)));
		if(finalTheta == -12) {
			finalTheta = 12;
		}
		result << runningSpeed / static_cast<float>(counter) << "," << finalTheta;
	}
}

void GeneratorEngine::handleObstacles(const StateConfig::obstaclesPresent, vector<obstacleInfo>& obstacles, stringstream& result) {
	for(vector<obstacleInfo>::const_iterator citer = obstacles.cbegin(); citer != obstacles.cend(); ++citer) {
		if(citer->zmax < 10 && citer->zmin > -10 && citer->xmax < 10 && citer->xmin > -10) {
			result << 'y';
			return;
		}
	}
	result << 'n';
} 

void GeneratorEngine::extractContextSamples(unsigned int contextNumber, vector<agentInfo>& agents, vector<obstacleInfo>& obstacles) {
	stringstream result;

	for(vector<StateConfig::state*>::const_iterator citer = stateSpace->iteratorBegin(-1); citer != stateSpace->iteratorEnd(-1); ++citer) {
		try {
			StateConfig::StateEnum childClass = (*citer)->type_id;
							
			switch(childClass) {
			case StateConfig::STATE_SLICE: 
				handleSlice(*(dynamic_cast<const StateConfig::slice*>(*citer)), agents, obstacles, result);
				break;
			case StateConfig::STATE_DENSITY:
				handleDensity(*(dynamic_cast<const StateConfig::density*>(*citer)), agents, obstacles, result);
				break;
			case StateConfig::STATE_FLOW:
				handleFlow(*(dynamic_cast<const StateConfig::flow*>(*citer)), agents, result);
				break;
			case StateConfig::STATE_OBSTACLES:
				handleObstacles(*(dynamic_cast<const StateConfig::obstaclesPresent*>(*citer)), obstacles, result);
				break;
			default:
				//throw generic exception, shouldn't get here except with coding errors
				throw new GenericException("State configuration calls for an invalid type of space.");
			}

			result << ",";
		}
		catch (exception &e) {
			cerr << "\nERROR: exception caught while parsing state space:\n" << e.what() << "\nIgnored this space and continued.";
		}
	}

	//add the contextNumber and a newline because this sample is complete
	result << contextNumber << '\n';

	contextSamples.push_back(result.str());
}

void GeneratorEngine::extractSpecializedSamples(unsigned int contextNumber, vector<agentInfo>& agents, vector<obstacleInfo>& obstacles) {
	stringstream result;

	for(vector<StateConfig::state*>::const_iterator citer = stateSpace->iteratorBegin(contextNumber); citer != stateSpace->iteratorEnd(contextNumber); ++citer) {
		try {
			StateConfig::StateEnum childClass = (*citer)->type_id;
							
			switch(childClass) {
			case StateConfig::STATE_SLICE: 
				handleSlice(*(dynamic_cast<const StateConfig::slice*>(*citer)), agents, obstacles, result);
				break;
			case StateConfig::STATE_DENSITY:
				handleDensity(*(dynamic_cast<const StateConfig::density*>(*citer)), agents, obstacles, result);
				break;
			case StateConfig::STATE_FLOW:
				handleFlow(*(dynamic_cast<const StateConfig::flow*>(*citer)), agents, result);
				break;
			case StateConfig::STATE_OBSTACLES:
				handleObstacles(*(dynamic_cast<const StateConfig::obstaclesPresent*>(*citer)), obstacles, result);
				break;
			default:
				//throw generic exception, shouldn't get here except with coding errors
				throw new GenericException("State configuration calls for an invalid type of space.");
			}

			result << ",";
		}
		catch (exception &e) {
			cerr << "\nERROR: exception caught while parsing state space:\n" << e.what() << "\nIgnored this space and continued.";
		}
	}

	//calculate goal data for this footstep: represent goal data as distance and angle for now
	float goalx = subject.subjectGoal.x - subject.subjectPrevX;
	float goalz = subject.subjectGoal.z - subject.subjectPrevZ;
	float xformedX = goalx * cosf(-subject.theta) - goalz * sinf(-subject.theta);
	float xformedZ = goalx * sinf(-subject.theta) + goalz * cosf(-subject.theta);
	float tempDistance = sqrtf(xformedX * xformedX + xformedZ * xformedZ);
	int distanceToGoal = static_cast<int>(ceilf(tempDistance));	//discretize this as well
	if(distanceToGoal > 20) {
		distanceToGoal = 20;
	}

	float tempAngle = ((12.0f * atan2(xformedZ, xformedX)) / M_PI);
	int angle = floorf(0.5f + tempAngle);
	if(angle == -12) {
		angle = 12;	//they're the same
	}

	result << distanceToGoal << "," << angle;

	//give other applicable agent state information, for now only 1 or 0 for whichfoot but may add velocity
	//result << "," << (subject.step.whichFoot ? "r" : "l");

	result << "," << static_cast<int>(lastFrame.step.state);

	result << ",";

	//add footstep selection to the table if it doesn't already exist and attach the table index of the action selection to this sample
		
	//we want to store the proper footstep action data to "reconstitute" the action at runtime, we could create the footstep using the same code
	//as footstepAI by remembering the very small number of necessary parameters

	//derive local-space stepInfo
	stepInfo action;
		
	//can create the footstep with stepDuration, phi (real or runtime constructed), and desired velocity with the prev step using existing footstepAI functionality
	//stepDuration is trivial
	action.duration = subject.step.endTime - subject.step.startTime;
			
	//phi needs to be converted to local-space, or flagged for goal-oriented calculation at runtime
	if(subject.step.phiIsIdeal) {
		action.phi = FLT_MAX;
	}
	else {
		action.phi = (subject.step.parabolaOrientationPhi - subject.theta) / (M_PI / 2);	//recall theta was calculated to be negative, so we add here instead of subtract
	}
	
	//"desired velocity" is a discretized parameter used to by the footstepAI code to generate realized speeds and was added to the footrec file format
	action.desiredSpeed = subject.step.desiredSpeed;

	//store the enum
	action.footstepEnum = subject.step.state;
		
	vector<stepInfo>::iterator stepIter = find(actionMap[subject.step.whichFoot].begin(), actionMap[subject.step.whichFoot].end(), action);	//find is O(n) in the length of the vector, which is very very small
	unsigned int actionIndex = distance(actionMap[subject.step.whichFoot].begin(), stepIter);

	if(!validation && actionIndex == actionMap[subject.step.whichFoot].size()) {
		//action not already in the map, add it and the index becomes valid
		actionMap[subject.step.whichFoot].push_back(action);
	}

	if(!(validation && actionIndex == actionMap[subject.step.whichFoot].size())) {
		//add the actionIndex and a newline because this sample is complete
		result << actionIndex << '\n';
		actionUsage[subject.step.whichFoot][actionIndex]++;

		samples[subject.step.whichFoot].push_back(result.str());
	}
}