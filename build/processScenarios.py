#!/usr/bin/python
import sys
import os
import math

usage = 'Usage: ' + sys.argv[0] + ' <scenariosFile>.ext \n\
It will process the log file and record the output in <scenariosFile>.ext.pro'


class Agent:
	def __init__(self,posX,posZ,goalX,goalZ):
		self.posX = float(posX)
		self.posZ = float(posZ)
		self.goalX = float(goalX)
		self.goalZ = float(goalZ)
	def predictIntersection(self,a):
		# implicit form of line 1
		A1 = self.goalZ - self.posZ
		B1 = self.posX - self.goalX
		C1 = A1*self.posX + B1*self.posZ

		# implicit form of line 2
		A2 = a.goalZ - a.posZ
		B2 = a.posX - a.goalX
		C2 = A2*a.posX + B2*a.posZ

		det = A1*B2-A2*B1
		if( abs(det) < 0.00001) :
			#lines are parallel
			return 0
		else:
			x = (B2*C1 - B1*C2)/det
			z = (A1*C2 - A2*C1)/det
			print "x = ", x, " z = ", z
			if( x < a.posX and x < a.goalX):
				return 0
			if( x > a.posX and x > a.goalX):
				return 0
			if( z < a.posZ and z < a.goalZ) :
				return 0
			if( z > a.posZ and z > a.goalZ) :
				return 0
			if( x < self.posX and x < self.goalX):
				return 0
			if( x > self.posX and x > self.goalX):
				return 0
			if( z < self.posZ and z < self.goalZ) :
				return 0
			if( z > self.posZ and z > self.goalZ) :
				return 0
			return 1
		

def processScenarios(scenariosFileName):
	print scenariosFileName

	# open the log file
	try:
		scenariosFile = open(scenariosFileName, 'r')
	except IOError:
		print 'Error: cannot open', scenariosFileName
		sys.exit(-1)
	resultsFileName = scenariosFileName + '.pro'
	try:
		resultsFile = open(resultsFileName, 'w')
	except IOError:
		print 'Error: cannot open',resultsFileName
		sys.exit(-1)

	for line in scenariosFile:
		if ( line[0] != '#' and len(line) > 10 and line[0] !='S'):
			processScenario(line,resultsFile)
	scenariosFile.close()
	resultsFile.close()

def processScenario(line, resultsFile):
	format = "ScenarioId      RandCalls       Collisions      Time    Effort" +\
		"Acceleration    Score   Success NumUniqueCollisions     totalTimeEnabled"+\
		"totalAcceleration       totalDistanceTraveled   totalChangeInSpeed"+\
		"totalDegreesTurned      sumTotalOfInstantaneousAcceleration" +\
		"sumTotalOfInstantaneousKineticEnergies  averageKineticEnergy"+\
		"AgentComplete   AgentSuccess    optimalPathLength       lengthRatio" +\
		"optimalTime     timeRatio"+\
		"ScenarioId RandCalls RandomSeed"+\
		"NumAgents NumObstacles"+\
		"\tAgent-i-PosX\tAgent-i-PoZ"+\
		"\tAgent-i-GoalX\tAgent-i-GoalZ\tNumObstacles\tObs-i-xmin\tObs-i-xmax"+\
		"\tObjs-i-zmin\tObs-i-zmax\tAreaRatio\tNumIntersections\n"
	
	# set the indices for the constant size part of the record
	# WARNING these are hardcoded value
	(ScenarioId, RandCalls, Collisions, Time, Effort,\
	Acceleration, Score, Success, NumUniqueCollisions, totalTimeEnabled,\
	totalAcceleration,  totalDistanceTraveled, totalChangeInSpeed,\
	totalDegreesTurned, sumTotalOfInstantaneousAcceleration,\
	sumTotalOfInstantaneousKineticEnergies,  averageKineticEnergy,\
	AgentComplete,   AgentSuccess,    optimalPathLength,       lengthRatio,\
	optimalTime,     timeRatio,\
	ScenarioId2, RandCalls2, RandomSeed, NumAgents, NumObstacles) = range(0,28)

	#read the record
	fields = line.split()
	numAgents = int(fields[NumAgents])
	numObstacles = int(fields[NumObstacles])
	print "NumAgents = ", numAgents, " NumObstacles = ", numObstacles

	agents = []
	for i in range(0,numAgents):
		offset = NumObstacles + 1 + i*4
		posX = fields[offset]
		posZ = fields[offset + 1]
		goalX = fields[offset + 2]
		goalZ = fields[offset + 3]
		a = Agent(posX,posZ,goalX,goalZ)
		agents.append(a)

	# count number of predicted intersections between agent 0 and the others
	numPredictedInters = int(0)
	for i in range(1,numAgents):
		numPredictedInters = numPredictedInters + \
			agents[0].predictIntersection(agents[i])
					    
		
	print "numPredictedInters = ",  numPredictedInters
	#a1 = Agent(0,0,5,5)
	#a2 = Agent(1,0,1,7)
	#print a1.computeNumPredInters(a2)

if __name__ == "__main__":
	if( len(sys.argv) == 1 ):
		print usage 
		sys.exit(-1)

	scenariosFileName = sys.argv[1]
	processScenarios(scenariosFileName)
