#!/usr/bin/python
import sys
import os
import subprocess

usage = 'Usage: ' + sys.argv[0] + ' <logFile> \n\
It will process the log file, record the output in <logFile>.sce\n\
and store the commands in <logFile>.com\n' +\
'It also merges the two files <logFile>.ext'

def setCommand(randomSeed,randomCalls, recordFileName):
	command = './steersim -module scenario,scenarioAI=pprAI,numScenarios=1,fixedDirection,' + \
	'reducedGoals,radius=7,grid=10,maxAgents=6,minAgents=3,' + \
	'randomSeed=' + str(randomSeed) +',' + \
	'randomCalls=' + str(randomCalls)+ ',' + \
	'checkAgentValid,reducedGoals,fixedSpeed,fixedDirection,checkAgentRelevant,' + \
	'fileToAppendScenarios=\"' + recordFileName + '\" -commandline'
	return command

# merges f1 and f2 into r
def mergeFiles(fn1,fn2,rn):
	try:
		f1 = open(fn1, 'r')
	except IOError:
		print 'Error: cannot open',fn1
		sys.exit(-1)
	try:
		f2 = open(fn2, 'r')
	except IOError:
		print 'Error: cannot open',fn2
		sys.exit(-1)

	lines1 = f1.readlines()
	lines2 = f2.readlines()
	if( len(lines1) != len(lines2)):
		print "Error: files do not have the same number of lines!"
		f1.close()
		f2.close()
		return
	
	try:
		r = open(rn, 'w')
	except IOError:
		print 'Error: cannot open',rn
		sys.exit(-1)

	lines = []
	for l1,l2 in zip(lines1,lines2):
		lines.append(l1.rstrip()+ '\t' + l2)
	r.writelines(lines)
	f1.close()
	f2.close()
	r.close()

def recordScenarios(logFileName):
	print logFileName

	# open the log file
	try:
		logFile = open(logFileName, 'r')
	except IOError:
		print 'Error: cannot open', logFileName
		sys.exit(-1)
	scenariosFileName = logFileName + '.sce'
	try:
		scenariosFile = open(scenariosFileName, 'w')
	except IOError:
		print 'Error: cannot open', logFileName+'.com'
		sys.exit(-1)
	commandLogName = logFileName+'.com'
	try:
		commandLog = open(commandLogName, 'w')
	except IOError:
		print 'Error: cannot open', logFileName+'.com'
		sys.exit(-1)

	# read the the header
	randomSeedLine = logFile.readline() ;
	print randomSeedLine.strip()
	randomSeed = int(randomSeedLine)

	# read the header line
	line = logFile.readline()

	# write the randomSeed to follow the format
	scenariosFile.write(str(randomSeed) + '\n')

	# write the header. This has to match with what the program actually rights!!!
	scenariosFile.write("ScenarioId\tRandCalls\tRandomSeed"+\
			"\tNumAgents\tNumObstacles\tAgent-i-PosX\tAgent-i-PoZ"+\
			"\tAgent-i-GoalX\tAgent-i-GoalZ\tObs-i-xmin\tObs-i-xmax"+\
			"\tObjs-i-zmin\tObs-i-zmax\tAreaRatio\tNumIntersections\n")
	# close the file
	scenariosFile.close()

	# record each scenario
	for line in logFile.readlines():
		#read the randomCalls field
		randomCalls = long( line.split()[1] )
		#set the command
		command = setCommand(randomSeed,randomCalls,scenariosFileName)
		# log the command
		commandLog.write(command + '\n')
		# execute the command
		os.system(command)

	# close the files
	logFile.close()
	commandLog.close()

	mergeFiles(logFileName, scenariosFileName, logFileName+'.ext')

if __name__ == "__main__":
	if( len(sys.argv) == 1 ):
		print usage 
		sys.exit(-1)

	logFileName = sys.argv[1]
	recordScenarios(logFileName)
