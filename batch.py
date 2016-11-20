import shlex, subprocess
import os 

for i in range(1):
	print("i = ", i)

	numScenarios = pow(2, i)
	AI = "egocentricAI"

	radius = 7
	gridResX = 10
	gridResY = 10
	maxAgents = 6
	minSpeed = 1
	maxSpeed = 2.5
	randSeed = i
	recFile = "rec"
	recFileFolder = "test" + str(i)

	command = "./steersim.exe -module scenario,scenarioAI=" + AI + ",numScenarios=" + str(numScenarios)	
	command += ",benchmarkLog=trial_" + str(i).zfill(3) + ".dat"
	command += ",parameters=" + str(radius) + "+" + str(gridResX) + "+" + str(gridResY) + "+" + str(maxAgents) + "+" + str(minSpeed) + "+" + str(maxSpeed) + "+" + str(randSeed)
	command += ",recFile=" + recFile + ",recFileFolder=" + recFileFolder
	command += " -config ../../config.xml"
	command += " -commandline"

	print(command + "\n\n")

	#args = shlex.split(command);
	#retcode = subprocess.call(args)
	
	os.system(command)
	
	
	#retcode = subprocess.call(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)




