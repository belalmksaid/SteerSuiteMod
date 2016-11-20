# Introduction:

SteerOpt is a package that uses SteerPlugin to run crowd simulations. It is not really a module because the package could make use of all of the features in the scenario module for statistical sampling. I believe this makes it fit best in the area of totally controlling SteerSuite, similar to SteerSim.

**Note:** This project is still in alpha stage. Expect changes and issues.

## Structure:

**Insert some diagram here....** 

### Initialization:

To run run an optimization you need to specify a configuration file. In the configuration file there are two options to run a steering algorithm optimization or a environment optimization.

To optimize a steering algorithm you can use the following command.
```bash
./build/bin/steeroptRun.exe steeropt/data/config_optimize_steering_alg.xml
```

To optimize a for Degree
```bash
./build/bin/steeroptRun.exe steeropt/data/config_optimize_degree.xml
```

To optimize an environment for diversity:
```bash
./build/bin/steeroptRun.exe steeropt/data/config_optimize_architecture.xml

```
OR for a simpler example.
```bash
./steeroptRun.exe steeropt/data/config_optimize_simple_architecture.xml

```


The config file has a section in it for the steeropt library

```XML
<steeropt>
        	<objectiveFunction>Degree</objectiveFunction>
        	<maxIterations>1000</maxIterations>
        	<graphFileName>../../testcases/complexBuilding.graph</graphFileName>
        	<parameterFileName>../../steeropt/data/optimize-complex-room.xml</parameterFileName>
        	<logFileName>diversityOpt.csv</logFileName>
        	<renderResults>true</renderResults>
        	<!-- Not Supported Yet
 	    	<renderOptimization>false</renderOptimization>
    	    <tolFunc>0.0001</tolFunc>
    	    <tolX>0.0001</tolX> 
        	 -->
        </steeropt>
```

**./steeropt/data/sfAI-params.xml** is a file that stores the [steering algorithm parameters](SteerSuiteAIParameterConfigFile) and bounds in a format.

The **./testcases/complexBuilding.graph** file stores the scene as a architectural graph. Which can be optimized using the above command.

## Notes:
Uses the [libcmaes](https://github.com/beniz/libcmaes) library for optimization.

### Building  

This library builds on its own just fine... As far as I know.

## Debugging

Run the program from the base SteerSuite directory.  

On Windows use the config file  
..\..\steeropt\data\config_optimize_degree.xml

On Unix use the config file  
../../data/config_optimize_degree.xml


## Convergence

You can log data in steersuite be setting the flag in the <logFileName>diversityOpt.csv</logFileName> in the config file.
If the data for the optimization is logged you can view it using plotting/plot_optimiztion.py.