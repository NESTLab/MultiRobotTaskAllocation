# Multi Robot Task Allocation (MRTA)
This repository provides solution for the multi robot task allocation (MRTA) problems.
___

## Usage:
Users can use this repository in two ways: with or without using a json file. There is no preference for one over other, it is up to users to decide.

You can skip [to next section](#directly-using-mrtaconfig) if you are not using 

[Skip to Getting Solution](#getting-solution)

### Using JSON file

The expected structure of the json file looks as following:

```json
{
// Setup of the problem
"setup": {
  "robots" : (int),           // (MANDATORY) number of robots 
  "tasks"  : (int),           // (MANDATORY) number of tasks 
  "num_skills" : (int),       // (MANDATORY) total number of skills 

  "use_robot_ends" : (bool),  // (optional) (default: False) Whether to consider travel to end points or not
  "use_stochasticity": (bool),// (optional) (default: False) Whether to use the travel time stochasticity or not
  
  "epsilon"       : (float),  // (optional) (default: 0.95) Probability that a robot should arrive at a task within time
  "mean_percent"  : (float),  // (optional) (default: 10)  Gaussian variable for time stochasticity
  "configuration" : "NONE",   // (optional) (unused variable) Will be used in future to specify which solver to be used.
   },

// Task description
"tasks": {
  "<task_name>" : {         // Task Name
    "pos" :  {              // Task Position
      "x" :  (float), 
      "y" : (float)
    },
    "skillset" :  {          // Skillset required
      "<skill_1_name>" : (float),  // Amount of skill 1 required
      "<skill_3_name>" : (float)   // Note that you can skip skill 2 in description
    },
    "duration" : (float)           // Duration that the task will take
  }
},

// Robots description
"robots" : {
  "<robot_name>" : {           // Robot Name
    "pos" : {                  // Robot Starting position
        "x" : (float),
        "y" : (float)
    },
    "desired_end_position": { // Robot's desired position at the end. 
                              // NOTE: this field will only be considered if 
                              // field `use_robot_ends` is set to TRUE in the setup. 
        "x": (float),
        "y": (float)
    },
    "skillset" : {        // Robot Skillset
        "<skill_1_name>" : (float),  // Amount of skill 1 offered by robot
        "<skill_3_name>" : (float)   // Note that you can skip skill 2 in description
    }
  },

// (optional) Expected rate of skill degradation, per timestep. 
"skill_degradation_rates": {      
    "<skill_1_name>": (float),      // Expected rate of degradation for skill 1.  
    "BATTERY": 0.1                  // Example of usage
  }
},

// (optional) Sigma percent value for EACH path
// This section is NOT required if the field "use_stochasticity" is set to FALSE
// If the said field is set to true, then the sigma MUST be mentioned for each path
  "sigma_percent": {
      "0,0": 0,    // Sigma path for START to START
      "9,9": 0,    // Sigma path for END to END
      "0,9": [0-100] (float),
      "9,0": [0-100] (float),
      "0,1": 22.58226441484873, // Sigma path for START to task 0 (Task 0 has id of 1, and task 1 will have id of 2)
      "1,0": 22.58226441484873, // Note that even if the distance between the two tasks is same, their sigma percent values can be different
      ..
      ..
      ..
}
```

Once your `json` file is ready, you can parse it and get the required config file with following:

```cpp
std::shared_ptr<const MrtaConfig::CompleteConfig> const mrta_config_ptr =
      MrtaJsonParser::parseJsonFile("<json file path>/<json file name>.json");
```

### Directly using `MrtaConfig`
Here it is assumed that you already have a variable of type `MrtaConfig::CompleteConfig`. This `struct` follows the same structure and concept as the `json` file explained in the [previous section](#using-json-file). Follow that section to know more about the fields and variables in `MrtaConfig::CompleteConfig`.

To check if your created config variable is correct or not, you can use following function:
```cpp
  MrtaConfig::CompleteConfig complete_config;
  //
  // Fill data in `complete_config`
  //

  MrtaInterface mrta_interface;
  std::shared_ptr<const MrtaConfig::CompleteConfig> const mrta_config_ptr =
      std::make_shared<const MrtaConfig::CompleteConfig>(complete_config);

  bool config_ok = mrta_interface.healthCheckConfig(mrta_config_ptr);

```

To debug and pring the config data, you can use any of following functions:
```cpp
  mrta_interface.debugPrintConfigCompleteConfig(mrta_config_ptr)
  mrta_interface.debugPrintConfigSetup(mrta_config_ptr->setup)
  mrta_interface.debugPrintConfigTasksMap(mrta_config_ptr->tasks_map)
  mrta_interface.debugPrintConfigRobotsMap(mrta_config_ptr->robots_map)
  mrta_interface.debugPrintConfigEnvironment(mrta_config_ptr->environment)
```

## Getting solution
Once you have set the config file, you can get the solution with following:
```cpp
  std::shared_ptr<const MrtaSolution::CompleteSolution> const solution =
      mrta_interface.solveMrtaProblem(mrta_config_ptr);
```

You can print out the solution contents with following:
```cpp
  mrta_interface.debugPrintSolution(mrta_solution_ptr)
  mrta_interface.debugPrintSolutionSchedule(mrta_solution_ptr)
  mrta_interface.debugPrintSolutionQuality(mrta_solution_ptr)
```

For more details on the structure of MrtaSolution, go to this section:

## Config and Solution structures:
### MrtaConfig
This `struct` follows the same structure and concept as the `json` file explained in the [previous section](#using-json-file). Follow that section to know more about the fields and variables in `MrtaConfig::CompleteConfig`.

### MrtaSolution
This is how an example solution file will look like:
```
solution.solution_quality:
|---result_status : 0                 // 0 for Success and -1 for Failure
|---result_description : success      // Description of the solution result
|---maximum_robot_schedule : 50       // The maximum of all the robots' "travel time"
                                      // Here, the "travel time" is considered the 
                                      //    arrival time at the end location. 
                                      // If the `use_robot_ends` is set to FALSE:
                                      //    then this will be the time at which the 
                                      //    robot's last task was completed
|---sum_of_all_robot_schedules : 80   // The sum of all robots' "travel time"
                                      // Definition for "travel time" is the same as 
                                      //    in `maximum_robot_schedule`
|---solver_runtime : 2H:45M:18S       // The runtime for the solver to produce solution
                                      // The formatting of the time may change soon


// The map for robots' names and their task schedules
solution.robots_schedule:           
|---robot_0 :                         // Robot name
|   |---robot_id : robot_0                // Robot name (even though it is the same as 
|   |                                     //      `key`, it helps when this schedule 
|   |                                     //      object needs to be passed around)
|   |---task_sequence_map :               // Sequence in which the task will be attended
|   |   |---task_0 : 1                        // Here, it means that the robot will attend 
|   |   |                                     //      task_0 at index 1. 
|   |   |                                     // NOTE: index starts at 1, because index
|   |   |                                     //      0 is considered to be the robot's start
|   |   |                                     //      location. 
|   |---task_arrival_time_map :           // Robot's arrival time at the tasks
|   |   |---task_0 : 10                       // Here, it means that the robot should
|   |   |                                      //      arrive at the task at time 10 
|---robot_1 : 
|   |---robot_id : robot_1
|   |---task_sequence_map : 
|   |   |---task_0 : 2                        // NOTE: Here, the task_0 is also being attended
|   |   |                                     //      by this robot (in addition to robot_0)
|   |   |---task_1 : 1                        // However, this robot goes to task_1 first, then
|   |   |                                     //      goes to task_1 after completing the task.
|   |---task_arrival_time_map : 
|   |   |---task_0 : 30                       // As this robot goes to task_1 first then task_0
|   |   |---task_1 : 10                       //      the arrival time for the robot at task_0 
|   |   |                                     //      is later than task_1
|---robot_2 : 
|   |---robot_id : robot_2
|   |---task_sequence_map : 
|   |   |---task_2 : 1
|   |---task_arrival_time_map : 
|   |   |---task_2 : 10
```
