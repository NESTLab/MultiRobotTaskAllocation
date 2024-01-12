# Multi Robot Task Allocation (MRTA)
This repository provides solution for the multi robot task allocation (MRTA) problems.
___

## Usage:
Users can use this repository in two ways: with or without using a json file. There is no preference for one over other, it is up to users to decide.

### Using JSON file
This repository uses [nlohmann/json](https://github.com/nlohmann/json.git) library to parse the json file. Please make sure to install it prior to usage. 

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


### Directly using `MrtaConfig` 
> Irrespective of you use json file or not, currently you are required to have [nlohmann/json](https://github.com/nlohmann/json.git) library installed. Otherwise the project may not compiled. This will be fixed in future. 

