#pragma once
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace MrtaConfig {
namespace StdSkillNames{
  static const std::string BATTERY_SKILL = "BATTERY";
}

struct Setup {
  int number_of_robots;
  int number_of_destinations;
  int number_of_skills;

  double epsilon;
  double mean_percent;

  bool use_stochasticity;
  bool use_robot_ends;

  std::vector<std::string> all_destination_names;
  std::vector<std::string> all_robot_names;
  std::vector<std::string> all_skill_names;
};

struct SolverConfig {
  // Members and methods for SolverConfig
};

enum SOLVER_TYPE { HEURISTIC_SOLVER, MILP_SOLVER, SORTED_SOLVER };

struct SolverInfo {
  SOLVER_TYPE solver_type;
  SolverConfig solver_config;
};

struct Position {
  double pos_x;
  double pos_y;
};

struct Task {
  std::string task_name;
  Position position;
  std::map<std::string, double> skillset;
  double duration;
};

struct Robot {
  std::string robot_name;
  Position position;
  double velocity;
  Position desired_end_position;
  std::map<std::string, double> skillset;
};

struct Environment {
  std::map<std::string, double> skill_degradation_rate_map;
  std::map<std::string, double> path_stochasticity_sigma_values;
};

struct CompleteConfig {
  Setup setup;
  SolverInfo solver_info;
  Environment environment;
  std::map<std::string, Task> tasks_map;
  std::map<std::string, Robot> robots_map;
};

}; // namespace MrtaConfig