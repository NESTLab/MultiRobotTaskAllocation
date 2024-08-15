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

namespace StdTaskNames{
  static const std::string START_TASK = "START";
  static const std::string END_TASK = "END";
}

// Custom comparator to compare the second value of the pairs
struct CompareSecond {
    bool operator()(const std::pair<std::string, double>& p1, const std::pair<std::string, double>& p2) {
        // For a max-heap based on the second value, return true if p1.second < p2.second
        // For a min-heap based on the second value, return true if p1.second > p2.second
        return p1.second >= p2.second;  // This creates a min-heap
    }
};

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

enum SOLVER_TYPE { DECENTRALIZED_HSS_SOLVER, HEURISTIC_SOLVER, MILP_SOLVER, SORTED_SOLVER };

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