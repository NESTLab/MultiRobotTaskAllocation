#pragma once
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

class MrtaConfig {
public:
  MrtaConfig(){};
  ~MrtaConfig(){};

  struct Setup {
    int number_of_robots;
    int number_of_tasks;
    int number_of_capabilities;

    double epsilon;
    double mean_percent;
    double task_arena_size;

    bool use_stochasticity;
    bool plot_solution;
    bool use_robot_ends;
    bool save_plot_solution;

    std::vector<std::string> skill_names;
  };

  struct Position {
    double pos_x;
    double pos_y;
  };

  struct Task {
    int id;
    Position position;
    std::map<std::string, double> skillset;
    double duration;
  };

  struct Robot {
    int id;
    Position position;
    std::map<std::string, double> skillset;
  };

  const Setup &GetSetupInfo() const { return setup; }

  const std::vector<Task> &GetAllTasks() const { return tasks; }

  const std::vector<Robot> &GetAllRobots() const { return robots; }

  const std::map<std::string, double> &GetSkillDegradations() const {
    return skill_degradation_rates;
  }

  bool isDebugMode() { return Debug; }

  void setDebugMode(bool debug_mode) { Debug = debug_mode; }

  void SetSetupInfo(const Setup& setup_in) {
    setup = setup_in;
  };

  void SetAllTasks(const std::vector<Task> & tasks_in) {
     tasks = tasks_in; }

  void SetAllRobots(const std::vector<Robot> & robots_in) {
     robots = robots_in; }

  void SetSkillDegradations(const std::map<std::string, double> & skill_degradation_rates_in) {
    skill_degradation_rates = skill_degradation_rates_in;
  }

protected:
  // Setup values
  Setup setup;
  std::vector<Task> tasks;
  std::vector<Robot> robots;
  std::map<std::string, double> skill_degradation_rates;

  bool Debug = false;

  friend class MrtaJsonParser;
};