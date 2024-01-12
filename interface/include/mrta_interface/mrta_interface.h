#pragma once
#include <memory>
#include <mrta_solvers/mrta_generic_solver.h>
#include <mrta_utilities/mrta_config.h>
#include <mrta_utilities/mrta_solution.h>
#include <stdio.h>

class MrtaInterface {
private:
  /* data */
public:
  MrtaInterface();
  ~MrtaInterface();

  // Some solvers may require something specific which may or may not have been
  // provided by the user. Hence return 'false' if initialization failed.
  inline bool setMrtaSolverMethod(std::shared_ptr<MrtaGenericSolver> solver) {
    solver_method = solver;
    return true;
  };

  // To check and print if all the essential fields are present in the config
  // Return false if mandatory field is missing
  // Also print which optional fields are provided and which are missing
  bool healthCheckConfig(const std::shared_ptr<const MrtaConfig::CompleteConfig>
                             mrta_complete_config);

  // This is strictly to be used by DECENTRALIZED solvers.
  // This will update what tasks have been already attended, which robot is
  //    attending which task at what time, etc information
  // Currently simply giving it the schedule of all the robots, but it can be
  //    done in a more sophesticated ways too
  void updateWorldStatus(
      const std::map<std::string, MrtaSolution::RobotTasksSchedule>
          &all_robot_schedules);

  // Returns the solution
  // However, if you have set any of the DECENTRALIZED solvers,
  // This will return the result of ONE iteration.
  // In that case, communicating data with other robots, updating the world
  // status
  //    and keeping track of convergence will be up to the user.
  std::shared_ptr<const MrtaSolution::CompleteSolution> const
  solveMrtaProblem(std::shared_ptr<const MrtaConfig::CompleteConfig> const
                       mrta_complete_config) {
    solver_method->updateMrtaConfig(mrta_complete_config);
    return solver_method->solveMrtaProblem();
  };

  void debugPrintConfigCompleteConfig(
      const std::shared_ptr<const MrtaConfig::CompleteConfig>
          mrta_complete_config);
  void debugPrintConfigSetup(const MrtaConfig::Setup &mrta_setup);
  void debugPrintConfigTasksMap(
      const std::map<std::string, MrtaConfig::Task> &mrta_task_map);
  void debugPrintConfigRobotsMap(
      const std::map<std::string, MrtaConfig::Robot> &mrta_robot_map);
  void
  debugPrintConfigEnvironment(const MrtaConfig::Environment &mrta_environment);

  void debugPrintSolution(
      std::shared_ptr<const MrtaSolution::CompleteSolution> const solution);

  void debugPrintSolutionSchedule(
      const std::map<std::string, MrtaSolution::RobotTasksSchedule>
          robots_schedule);
  void debugPrintSolutionQuality(
      const MrtaSolution::SolutionQuality &solution_quality);

private:
  std::shared_ptr<MrtaGenericSolver> solver_method;

  template <typename T>
  void debugPrintSingleLine(const std::string &field, const T &value,
                            int number_of_indents);
  
  template <typename T>
  void debugPrintSolutionTaskMap(const std::map<std::string, T> task_map, int indent_level = 0);

  template <typename T>
  bool healthCheckField(const std::string &field, const T &value);
  bool healthCheckNumOfElements(const std::string &item,
                                const std::string &item_detail,
                                int expected_count, int received_count);
  bool healthCheckMandatoryFields(
      const std::shared_ptr<const MrtaConfig::CompleteConfig>
          mrta_complete_config);
  bool
  healthCheckNumOfRobots(const std::shared_ptr<const MrtaConfig::CompleteConfig>
                             mrta_complete_config);
  bool
  healthCheckNumOfTasks(const std::shared_ptr<const MrtaConfig::CompleteConfig>
                            mrta_complete_config);

  const int NUMBER_OF_INDENTS_PER_LEVEL = 1;
  const int NUMBER_OF_DASHES_PER_INDENT = 3;
};
