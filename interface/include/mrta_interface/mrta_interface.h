#pragma once
#include <memory>
#include <mrta_solvers/mrta_decentralized_hss_solver.h>
#include <mrta_solvers/mrta_generic_solver.h>
#include <mrta_solvers/mrta_heuristic_solver.h>
#include <mrta_solvers/mrta_milp_solver.h>
#include <mrta_solvers/mrta_sorted_solver.h>
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

  // Some solvers may require something specific which may or may not have been
  // provided by the user. Hence return 'false' if initialization failed.
  inline bool setMrtaSolverMethod(MrtaConfig::SolverInfo solver_info,
                                  const std::string &robot_name = "",
                                  int robot_id = -1) {
    bool solver_set = true;
    switch (solver_info.solver_type) {
    case MrtaConfig::SOLVER_TYPE::HEURISTIC_SOLVER:
      solver_method = std::make_shared<MrtaHeuristicSolver>();
      break;

    case MrtaConfig::SOLVER_TYPE::MILP_SOLVER:
      solver_method = std::make_shared<MrtaMilpSolver>();
      break;

    case MrtaConfig::SOLVER_TYPE::SORTED_SOLVER:
      solver_method = std::make_shared<MrtaSortedSolver>();
      break;

    case MrtaConfig::SOLVER_TYPE::DECENTRALIZED_HSS_SOLVER:
      solver_method =
          std::make_shared<MrtaDecentralizedHssSolver>(robot_name, robot_id);
      break;

    default:
      std::cerr << "Solver given in the config does not match the required "
                   "list. Supported solvers: HEURISTIC, MILP, SORTED"
                << std::endl;
      solver_set = false;
      break;
    }
    return solver_set;
  };

  // To check and print if all the essential fields are present in the config
  // Return false if mandatory field is missing
  // Also print which optional fields are provided and which are missing
  bool
  healthCheckConfig(const MrtaConfig::CompleteConfig &mrta_complete_config);

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
  void solveMrtaProblem(const MrtaConfig::CompleteConfig &mrta_complete_config,
                        MrtaSolution::CompleteSolution &ret_complete_solution) {
    solver_method->updateMrtaConfig(mrta_complete_config);
    solver_method->solveMrtaProblem(mrta_complete_config,
                                    ret_complete_solution);
  };

  /**
   * @brief Set the Limited Info Mode object
   * Defines if this solver is being used by the CAF methods. If that is the
   * case, then the skill matrix calculation should be skipped and should be
   * taken as an input from the user.
   *
   * @param limited_info_mode_in
   */
  inline void setLimitedInfoMode(bool limited_info_mode_in) {
    solver_method->setLimitedInfoMode(limited_info_mode_in);
  };

  void debugPrintConfigCompleteConfig(
      const MrtaConfig::CompleteConfig &mrta_complete_config);
  void debugPrintConfigSetup(const MrtaConfig::Setup &mrta_setup);
  void debugPrintConfigTasksMap(
      const std::map<std::string, MrtaConfig::Task> &mrta_task_map);
  void debugPrintConfigRobotsMap(
      const std::map<std::string, MrtaConfig::Robot> &mrta_robot_map);
  void
  debugPrintConfigEnvironment(const MrtaConfig::Environment &mrta_environment);

  void debugPrintSolution(const MrtaSolution::CompleteSolution &solution);

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
  void debugPrintSolutionTaskMap(const std::map<std::string, T> task_map,
                                 int indent_level = 0);

  void debugPrintSolutionTaskVec(const std::vector<std::string> &task_seq_vec,
                                 int indent_level = 0);

  template <typename T>
  bool healthCheckField(const std::string &field, const T &value);
  bool healthCheckNumOfElements(const std::string &item,
                                const std::string &item_detail,
                                int expected_count, int received_count);
  bool healthCheckMandatoryFields(
      const MrtaConfig::CompleteConfig &mrta_complete_config);
  bool healthCheckNumOfRobots(
      const MrtaConfig::CompleteConfig &mrta_complete_config);
  bool
  healthCheckNumOfTasks(const MrtaConfig::CompleteConfig &mrta_complete_config);
  bool healthCheckNumOfSkills(
      const MrtaConfig::CompleteConfig &mrta_complete_config);

  const int NUMBER_OF_INDENTS_PER_LEVEL = 1;
  const int NUMBER_OF_DASHES_PER_INDENT = 3;
};
