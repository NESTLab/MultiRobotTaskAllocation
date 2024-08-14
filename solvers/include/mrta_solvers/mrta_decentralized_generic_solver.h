#pragma once
#include <mrta_solvers/mrta_generic_solver.h>
#include <set>

class MrtaDecentralizedGenericSolver : public MrtaGenericSolver {
public:
  MrtaDecentralizedGenericSolver(const std::string &robot_name, int robot_id)
      : MrtaGenericSolver(), robot_name(robot_name), robot_id(robot_id){};
  ~MrtaDecentralizedGenericSolver() override {};

protected:
  void updateMrtaConfig(
      const MrtaConfig::CompleteConfig &mrta_complete_config_in) override {
    MrtaGenericSolver::updateMrtaConfig(mrta_complete_config_in);
    setRequiredMatrices();
  };
  /**
   * brief:
   */
  void solveMrtaProblem(
      const MrtaConfig::CompleteConfig &mrta_complete_config,
      MrtaSolution::CompleteSolution &ret_complete_solution) override {
    solveOneIteration(mrta_complete_config, ret_complete_solution);
    ++timestep;
  };

  void updateWorldStatus() override{};

  void setRequiredMatrices() { setRelevantTasks(); }

  void setRelevantTasks() {
    for (int j = START_ID + 1; j < END_ID; ++j) {
      double total_effective_contribution =
          task_requirements_matrix.row(j).dot(robot_skillset_matrix.row(robot_id));
      if (total_effective_contribution > 0.0) {
        std::string task_name = mrta_complete_config->setup.all_destination_names.at(j);
        relevant_tasks_names.push_back(task_name);
      }
    }
  };

  virtual bool checkConvergence() = 0;

  /**
   * @brief: solveOneIteration()
   *
   */
  virtual bool
  solveOneIteration(const MrtaConfig::CompleteConfig &mrta_complete_config,
                    MrtaSolution::CompleteSolution &ret_complete_solution) = 0;

  void updateKnowledgeAboutOtherAgentsSchedules(
      const std::set<const MrtaSolution::CompleteSolution &>
          &others_solutions_in) {
    // others_solutions = &others_solutions_in;
  }

  std::vector<std::string> relevant_tasks_names;

  size_t timestep = 0;
  size_t schedule_convergence_timer = 0;

  size_t MAX_SCHEDULE_CONVERGENCE_TIME = 500;

  bool converged = false;

  std::string robot_name;
  int robot_id;

  Eigen::Matrix2d robot_skillset;
  Eigen::Matrix2d task_skillset;

  // std::set<const MrtaSolution::CompleteSolution &> const *others_solutions;
};
