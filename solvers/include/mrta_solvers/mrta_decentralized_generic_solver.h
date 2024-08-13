#pragma once
#include <mrta_solvers/mrta_generic_solver.h>
#include <set>

class MrtaDecentralizedGenericSolver : public MrtaGenericSolver {
public:
  MrtaDecentralizedGenericSolver(){};
  MrtaDecentralizedGenericSolver(const std::string &robot_name): MrtaGenericSolver(), robot_name(robot_name){};
  ~MrtaDecentralizedGenericSolver(){
      // delete others_solutions;
  };

protected:
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

  std::set<std::string> relevant_tasks_names;

  size_t timestep = 0;
  size_t schedule_convergence_timer = 0;

  size_t MAX_SCHEDULE_CONVERGENCE_TIME = 500;

  bool converged = false;

  std::string robot_name;

  // std::set<const MrtaSolution::CompleteSolution &> const *others_solutions;
};
