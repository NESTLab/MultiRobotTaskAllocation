#pragma once
#include <mrta_solvers/mrta_generic_solver.h>
#include <set>

class MrtaDecentralizedGenericSolver : public MrtaGenericSolver {
public:
  MrtaDecentralizedGenericSolver(const std::string &robot_name);
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
    throw std::domain_error(
        "This function only works for the centralized solvers, whereas "
        "currently a decentralized solver is being used. Please use function "
        "`solveOneIteration()` in decentralized mode.");
  };

  void updateWorldStatus() override {};

  virtual bool checkConvergence() = 0;

  /**
   * @brief: solveOneIteration()
   *
   */
  virtual bool solveOneIteration() = 0;

  void updateKnowledgeAboutOtherAgentsSchedules(
      const std::set<const MrtaSolution::CompleteSolution &>
          &others_solutions_in) {
            // others_solutions = &others_solutions_in;
          }

  bool step() {
    solveOneIteration();
    ++timestep;
    return converged;
  };

  std::set<std::string> relevant_tasks_names;

  size_t timestep = 0;
  size_t schedule_convergence_timer = 0;

  size_t MAX_SCHEDULE_CONVERGENCE_TIME = 500;

  bool converged = false;

  std::string robot_name;

  // std::set<const MrtaSolution::CompleteSolution &> const *others_solutions;
};
