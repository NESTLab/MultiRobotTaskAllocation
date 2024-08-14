#pragma once
#include <mrta_solvers/mrta_decentralized_generic_solver.h>

class MrtaDecentralizedHssSolver : public MrtaDecentralizedGenericSolver {
public:
  MrtaDecentralizedHssSolver(const std::string &robot_name, int robot_id)
      : MrtaDecentralizedGenericSolver(robot_name, robot_id){};
  ~MrtaDecentralizedHssSolver() override{};

private:
  bool solveOneIteration(
      const MrtaConfig::CompleteConfig &mrta_complete_config,
      MrtaSolution::CompleteSolution &ret_complete_solution) override;

  void updateWorldStatus() override{};

  bool areAllTasksSatisfied() { return false; };

  bool checkConvergence() { return false; };

  /////////////////////////////////////////////////////////////
  /////////  T A S K   I N C L U S I O N   P H A S E  /////////
  /////////////////////////////////////////////////////////////
  void taskInclusionPhase();

  std::pair<size_t, size_t> getIndexOfPredecessorSuccessorToTask(
      const std::string &task, const std::vector<std::string> &curr_path_i_A,
      const std::vector<std::string> &ordered_tasks_i_B);

  inline std::vector<std::string>::const_iterator
  getIterToObjectInVector(const std::vector<std::string> &task_vector,
                          const std::string &task) {
    return std::find(task_vector.begin(), task_vector.end(), task);
  }

  inline int getIndexOfObjectInVector(
      const std::vector<std::string>::const_iterator &task_iter,
      const std::vector<std::string> &task_vector) {
    return task_iter - task_vector.begin();
  }

  inline int
  getIndexOfObjectInVector(const std::string &task,
                           const std::vector<std::string> &task_vector) {
    std::vector<std::string>::const_iterator task_iter =
        getIterToObjectInVector(task_vector,task);
    return getIndexOfObjectInVector(task_iter, task_vector);
  }

  inline bool
  isObjectInVector(const std::vector<std::string>::const_iterator &task_iter,
                   const std::vector<std::string> &task_vector) {
    return task_iter != task_vector.end();
  }

  inline int
  isObjectInVector(const std::string &task,
                           const std::vector<std::string> &task_vector) {
    std::vector<std::string>::const_iterator task_iter =
        getIterToObjectInVector(task_vector,task);
    return isObjectInVector(task_iter, task_vector);
  }

  double getCost(const std::vector<std::string> &curr_path_i_A);

  double
  getAfterTaskInsertionCost(const std::vector<std::string> &curr_path_i_A,
                            const std::string &task, size_t index);

  size_t getTaskInsertionIndex(const std::vector<std::string> &curr_path_i_A,
                               const std::string &task,
                               std::pair<size_t, size_t> index_of_pred_succ);

  std::vector<std::string> ordered_tasks_i_B;

  ////////////////////////////////////////////////////////////////////////
  /////////  C O M M   A N D   V A R   U P D A T E   P H A S E   /////////
  ////////////////////////////////////////////////////////////////////////
  void commAndVarUpdatePhase(){};

  void updateKnowledge(){};

  bool hasEveryoneConverged() { return false; };

  void defineTaskSequence(){};

  bool first = false;
};