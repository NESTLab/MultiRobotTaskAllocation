#pragma once
#include <mrta_solvers/mrta_decentralized_generic_solver.h>

class MrtaDecentralizedHssSolver : public MrtaDecentralizedGenericSolver {
public:
  MrtaDecentralizedHssSolver(const std::string &robot_name){};
  ~MrtaDecentralizedHssSolver(){};

private:
  void
  updateMrtaConfig(const MrtaConfig::CompleteConfig &mrta_complete_config_in) {
    mrta_complete_config = &mrta_complete_config_in;
  };

  bool solveOneIteration(
      const MrtaConfig::CompleteConfig &mrta_complete_config,
      MrtaSolution::CompleteSolution &ret_complete_solution) override;

  void updateWorldStatus() override{};

  bool areAllTasksSatisfied(){return false;};

  bool checkConvergence(){return false;};

  /////////////////////////////////////////////////////////////
  /////////  T A S K   I N C L U S I O N   P H A S E  /////////
  /////////////////////////////////////////////////////////////
  void taskInclusionPhase();

  std::pair<size_t, size_t>
  getIndexOfPredecessorSuccessorToTask(const std::string &task);

  double getCost(const std::vector<std::string> &curr_path_i_A);

  double
  getAfterTaskInsertionCost(const std::vector<std::string> &curr_path_i_A,
                            const std::string &task, size_t index);

  size_t getTaskInsertionIndex(const std::vector<std::string> &curr_path_i_A,
                               const std::string &task,
                               std::pair<size_t, size_t> index_of_pred_succ);

  ////////////////////////////////////////////////////////////////////////
  /////////  C O M M   A N D   V A R   U P D A T E   P H A S E   /////////
  ////////////////////////////////////////////////////////////////////////
  void commAndVarUpdatePhase(){};

  void updateKnowledge(){};

  bool hasEveryoneConverged(){return false;};

  void defineTaskSequence(){};

  bool first = false;
};