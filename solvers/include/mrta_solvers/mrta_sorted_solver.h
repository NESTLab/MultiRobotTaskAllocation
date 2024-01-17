#pragma once
#include <mrta_solvers/mrta_generic_solver.h>

class MrtaSortedSolver : public MrtaGenericSolver {
public:
  MrtaSortedSolver(/* args */){};
  ~MrtaSortedSolver(){};

private:
  std::shared_ptr<const MrtaSolution::CompleteSolution> const solveMrtaProblem();

  void updateMrtaConfig(const std::shared_ptr<const MrtaConfig::CompleteConfig> mrta_complete_config_in) {
    mrta_complete_config = mrta_complete_config_in;
  };

  void updateWorldStatus(){};

private:
  std::shared_ptr<const MrtaConfig::CompleteConfig> mrta_complete_config;
};