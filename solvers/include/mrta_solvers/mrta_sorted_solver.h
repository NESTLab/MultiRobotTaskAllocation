#pragma once
#include <mrta_solvers/mrta_generic_solver.h>

class MrtaSortedSolver : public MrtaGenericSolver {
public:
  MrtaSortedSolver(/* args */){};
  ~MrtaSortedSolver(){};

private:
  void solveMrtaProblem(const MrtaConfig::CompleteConfig &mrta_complete_config,
                        MrtaSolution::CompleteSolution &ret_complete_solution);

  void updateMrtaConfig(const MrtaConfig::CompleteConfig& mrta_complete_config_in) {
    mrta_complete_config = &mrta_complete_config_in;
  };

  void updateWorldStatus(){};

private:
  MrtaConfig::CompleteConfig const * mrta_complete_config;
};