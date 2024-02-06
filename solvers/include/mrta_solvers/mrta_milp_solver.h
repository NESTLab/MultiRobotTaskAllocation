#pragma once
#include "gurobi_c++.h"
#include <mrta_solvers/mrta_generic_solver.h>

class MrtaMilpSolver : public MrtaGenericSolver {
public:
  MrtaMilpSolver(/* args */){};
  ~MrtaMilpSolver(){};

  void solveMrtaProblem(const MrtaConfig::CompleteConfig &mrta_complete_config,
                        MrtaSolution::CompleteSolution &ret_complete_solution);

  bool setMrtaConfig(const MrtaConfig &config_object_in) {
    config_object = &config_object_in;
    return true;
  };

  bool updateWorldStatus() { return true; };

private:
  MrtaConfig const *config_object;
};
