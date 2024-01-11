#pragma once
#include "gurobi_c++.h"
#include <mrta_solvers/mrta_generic_solver.h>

class MrtaMilpSolver : public MrtaGenericSolver {
public:
  MrtaMilpSolver(/* args */){};
  ~MrtaMilpSolver(){};

  std::shared_ptr<MrtaSolution::CompleteSolution> solveMrtaProblem();

  bool setMrtaConfig(const MrtaConfig &config_object_in) {
    config_object = &config_object_in;
    return true;
  };

  bool updateWorldStatus() { return true; };

private:
  MrtaConfig const *config_object;
};
