#pragma once
// #include "gurobi_c++.h"
#include <mrta_solvers/mrta_generic_solver.h>

class MrtaMilpSolver : public MrtaGenericSolver {
public:
  MrtaMilpSolver(/* args */){};
  ~MrtaMilpSolver(){};

private:
  void solveMrtaProblem(
      const MrtaConfig::CompleteConfig &mrta_complete_config,
      MrtaSolution::CompleteSolution &ret_complete_solution) override;

  inline void updateWorldStatus() override{};
};