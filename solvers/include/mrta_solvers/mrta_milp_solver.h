#pragma once
#include <mrta_solvers/mrta_generic_solver.h>

class MrtaMilpSolver: public MrtaGenericSolver
{
private:
  /* data */
public:
  MrtaMilpSolver(/* args */){};
  ~MrtaMilpSolver(){};

  MrtaSolution::CompleteSolution solveMrtaProblem();

  bool setMrtaConfig(const MrtaConfig& config_object) {
    return true;
  };

  bool updateWorldStatus() {
    return true;
  };
};
