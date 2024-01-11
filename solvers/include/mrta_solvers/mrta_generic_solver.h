#pragma once
#include <mrta_utilities/mrta_config.h>
#include <mrta_utilities/mrta_solution.h>


class MrtaGenericSolver
{
public:
  MrtaGenericSolver(/* args */){};
  ~MrtaGenericSolver(){};

  virtual MrtaSolution::CompleteSolution solveMrtaProblem() = 0;

  virtual bool setMrtaConfig(const MrtaConfig& config_object) = 0;

  virtual bool updateWorldStatus() = 0;

  void debugPrintSolution(const MrtaSolution::CompleteSolution solution);

private:
  /* data */
};
