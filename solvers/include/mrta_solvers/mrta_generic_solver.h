#pragma once
#include <mrta_utilities/mrta_config.h>
#include <mrta_utilities/mrta_solution.h>
#include <memory>


class MrtaGenericSolver
{
public:
  MrtaGenericSolver(/* args */){};
  ~MrtaGenericSolver(){};

  virtual std::shared_ptr<MrtaSolution::CompleteSolution> solveMrtaProblem() = 0;

  virtual void setMrtaConfig(const MrtaConfig::CompleteConfig &mrta_complete_config) = 0;

  virtual void updateWorldStatus() = 0;

private:
  /* data */
};
