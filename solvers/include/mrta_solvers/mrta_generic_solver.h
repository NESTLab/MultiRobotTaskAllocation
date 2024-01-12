#pragma once
#include <mrta_utilities/mrta_config.h>
#include <mrta_utilities/mrta_solution.h>
#include <memory>


class MrtaGenericSolver
{
public:
  MrtaGenericSolver(/* args */){};
  ~MrtaGenericSolver(){};

private:
  virtual std::shared_ptr<const MrtaSolution::CompleteSolution> const solveMrtaProblem() = 0;

  virtual void updateMrtaConfig(const std::shared_ptr<const MrtaConfig::CompleteConfig>mrta_complete_config) = 0;

  virtual void updateWorldStatus() = 0;

friend class MrtaInterface;
};
