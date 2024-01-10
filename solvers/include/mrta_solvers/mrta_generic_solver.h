#pragma once
#include <mrta_utilities/mrta_config.h>
#include <mrta_utilities/mrta_solution.h>


class MrtaGenericSolver
{
public:
  MrtaGenericSolver(/* args */);
  ~MrtaGenericSolver();

template<typename T>
  const T& solveMrtaProblem();

  virtual bool setMrtaConfig() = 0;

  virtual bool updateWorldStatus();

private:
  /* data */
};
