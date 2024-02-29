#pragma once
#include <memory>
#include <mrta_utilities/mrta_config.h>
#include <mrta_utilities/mrta_solution.h>

class MrtaGenericSolver {
public:
  MrtaGenericSolver(/* args */){};
  ~MrtaGenericSolver(){};

protected:
  virtual void
  solveMrtaProblem(const MrtaConfig::CompleteConfig &mrta_complete_config,
                   MrtaSolution::CompleteSolution &ret_complete_solution) = 0;

  virtual void
  updateMrtaConfig(const MrtaConfig::CompleteConfig &mrta_complete_config) = 0;

  virtual void updateWorldStatus() = 0;

  bool limited_info_mode = false;

  /**
   * @brief Set the Limited Info Mode object
   * Defines if this solver is being used by the CRA methods. If that is the
   * case, then the skill matrix calculation should be skipped and should be
   * taken as an input from the user.
   *
   * @param limited_info_mode_in
   */
  void setLimitedInfoMode(bool limited_info_mode_in) {
    limited_info_mode = limited_info_mode_in;
  }

  template <typename M_k, typename M_v>
  inline typename std::map<M_k, M_v>::const_iterator
  getMapIterator(const std::map<M_k, M_v> &const_map, const M_k &key) {
    return const_map.find(key);
  };

  friend class MrtaInterface;
};
