#pragma once
#include <stdio.h>
#include <mrta_utilities/mrta_config.h>
#include <mrta_utilities/mrta_solution.h>
#include <mrta_solvers/mrta_generic_solver.h>
#include <memory>

class MrtaInterface
{
private:
  /* data */
public:
  MrtaInterface(const MrtaConfig& mrta_setup_config);
  ~MrtaInterface();
  
  // Some solvers may require something specific which may or may not have been 
  // provided by the user. Hence return 'false' if initialization failed.
  bool setMrtaSolverMethod(std::shared_ptr<MrtaGenericSolver> solver);

  // To check and print if all the essential fields are present in the config 
  // Return false if mandatory field is missing
  // Also print which optional fields are provided and which are missing
  bool healthCheckConfig();

  // This is strictly to be used by DECENTRALIZED solvers. 
  // This will update what tasks have been already attended, which robot is
  //    attending which task at what time, etc information
  // Currently simply giving it the schedule of all the robots, but it can be 
  //    done in a more sophesticated ways too
  bool updateWorldStatus(const std::map<std::string, MrtaSolution::RobotTasksSchedule>& all_robot_schedules);

  // Returns the solution
  // However, if you have set any of the DECENTRALIZED solvers, 
  // This will return the result of ONE iteration. 
  // In that case, communicating data with other robots, updating the world status
  //    and keeping track of convergence will be up to the user.
  // The return template will be as follows:
  //      Using centralized method   => MrtaSolution::CompleteSolution
  //      Using decentralized method => MrtaSolution::SingleRobotSolution
template<typename T>
  const T& solveMrtaProblem();

private:
};
