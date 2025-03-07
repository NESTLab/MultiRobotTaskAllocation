#pragma once
#include <eigen3/Eigen/Dense>
#include <mrta_solvers/mrta_decentralized_generic_solver.h>
#include <queue>
#include <unordered_map>

class MrtaDecentralizedHssSolver : public MrtaDecentralizedGenericSolver {
public:
  MrtaDecentralizedHssSolver(const std::string &robot_name, int robot_id)
      : MrtaDecentralizedGenericSolver(robot_name, robot_id){};
  ~MrtaDecentralizedHssSolver() override{};

private:
  void solveOneIteration(
      const MrtaConfig::CompleteConfig &mrta_complete_config,
      MrtaSolution::CompleteSolution &ret_complete_solution) override;

  void updateWorldStatus() override{};

  bool areAllTasksSatisfied() { return true; };

  bool checkConvergence() override { return converged; };

  void communicateSolutionToAgents(
      const MrtaSolution::CompleteSolution &complete_solution) override;

  void updateSolution(const std::vector<std::string> &curr_path_i_A,
                      MrtaSolution::CompleteSolution &ret_complete_solution);

  /////////////////////////////////////////////////////////////
  /////////  T A S K   I N C L U S I O N   P H A S E  /////////
  /////////////////////////////////////////////////////////////
  void taskInclusionPhase(std::vector<std::string> &curr_path_i_A);

  std::pair<size_t, size_t> getIndexOfPredecessorSuccessorToTask(
      const std::string &task, const std::vector<std::string> &curr_path_i_A,
      const std::vector<std::string> &ordered_tasks_i_B);

  inline std::vector<std::string>::const_iterator
  getIterToObjectInVector(const std::vector<std::string> &task_vector,
                          const std::string &task) {
    return std::find(task_vector.begin(), task_vector.end(), task);
  }

  inline int getIndexOfObjectInVector(
      const std::vector<std::string>::const_iterator &task_iter,
      const std::vector<std::string> &task_vector) {
    return task_iter - task_vector.begin();
  }

  inline int
  getIndexOfObjectInVector(const std::string &task,
                           const std::vector<std::string> &task_vector) {
    std::vector<std::string>::const_iterator task_iter =
        getIterToObjectInVector(task_vector, task);
    return getIndexOfObjectInVector(task_iter, task_vector);
  }

  inline bool
  isObjectInVector(const std::vector<std::string>::const_iterator &task_iter,
                   const std::vector<std::string> &task_vector) {
    return task_iter != task_vector.end();
  }

  inline int isObjectInVector(const std::string &task,
                              const std::vector<std::string> &task_vector) {
    std::vector<std::string>::const_iterator task_iter =
        getIterToObjectInVector(task_vector, task);
    return isObjectInVector(task_iter, task_vector);
  }

  double getCost(const std::vector<std::string> &curr_path_i_A);

  double getCostOfTravelC1(const std::vector<std::string> &curr_path_i_A,
                           bool update_solution = false);

  double getCostOfAttendanceC2(const std::vector<std::string> &curr_path_i_A);

  double
  getAfterTaskInsertionCost(const std::vector<std::string> &curr_path_i_A,
                            const std::string &task, size_t index);

  size_t getTaskInsertionIndex(const std::vector<std::string> &curr_path_i_A,
                               const std::string &task);

  std::vector<std::string> ordered_tasks_i_B;
  std::priority_queue<std::pair<std::string, double>,
                      std::vector<std::pair<std::string, double>>,
                      MrtaConfig::CompareSecond>
      temp_ordered_tasks_set_i_B;

  std::unordered_map<std::string, bool> robot_unnecessary_at_task_i_u_j;

  const double LARGE_COST = 100000;

  ////////////////////////////////////////////////////////////////////////
  /////////  C O M M   A N D   V A R   U P D A T E   P H A S E   /////////
  ////////////////////////////////////////////////////////////////////////
  void commAndVarUpdatePhase(
      const MrtaSolution::CompleteSolution &complete_solution);

  void defineUnnecessaryRobots(
      const MrtaSolution::CompleteSolution &complete_solution);

  void getRobotsQueueAttendingTask(
      std::priority_queue<std::pair<std::string, double>,
                          std::vector<std::pair<std::string, double>>,
                          MrtaConfig::CompareSecond>
          &robot_arrival_time_queue_j_I,
      const std::string &task,
      const MrtaSolution::CompleteSolution &complete_solution);

  void updateKnowledge(){};

  bool hasOverallScheduleSettled(
      const MrtaSolution::CompleteSolution &complete_solution);

  void
  defineTaskSequence(const MrtaSolution::CompleteSolution &complete_solution);

  int isMapSubsetOfSet(const std::map<std::string, double> &source_map,
                       const std::set<std::string> &ref_set);

  void addMapKeysToSet(const std::map<std::string, double> &source_map,
                       std::set<std::string> &ref_set);

  double getScheduleDifferenceInThisTimestep(
      const MrtaSolution::CompleteSolution &complete_solution);

  bool first = false;

  const double ARRIVAL_TIME_CHANGE_THRESHOLD_Y = 0.5;

  Eigen::MatrixXd last_timestep_robot_arrival_times_at_tasks;
  Eigen::MatrixXd robot_arrival_times_at_tasks;
};