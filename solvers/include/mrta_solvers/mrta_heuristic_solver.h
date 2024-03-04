#include <mrta_solvers/mrta_generic_solver.h>
#include <mrta_utilities/mrta_config.h>

class MrtaHeuristicSolver : public MrtaGenericSolver {
public:
  MrtaHeuristicSolver(/* args */){};
  ~MrtaHeuristicSolver(){};

  enum SELECTION_LIST { ONE_SKILL, MAX_SKILL };
  enum CHOOSE_R_T_PAIR { SOONEST_PAIR, NEAREST_PAIR, LEAST_COST_WITH_RHO };
  enum DELETE_SUPERFLUOUS { EARLIEST_ROBOT, LATEST_ROBOT };
  struct PHASE_I_CONFIG {
    SELECTION_LIST selection_list_config;
    CHOOSE_R_T_PAIR choosing_robot_task_pair_config;
  };
  struct PHASE_II_CONFIG {
    SELECTION_LIST selection_list_config;
    CHOOSE_R_T_PAIR choosing_robot_task_pair_config;
  };
  struct HEURISTIC_SOLVER_CONFIG {
    PHASE_I_CONFIG phase_i_config;
    PHASE_II_CONFIG phase_ii_config;
    DELETE_SUPERFLUOUS delete_superfluous;
  };

  inline void
  setHeuristicSolverConfig(const HEURISTIC_SOLVER_CONFIG &req_config) {
    heuristic_method_config = req_config;
  }

private:
  HEURISTIC_SOLVER_CONFIG heuristic_method_config;
  void solveMrtaProblem(const MrtaConfig::CompleteConfig &mrta_complete_config,
                        MrtaSolution::CompleteSolution &ret_complete_solution);

  Eigen::MatrixXd contribution_array;


  void updateWorldStatus(){};

  void updateContributionsFromConfig();

  /**
   * @brief If the solver is in limited information mode, then there needs to be
   * a way to make sure that all the necessary information has been supplied.
   * This function will make sure that the solver has all the necessary
   * information required.
   *
   */
  void healthCheckSolverInfo(){};

  /**
   * @brief Is responsible to setup the things required by the class
   *        Example of things to setup:
   *            - Initialize matrices dimentions
   *            - Distance matrix
   *            - Contribution matrix
   *
   */
  void setupEnvironment();

  void
  getSelectedRobotTaskPair(std::pair<int, int> &ret_selected_robot_task_pair);

  void getRobotTaskPairsWithMinOneContributions(
      std::vector<std::pair<int, int>> &ret_candidate_robot_task_pairs);

  void getRobotTaskPairsWithMaxContributions(
      std::vector<std::pair<int, int>> &ret_candidate_robot_task_pairs);

  void getRequiredRobotTaskWithContributionsAboveThreshold(
      double threshold,
      std::vector<std::pair<int, int>> &ret_candidate_robot_task_pairs);

  void getCandidateRobotTaskPairs(
      std::vector<std::pair<int, int>> &ret_candidate_robot_task_pairs);

  void getChosenRobotTaskPair(
      const std::vector<std::pair<int, int>> &candidate_robot_task_id_pairs,
      std::pair<int, int> &ret_chosen_robot_task_pair);

  void getSoonestRobotTaskPair(
      const std::vector<std::pair<int, int>> &candidate_robot_task_id_pairs,
      std::pair<int, int> &ret_chosen_robot_task_pair);

  void getNearestRobotTaskPair(
      const std::vector<std::pair<int, int>> &candidate_robot_task_id_pairs,
      std::pair<int, int> &ret_chosen_robot_task_pair);

  void debugPrintContributionArray(const Eigen::MatrixXd matrix);

  void assignTaskToRobot(const MrtaConfig::CompleteConfig &mrta_complete_config,
                         MrtaSolution::CompleteSolution &ret_complete_solution,
                         int robot_id, int task_id);

  int getRobotToBeAddedToCoalition(int task_id);
  void getThresholdCrossingRobotsForCoalition(
      int task_id, std::vector<int> &threshold_crossing_robots_vector);
  void getRobotsMeetingExpectedThresholds(
      int task_id, double threshold,
      std::vector<int> &threshold_crossing_robots_vector);

  int pickRobotForCoalition(int task_id,
                            std::vector<int> &threshold_crossing_robots_vector);

  int getEarliestArrivingRobot(
      int task_id, const std::vector<int> &threshold_crossing_robots_vector);

  int getClosestRobotToTask(
      int task_id, const std::vector<int> &threshold_crossing_robots_vector);
};