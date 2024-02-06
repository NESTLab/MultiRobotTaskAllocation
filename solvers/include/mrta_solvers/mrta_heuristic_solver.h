#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/CXX11/Tensor>
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
  int START_ID = 0;
  int END_ID = 0;
  HEURISTIC_SOLVER_CONFIG heuristic_method_config;
  void solveMrtaProblem(const MrtaConfig::CompleteConfig &mrta_complete_config,
                        MrtaSolution::CompleteSolution &ret_complete_solution);

  void
  updateMrtaConfig(const MrtaConfig::CompleteConfig &mrta_complete_config_in) {
    config_initialized = true;
    mrta_complete_config = &mrta_complete_config_in;
    END_ID = mrta_complete_config->setup.number_of_destinations - 1;
    robot_task_id_attendance_sequence.resize(
        mrta_complete_config->setup.number_of_robots);
    for (auto &robot_task_att : robot_task_id_attendance_sequence) {
      robot_task_att = std::vector<int>(1, START_ID);
    }
    initializeDistanceTensor();
    task_start_time = std::vector<double>(
        mrta_complete_config->setup.number_of_destinations, 0.0);
    local_tasks_map = mrta_complete_config->tasks_map;
  };

  bool config_initialized = false;
  MrtaConfig::CompleteConfig const * mrta_complete_config;

  Eigen::MatrixXd contribution_array;
  std::vector<Eigen::MatrixXd> robot_distances_vector;

  std::vector<std::vector<int>> robot_task_id_attendance_sequence;
  std::vector<double> task_start_time;
  std::map<std::string, MrtaConfig::Task> local_tasks_map;

  void updateWorldStatus(){};

  void initializeDistanceTensor();

  void putDistancesForRobot(int robot_id,
                            Eigen::MatrixXd &ret_i_distance_matrix);
  double getPureDistance(const MrtaConfig::Position &task_1_position,
                         const MrtaConfig::Position &task_2_position);

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
};