#include <limits>
#include <mrta_solvers/mrta_heuristic_solver.h>

std::shared_ptr<const MrtaSolution::CompleteSolution> const
MrtaHeuristicSolver::solveMrtaProblem() {

  // TESTING TESTING TESTING
  HEURISTIC_SOLVER_CONFIG solver_config;
  solver_config.phase_i_config.selection_list_config =
      SELECTION_LIST::MAX_SKILL;
  solver_config.phase_i_config.choosing_robot_task_pair_config =
      CHOOSE_R_T_PAIR::NEAREST_PAIR;
  limited_info_mode = false;
  setHeuristicSolverConfig(solver_config);

  if (limited_info_mode)
    healthCheckSolverInfo();
  else
    updateContributionsFromConfig();

  // Phase 1: Select a pair of robot and tasks
  std::pair<int, int> selected_robot_task_pair;
  getSelectedRobotTaskPair(selected_robot_task_pair);

  // Phase 2: Select robots until the task satisfied

  // Delete superfluous robots
  std::shared_ptr<const MrtaSolution::CompleteSolution> test;
  return test;
}

void MrtaHeuristicSolver::initializeDistanceTensor() {
  int number_of_robots = mrta_complete_config->setup.number_of_robots;
  int number_of_destinations =
      mrta_complete_config->setup.number_of_destinations;
  robot_distances_vector.resize(number_of_robots);
  for (size_t i = 0; i < number_of_robots; i++) {
    robot_distances_vector.at(i) =
        Eigen::MatrixXd::Zero(number_of_destinations, number_of_destinations);
    putDistancesForRobot(i, robot_distances_vector.at(i));
    debugPrintContributionArray(robot_distances_vector.at(i));
  }
}

void MrtaHeuristicSolver::putDistancesForRobot(
    int robot_id, Eigen::MatrixXd &ret_i_distance_matrix) {
  int number_of_destinations =
      mrta_complete_config->setup.number_of_destinations;

  // FIRST and LAST destinations are START and END, hence skipping them
  for (size_t task_id = START_ID + 1; task_id < number_of_destinations - 1;
       task_id++) {
    std::map<std::string, MrtaConfig::Robot>::const_iterator robot_itr =
        mrta_complete_config->robots_map.find(
            mrta_complete_config->setup.all_robot_names.at(robot_id));
    std::map<std::string, MrtaConfig::Task>::const_iterator task_itr =
        mrta_complete_config->tasks_map.find(
            mrta_complete_config->setup.all_destination_names.at(task_id));

    ret_i_distance_matrix(START_ID, task_id) =
        getPureDistance(robot_itr->second.position, task_itr->second.position);
    ret_i_distance_matrix(END_ID, task_id) = getPureDistance(
        robot_itr->second.desired_end_position, task_itr->second.position);
    ret_i_distance_matrix(task_id, START_ID) =
        ret_i_distance_matrix(START_ID, task_id);
    ret_i_distance_matrix(task_id, END_ID) =
        ret_i_distance_matrix(END_ID, task_id);
    ret_i_distance_matrix(START_ID, START_ID) =
        std::numeric_limits<double>::infinity();
    ret_i_distance_matrix(END_ID, END_ID) =
        std::numeric_limits<double>::infinity();
    ret_i_distance_matrix(task_id, task_id) =
        std::numeric_limits<double>::infinity();
    for (size_t second_task_id = task_id + 1;
         second_task_id < number_of_destinations - 1; second_task_id++) {
      std::map<std::string, MrtaConfig::Task>::const_iterator second_task_itr =
          mrta_complete_config->tasks_map.find(
              mrta_complete_config->setup.all_destination_names.at(
                  second_task_id));
      ret_i_distance_matrix(task_id, second_task_id) = getPureDistance(
          task_itr->second.position, second_task_itr->second.position);
      ret_i_distance_matrix(second_task_id, task_id) =
          ret_i_distance_matrix(task_id, second_task_id);
    }
  }
}

double MrtaHeuristicSolver::getPureDistance(
    const MrtaConfig::Position &task_1_position,
    const MrtaConfig::Position &task_2_position) {
  double x_diff = task_1_position.pos_x - task_2_position.pos_x;
  double y_diff = task_1_position.pos_y - task_2_position.pos_y;
  return std::sqrt(std::abs(x_diff * x_diff + y_diff * y_diff));
}

void MrtaHeuristicSolver::getSelectedRobotTaskPair(
    std::pair<int, int> &ret_selected_robot_task_pair) {

  std::vector<std::pair<int, int>> candidate_robot_task_id_pairs;
  getCandidateRobotTaskPairs(candidate_robot_task_id_pairs);

  getChosenRobotTaskPair(candidate_robot_task_id_pairs,
                         ret_selected_robot_task_pair);
}

void MrtaHeuristicSolver::getCandidateRobotTaskPairs(
    std::vector<std::pair<int, int>> &ret_candidate_robot_task_pairs) {
  if (heuristic_method_config.phase_i_config.selection_list_config ==
      SELECTION_LIST::ONE_SKILL)
    getRobotTaskPairsWithMinOneContributions(ret_candidate_robot_task_pairs);
  else if (heuristic_method_config.phase_i_config.selection_list_config ==
           SELECTION_LIST::MAX_SKILL)
    getRobotTaskPairsWithMaxContributions(ret_candidate_robot_task_pairs);
  else
    throw std::invalid_argument("UNKNOWN selection priority set");
}

void MrtaHeuristicSolver::getRobotTaskPairsWithMinOneContributions(
    std::vector<std::pair<int, int>> &ret_candidate_robot_task_pairs) {
  double threshold = 1.0;
  getRequiredRobotTaskWithContributionsAboveThreshold(
      threshold, ret_candidate_robot_task_pairs);
}

void MrtaHeuristicSolver::getRobotTaskPairsWithMaxContributions(
    std::vector<std::pair<int, int>> &ret_candidate_robot_task_pairs) {
  double max_contribution = contribution_array.maxCoeff();
  getRequiredRobotTaskWithContributionsAboveThreshold(
      max_contribution, ret_candidate_robot_task_pairs);
}

void MrtaHeuristicSolver::getRequiredRobotTaskWithContributionsAboveThreshold(
    double threshold,
    std::vector<std::pair<int, int>> &ret_candidate_robot_task_pairs) {
  Eigen::MatrixXi mask = contribution_array.array().unaryExpr(
      [threshold](double val) { return val >= threshold ? 1 : 0; });
  for (int i = 0; i < mask.rows(); ++i) {
    for (int j = 0; j < mask.cols(); ++j) {
      if (mask(i, j)) {
        ret_candidate_robot_task_pairs.push_back(std::make_pair(i, j));
      }
    }
  }
}

void MrtaHeuristicSolver::getChosenRobotTaskPair(
    const std::vector<std::pair<int, int>> &candidate_robot_task_id_pairs,
    std::pair<int, int> &ret_chosen_robot_task_pair) {
  if (heuristic_method_config.phase_i_config.choosing_robot_task_pair_config ==
      CHOOSE_R_T_PAIR::SOONEST_PAIR)
    getSoonestRobotTaskPair(candidate_robot_task_id_pairs,
                            ret_chosen_robot_task_pair);
  else if (heuristic_method_config.phase_i_config
               .choosing_robot_task_pair_config ==
           CHOOSE_R_T_PAIR::NEAREST_PAIR)
    getNearestRobotTaskPair(candidate_robot_task_id_pairs,
                            ret_chosen_robot_task_pair);
  else
    throw std::invalid_argument("UNKNOWN selection priority set");
}

void MrtaHeuristicSolver::getSoonestRobotTaskPair(
    const std::vector<std::pair<int, int>> &candidate_robot_task_id_pairs,
    std::pair<int, int> &ret_chosen_robot_task_pair) {}

void MrtaHeuristicSolver::getNearestRobotTaskPair(
    const std::vector<std::pair<int, int>> &candidate_robot_task_id_pairs,
    std::pair<int, int> &ret_chosen_robot_task_pair) {}

void MrtaHeuristicSolver::debugPrintContributionArray(
    const Eigen::MatrixXd matrix) {
  std::cout << "CONTRIBUTION ARRAY IS:" << std::endl;
  for (int i = 0; i < matrix.rows(); ++i) {
    for (int j = 0; j < matrix.cols(); ++j) {
      std::cout << matrix(i, j) << " - ";
    }
    std::cout << std::endl;
  }
}

void MrtaHeuristicSolver::updateContributionsFromConfig() {
  int number_of_robots = mrta_complete_config->setup.number_of_robots;
  int number_of_destinations =
      mrta_complete_config->setup.number_of_destinations;
  contribution_array =
      Eigen::MatrixXd::Zero(number_of_robots, number_of_destinations);
  for (int i = 0; i < number_of_robots; ++i) {
    for (int j = START_ID + 1;           // Skipping the START task
         j < number_of_destinations - 1; // Skipping the END task
         ++j) {
      for (int s = 0; s < mrta_complete_config->setup.number_of_skills; ++s) {
        const std::string &robot_name =
            mrta_complete_config->setup.all_robot_names.at(i);
        const std::string &task_name =
            mrta_complete_config->setup.all_destination_names.at(j);
        const std::string &skill_name =
            mrta_complete_config->setup.all_skill_names.at(s);

        std::map<std::string, MrtaConfig::Robot>::const_iterator
            robot_info_itr = mrta_complete_config->robots_map.find(robot_name);
        std::map<std::string, MrtaConfig::Task>::const_iterator task_info_itr =
            mrta_complete_config->tasks_map.find(task_name);

        std::map<std::string, double>::const_iterator robot_skill_itr =
            robot_info_itr->second.skillset.find(skill_name);
        std::map<std::string, double>::const_iterator task_skill_itr =
            task_info_itr->second.skillset.find(skill_name);

        if (robot_skill_itr->second >= task_skill_itr->second)
          ++contribution_array(i, j);
      }
    }
  }
  std::cout << "=================================================="
            << std::endl;
  debugPrintContributionArray(contribution_array);
  std::cout << "=================================================="
            << std::endl;
}
