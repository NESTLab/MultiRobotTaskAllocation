#include <limits>
#include <mrta_solvers/mrta_heuristic_solver.h>

void MrtaHeuristicSolver::solveMrtaProblem(
    const MrtaConfig::CompleteConfig &mrta_complete_config,
    MrtaSolution::CompleteSolution &ret_complete_solution) {
  // TESTING TESTING TESTING
  HEURISTIC_SOLVER_CONFIG solver_config;
  solver_config.phase_i_config.selection_list_config =
      SELECTION_LIST::ONE_SKILL;
  solver_config.phase_i_config.choosing_robot_task_pair_config =
      CHOOSE_R_T_PAIR::NEAREST_PAIR;
  solver_config.phase_ii_config.selection_list_config =
      SELECTION_LIST::MAX_SKILL;
  solver_config.phase_ii_config.choosing_robot_task_pair_config =
      CHOOSE_R_T_PAIR::SOONEST_PAIR;
  solver_config.delete_superfluous = DELETE_SUPERFLUOUS::LATEST_ROBOT;
  setHeuristicSolverConfig(solver_config);

  if (limited_info_mode)
    healthCheckSolverInfo();
  else
    updateContributionsFromConfig();
  /////////////////////////////////////////////////////////
  /////// Phase 1: Select a pair of robot and tasks ///////
  /////////////////////////////////////////////////////////
  while (contribution_array.any()) {
    std::pair<int, int> selected_robot_task_pair;
    getSelectedRobotTaskPair(selected_robot_task_pair);
    assignTaskToRobot(mrta_complete_config, ret_complete_solution,
                      selected_robot_task_pair.first,
                      selected_robot_task_pair.second);

    updateContributionsFromConfig();
    std::vector<int> robots_in_coalition;
    robots_in_coalition.push_back(selected_robot_task_pair.first);

    /////////////////////////////////////////////////////////
    /////// Phase 2: Select a pair of robot and tasks ///////
    /////////////////////////////////////////////////////////
    int task_id = selected_robot_task_pair.second;
    while (contribution_array(Eigen::all, task_id).any()) {
      int selected_robot = getRobotToBeAddedToCoalition(task_id);
      assignTaskToRobot(mrta_complete_config, ret_complete_solution,
                        selected_robot, task_id);
      updateContributionsFromConfig();
      robots_in_coalition.push_back(selected_robot);

      /////////////////////////////////////////////////////////
      ///////////// Phase 3: Refine the coalition /////////////
      /////////////////////////////////////////////////////////
      /**TODO**/
    }
    // Delete superfluous robots

    // Set the task start times
    double task_start_time = 0.0;
    for (int robot_id : robots_in_coalition) {
      const std::string &robot_name =
          mrta_complete_config.setup.all_robot_names.at(robot_id);
      const std::string &task_name =
          mrta_complete_config.setup.all_destination_names.at(task_id);

      if (task_start_time <
          robot_task_attendance_times_map[robot_name][task_name])
        task_start_time =
            robot_task_attendance_times_map[robot_name][task_name];
    }
    for (int robot_id : robots_in_coalition) {
      const std::string &robot_name =
          mrta_complete_config.setup.all_robot_names.at(robot_id);
      const std::string &task_name =
          mrta_complete_config.setup.all_destination_names.at(task_id);

      robot_task_attendance_times_map[robot_name][task_name] = task_start_time;
    }
  }

  for (int i = 0; i < mrta_complete_config.setup.number_of_robots; i++) {
    assignTaskToRobot(mrta_complete_config, ret_complete_solution, i, END_ID);
  }
}

void MrtaHeuristicSolver::setTaskStartTimes() {}

void MrtaHeuristicSolver::assignTaskToRobot(
    const MrtaConfig::CompleteConfig &mrta_complete_config,
    MrtaSolution::CompleteSolution &ret_complete_solution, int robot_id,
    int task_id) {

  const std::string &robot_name =
      mrta_complete_config.setup.all_robot_names.at(robot_id);
  const std::string &task_name =
      mrta_complete_config.setup.all_destination_names.at(task_id);

  ret_complete_solution.robot_task_schedule_map[robot_name]
      .task_attendance_sequence.push_back(task_name);

  int last_task_id = robot_task_id_attendance_sequence.at(robot_id).back();
  const std::string &last_task_name =
      mrta_complete_config.setup.all_destination_names.at(last_task_id);

  robot_task_id_attendance_sequence.at(robot_id).push_back(task_id);

  std::map<std::string, MrtaConfig::Robot>::const_iterator robot_itr =
      mrta_complete_config.robots_map.find(
          mrta_complete_config.setup.all_robot_names.at(robot_id));

  std::map<std::string, MrtaConfig::Task>::const_iterator task_itr =
      mrta_complete_config.tasks_map.find(
          mrta_complete_config.setup.all_destination_names.at(task_id));

  if (robot_itr != mrta_complete_config.robots_map.end()) {
    for (int skill_id = 0;
         skill_id < mrta_complete_config.setup.all_skill_names.size();
         skill_id++) {
      std::map<std::string, double>::const_iterator robot_skill_itr =
          robot_itr->second.skillset.find(
              mrta_complete_config.setup.all_skill_names.at(skill_id));
      if (robot_skill_itr->second > 0.0 &&
          robot_skill_itr->second >=
              task_requirements_matrix(task_id, skill_id) &&
          task_requirements_matrix(task_id, skill_id) > 0.0) {
        task_requirements_matrix(task_id, skill_id) = 0.0;
      }
    }
  }

  double last_task_attendance_time =
      robot_task_attendance_times_map[robot_name][last_task_name];

  double attendance_time =
      last_task_attendance_time + task_itr->second.duration +
      robot_distances_vector.at(robot_id)(last_task_id, task_id);

  ret_complete_solution.robot_task_schedule_map[robot_name]
      .task_arrival_time_map[task_name] = attendance_time;
  robot_task_attendance_times_map[robot_name][task_name] = attendance_time;
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
  Eigen::MatrixXi mask =
      contribution_array.array().unaryExpr([threshold](double val) {
        return (val >= threshold && val > 0.0) ? 1 : 0;
      });
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
    std::pair<int, int> &ret_chosen_robot_task_pair) {
  double min_arrival_time = std::numeric_limits<double>::infinity();
  for (const auto &robot_task_id_pair : candidate_robot_task_id_pairs) {
    int robot_id = robot_task_id_pair.first;
    int task_id = robot_task_id_pair.second;
    int last_task_id = robot_task_id_attendance_sequence.at(robot_id).back();
    double travel_dist =
        robot_distances_vector.at(robot_id).coeff(last_task_id, task_id);
    double last_task_start_time = task_start_time.at(last_task_id);
    const std::string &last_task_name =
        mrta_complete_config->setup.all_destination_names.at(task_id);
    std::map<std::string, MrtaConfig::Task>::const_iterator task_itr =
        mrta_complete_config->tasks_map.find(last_task_name);
    double last_task_exec_time = task_itr->second.duration;
    double arrival_time =
        last_task_start_time + last_task_exec_time + travel_dist;

    if (arrival_time < min_arrival_time) {
      min_arrival_time = arrival_time;
      ret_chosen_robot_task_pair.first = robot_id;
      ret_chosen_robot_task_pair.second = task_id;
    }
  }
}

void MrtaHeuristicSolver::getNearestRobotTaskPair(
    const std::vector<std::pair<int, int>> &candidate_robot_task_id_pairs,
    std::pair<int, int> &ret_chosen_robot_task_pair) {
  double min_travel_dist = std::numeric_limits<double>::infinity();
  for (const auto &robot_task_id_pair : candidate_robot_task_id_pairs) {
    int robot_id = robot_task_id_pair.first;
    int task_id = robot_task_id_pair.second;
    int last_task_id = robot_task_id_attendance_sequence.at(robot_id).back();
    double travel_dist =
        robot_distances_vector.at(robot_id).coeff(last_task_id, task_id);
    if (travel_dist < min_travel_dist) {
      min_travel_dist = travel_dist;
      ret_chosen_robot_task_pair.first = robot_id;
      ret_chosen_robot_task_pair.second = task_id;
    }
  }
}

void MrtaHeuristicSolver::debugPrintContributionArray(
    const Eigen::MatrixXd matrix) {
  for (int i = 0; i < matrix.rows(); ++i) {
    for (int j = 0; j < matrix.cols(); ++j) {
    }
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

        std::map<std::string, double>::const_iterator robot_skill_itr =
            robot_info_itr->second.skillset.find(skill_name);

        if (robot_skill_itr->second >= task_requirements_matrix(j, s) &&
            task_requirements_matrix(j, s) > 0.0)
          ++contribution_array(i, j);
      }
    }
  }
}

int MrtaHeuristicSolver::getRobotToBeAddedToCoalition(int task_id) {
  std::vector<int> threshold_crossing_robots_vector;
  getThresholdCrossingRobotsForCoalition(task_id,
                                         threshold_crossing_robots_vector);
  return pickRobotForCoalition(task_id, threshold_crossing_robots_vector);
}

void MrtaHeuristicSolver::getThresholdCrossingRobotsForCoalition(
    int task_id, std::vector<int> &threshold_crossing_robots_vector) {
  double threshold = 1;
  if (heuristic_method_config.phase_ii_config.selection_list_config ==
      SELECTION_LIST::ONE_SKILL)
    threshold = 1;
  else if (heuristic_method_config.phase_ii_config.selection_list_config ==
           SELECTION_LIST::MAX_SKILL)
    threshold = contribution_array(Eigen::all, task_id).maxCoeff();
  else
    throw std::invalid_argument("UNKNOWN selection priority set");
  getRobotsMeetingExpectedThresholds(task_id, threshold,
                                     threshold_crossing_robots_vector);
}

void MrtaHeuristicSolver::getRobotsMeetingExpectedThresholds(
    int task_id, double threshold,
    std::vector<int> &threshold_crossing_robots_vector) {
  for (int i = 0; i < contribution_array.rows(); ++i) {
    if (contribution_array(i, task_id) >= threshold) {
      threshold_crossing_robots_vector.push_back(i);
    }
  }
}

int MrtaHeuristicSolver::pickRobotForCoalition(
    int task_id, std::vector<int> &threshold_crossing_robots_vector) {
  int robot_id = 0;
  if (heuristic_method_config.phase_ii_config.choosing_robot_task_pair_config ==
      CHOOSE_R_T_PAIR::SOONEST_PAIR)
    robot_id =
        getEarliestArrivingRobot(task_id, threshold_crossing_robots_vector);
  else if (heuristic_method_config.phase_ii_config
               .choosing_robot_task_pair_config ==
           CHOOSE_R_T_PAIR::NEAREST_PAIR)
    robot_id = getClosestRobotToTask(task_id, threshold_crossing_robots_vector);
  else
    throw std::invalid_argument("UNKNOWN selection priority set");
  return robot_id;
}

int MrtaHeuristicSolver::getEarliestArrivingRobot(
    int task_id, const std::vector<int> &threshold_crossing_robots_vector) {
  int picked_robot_id;
  double min_arrival_time = std::numeric_limits<double>::infinity();
  for (const auto &robot_id : threshold_crossing_robots_vector) {
    int last_task_id = robot_task_id_attendance_sequence.at(robot_id).back();
    double travel_dist =
        robot_distances_vector.at(robot_id).coeff(last_task_id, task_id);
    double last_task_start_time = task_start_time.at(last_task_id);
    const std::string &last_task_name =
        mrta_complete_config->setup.all_destination_names.at(task_id);
    std::map<std::string, MrtaConfig::Task>::const_iterator task_itr =
        mrta_complete_config->tasks_map.find(last_task_name);
    double last_task_exec_time = task_itr->second.duration;
    double arrival_time =
        last_task_start_time + last_task_exec_time + travel_dist;
    if (arrival_time < min_arrival_time) {
      min_arrival_time = arrival_time;
      picked_robot_id = robot_id;
    }
  }
  return picked_robot_id;
}

int MrtaHeuristicSolver::getClosestRobotToTask(
    int task_id, const std::vector<int> &threshold_crossing_robots_vector) {
  int picked_robot_id;
  double min_travel_dist = std::numeric_limits<double>::infinity();
  for (const auto &robot_id : threshold_crossing_robots_vector) {
    int last_task_id = robot_task_id_attendance_sequence.at(robot_id).back();
    double travel_dist =
        robot_distances_vector.at(robot_id).coeff(last_task_id, task_id);
    if (travel_dist < min_travel_dist) {
      min_travel_dist = travel_dist;
      picked_robot_id = robot_id;
    }
  }
  return picked_robot_id;
}
