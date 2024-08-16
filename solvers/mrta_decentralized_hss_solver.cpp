#include <mrta_solvers/mrta_decentralized_hss_solver.h>

void MrtaDecentralizedHssSolver::solveOneIteration(
    const MrtaConfig::CompleteConfig &mrta_complete_config,
    MrtaSolution::CompleteSolution &ret_complete_solution) {
  std::vector<std::string> curr_path_i_A = {"START", "END"};
  commAndVarUpdatePhase(ret_complete_solution);
  taskInclusionPhase(curr_path_i_A);
  if ((schedule_convergence_timer > SCHEDULE_STABLE_FOR_TIMESTEPS_y_T) &&
      areAllTasksSatisfied()) {
    converged = true;
  } else
    converged = false;

  updateSolution(curr_path_i_A, ret_complete_solution);
}

void MrtaDecentralizedHssSolver::updateSolution(
    const std::vector<std::string> &curr_path_i_A,
    MrtaSolution::CompleteSolution &ret_complete_solution) {
  double max_robot_schedule = getCostOfTravelC1(curr_path_i_A, true);
  ret_complete_solution.solution_quality.result_status = 0;
  ret_complete_solution.solution_quality.result_description = "SUCCESS";
  ret_complete_solution.solution_quality.maximum_robot_schedule =
      max_robot_schedule;
  ret_complete_solution.solution_quality.sum_of_all_robot_schedules = -1.0;
  ret_complete_solution.solution_quality.solver_runtime = 0.0;

  MrtaSolution::RobotTasksSchedule robot_solution;
  robot_solution.robot_id = robot_id;

  for (const std::string task : curr_path_i_A) {
    robot_solution.task_attendance_sequence.push_back(task);
    robot_solution.task_arrival_time_map[task] =
        robot_task_attendance_times_map[robot_name][task];
  }

  ret_complete_solution.robot_task_schedule_map[robot_name] = robot_solution;
}

void MrtaDecentralizedHssSolver::taskInclusionPhase(
    std::vector<std::string> &curr_path_i_A) {
  for (const std::string &task : relevant_tasks_names) {
    std::pair<size_t, size_t> index_of_pred_succ =
        getIndexOfPredecessorSuccessorToTask(task, curr_path_i_A,
                                             ordered_tasks_i_B);
    size_t task_proposed_index_l =
        getTaskInsertionIndex(curr_path_i_A, task, index_of_pred_succ);
    if (getAfterTaskInsertionCost(curr_path_i_A, task, task_proposed_index_l) <
        getCost(curr_path_i_A)) {
      curr_path_i_A.insert(curr_path_i_A.begin() + task_proposed_index_l, task);
    }
  }
}

std::pair<size_t, size_t>
MrtaDecentralizedHssSolver::getIndexOfPredecessorSuccessorToTask(
    const std::string &task, const std::vector<std::string> &curr_path_i_A,
    const std::vector<std::string> &ordered_tasks_i_B) {
  int start_task_index = getIndexOfObjectInVector(
      MrtaConfig::StdTaskNames::START_TASK, curr_path_i_A);
  int end_task_index = getIndexOfObjectInVector(
      MrtaConfig::StdTaskNames::END_TASK, curr_path_i_A);
  std::pair<size_t, size_t> pred_succ_pair(start_task_index, end_task_index);
  std::vector<std::string>::const_iterator task_iter_in_ordered_tasks =
      getIterToObjectInVector(ordered_tasks_i_B, task);
  if (isObjectInVector(task_iter_in_ordered_tasks, ordered_tasks_i_B)) {
    int task_index_in_ordered_tasks_B_l_j =
        getIndexOfObjectInVector(task_iter_in_ordered_tasks, ordered_tasks_i_B);
    if (task_index_in_ordered_tasks_B_l_j > 0) {
      for (int j_prime_index_in_ordered_tasks =
               task_index_in_ordered_tasks_B_l_j - 1;
           j_prime_index_in_ordered_tasks >= 0;
           --j_prime_index_in_ordered_tasks) {
        std::string j_prime =
            ordered_tasks_i_B.at(j_prime_index_in_ordered_tasks);
        std::vector<std::string>::const_iterator j_prime_iter_in_curr_path =
            getIterToObjectInVector(curr_path_i_A, j_prime);
        if (isObjectInVector(j_prime_iter_in_curr_path, curr_path_i_A)) {
          pred_succ_pair.first = getIndexOfObjectInVector(
              j_prime_iter_in_curr_path, curr_path_i_A);
          break;
        }
      }
    }
    if (task_index_in_ordered_tasks_B_l_j < ordered_tasks_i_B.size() - 1) {
      for (int j_double_prime_index_in_ordered_tasks =
               task_index_in_ordered_tasks_B_l_j + 1;
           j_double_prime_index_in_ordered_tasks < ordered_tasks_i_B.size();
           ++j_double_prime_index_in_ordered_tasks) {
        std::string j_double_prime =
            ordered_tasks_i_B.at(j_double_prime_index_in_ordered_tasks);
        std::vector<std::string>::const_iterator
            j_double_prime_iter_in_curr_path =
                getIterToObjectInVector(curr_path_i_A, j_double_prime);
        if (isObjectInVector(j_double_prime_iter_in_curr_path, curr_path_i_A)) {
          pred_succ_pair.second = getIndexOfObjectInVector(
              j_double_prime_iter_in_curr_path, curr_path_i_A);
          break;
        }
      }
    }
  }
  return pred_succ_pair;
}

size_t MrtaDecentralizedHssSolver::getTaskInsertionIndex(
    const std::vector<std::string> &curr_path_i_A, const std::string &task,
    std::pair<size_t, size_t> index_of_pred_succ) {
  double min_insertion_cost = std::numeric_limits<double>::max();
  size_t min_insertion_index = -1;
  for (size_t index_l = index_of_pred_succ.first + 1;
       index_l <= index_of_pred_succ.second; ++index_l) {
    double insertion_cost =
        getAfterTaskInsertionCost(curr_path_i_A, task, index_l);
    if (insertion_cost < min_insertion_cost) {
      min_insertion_index = index_l;
      min_insertion_cost = insertion_cost;
    }
  }
  return min_insertion_index;
}

double MrtaDecentralizedHssSolver::getAfterTaskInsertionCost(
    const std::vector<std::string> &curr_path_i_A, const std::string &task,
    size_t index) {
  std::vector<std::string> curr_path_with_insertion_i_oplus_j_A(curr_path_i_A);

  curr_path_with_insertion_i_oplus_j_A.insert(
      curr_path_with_insertion_i_oplus_j_A.begin() + index, task);

  return getCost(curr_path_with_insertion_i_oplus_j_A);
}

double MrtaDecentralizedHssSolver::getCost(
    const std::vector<std::string> &curr_path_i_A) {
  return getCostOfTravelC1(curr_path_i_A) +
         getCostOfAttendanceC2(curr_path_i_A);
}

double MrtaDecentralizedHssSolver::getCostOfTravelC1(
    const std::vector<std::string> &curr_path_i_A, bool update_solution) {
  std::string last_task = MrtaConfig::StdTaskNames::START_TASK;
  double robot_arrival_at_last_task = 0.0;
  double last_task_execution_start = 0.0;
  double last_task_execution_duration = 0.0;

  for (const std::string &task : curr_path_i_A) {
    if (task == MrtaConfig::StdTaskNames::START_TASK) {
      continue;
    }

    int index_of_last_task = task_name_to_id_map[last_task];
    int index_of_task = task_name_to_id_map[task];
    int index_of_robot = robot_name_to_id_map[robot_name];

    double arrival_at_task =
        last_task_execution_start + last_task_execution_duration +
        robot_travel_times_vector.at(index_of_robot)(index_of_last_task,
                                                     index_of_task);

    last_task = task;
    robot_arrival_at_last_task = arrival_at_task;
    last_task_execution_start =
        std::max(task_start_time.at(index_of_task), arrival_at_task);

    if (update_solution) {
      task_start_time.at(index_of_task) = last_task_execution_start;
      robot_task_attendance_times_map[robot_name][task] =
          robot_arrival_at_last_task;
    }

    if (task != MrtaConfig::StdTaskNames::END_TASK) {
      last_task_execution_duration =
          mrta_complete_config->tasks_map.find(last_task)->second.duration;
    }
  }
  return robot_arrival_at_last_task;
}

double MrtaDecentralizedHssSolver::getCostOfAttendanceC2(
    const std::vector<std::string> &curr_path_i_A) {
  double accumulated_cost = 0;
  for (const std::string &task : curr_path_i_A) {
    if (task == MrtaConfig::StdTaskNames::START_TASK ||
        task == MrtaConfig::StdTaskNames::END_TASK)
      continue;

    if (robot_unnecessary_at_task_i_u_j[task]) {
      accumulated_cost += LARGE_COST;
    } else {
      accumulated_cost -= LARGE_COST;
    }
  }
  return accumulated_cost;
}

void MrtaDecentralizedHssSolver::commAndVarUpdatePhase(
    const MrtaSolution::CompleteSolution &complete_solution) {
  defineUnnecessaryRobots(complete_solution);
  if (hasOverallScheduleSettled(complete_solution)) {
    ++schedule_convergence_timer;
  } else {
    schedule_convergence_timer = 0;
  }

  if (schedule_convergence_timer > SCHEDULE_STABLE_FOR_TIMESTEPS_y_T ||
      timestep > MAXIMUM_NUMBER_OF_ITERATIONS) {
    defineTaskSequence(complete_solution);
    if (first) {
      schedule_convergence_timer = 0;
    }
  }
}

void MrtaDecentralizedHssSolver::defineTaskSequence(
    const MrtaSolution::CompleteSolution &complete_solution) {
  ordered_tasks_i_B.clear();
  while (temp_ordered_tasks_set_i_B.size() > 0) {
    ordered_tasks_i_B.push_back(temp_ordered_tasks_set_i_B.top().first);
    temp_ordered_tasks_set_i_B.pop();
  }
}

void MrtaDecentralizedHssSolver::defineUnnecessaryRobots(
    const MrtaSolution::CompleteSolution &complete_solution) {
  std::map<std::string, std::set<std::string>> tasks_coalition_map_Z;
  for (const auto &task : relevant_tasks_names) {
    std::pair<std::string, double> robot_arrival_time_pair;
    std::priority_queue<std::pair<std::string, double>,
                        std::vector<std::pair<std::string, double>>,
                        MrtaConfig::CompareSecond>
        robot_arrival_time_queue_j_I;
    temp_ordered_tasks_set_i_B =
        std::priority_queue<std::pair<std::string, double>,
                            std::vector<std::pair<std::string, double>>,
                            MrtaConfig::CompareSecond>();

    std::set<std::string> coalition_at_task;
    std::set<std::string> coalition_skillset;

    getRobotsQueueAttendingTask(robot_arrival_time_queue_j_I, task,
                                complete_solution);

    double latest_arrival_time = 0.0;
    double sum_of_arrival_times = 0.0;
    while (robot_arrival_time_queue_j_I.size() > 0) {
      std::pair<std::string, double> robot_arrival_time_pair =
          robot_arrival_time_queue_j_I.top();
      robot_arrival_time_queue_j_I.pop();
      std::map<std::string, MrtaConfig::Robot>::const_iterator robot_iter =
          mrta_complete_config->robots_map.find(robot_arrival_time_pair.first);
      if (robot_iter == mrta_complete_config->robots_map.end())
        continue;
      bool robot_skills_subset_of_coalition =
          isMapSubsetOfSet(robot_iter->second.skillset, coalition_skillset);
      if (robot_skills_subset_of_coalition)
        continue;
      coalition_at_task.insert(robot_arrival_time_pair.first);

      addMapKeysToSet(robot_iter->second.skillset, coalition_skillset);

      sum_of_arrival_times += robot_arrival_time_pair.second;
      if (robot_arrival_time_pair.second > latest_arrival_time &&
          robot_arrival_time_pair.first != robot_name) {
        latest_arrival_time = robot_arrival_time_pair.second;
      }

      std::map<std::string, MrtaConfig::Task>::const_iterator task_iter =
          mrta_complete_config->tasks_map.find(task);
      bool task_requirements_subset_of_coalition_skillset =
          isMapSubsetOfSet(task_iter->second.skillset, coalition_skillset);
      if (task_requirements_subset_of_coalition_skillset)
        break;
    }

    if (coalition_at_task.size() > 1) {
      double average_arrival_time =
          sum_of_arrival_times / coalition_at_task.size();
      temp_ordered_tasks_set_i_B.emplace(
          std::make_pair(task, average_arrival_time));
    }

    bool robot_not_in_coalition =
        (coalition_at_task.find(robot_name) == coalition_at_task.end());
    if (coalition_at_task.size() > 0 && robot_not_in_coalition) {
      robot_unnecessary_at_task_i_u_j[task] = true;
    } else {
      robot_unnecessary_at_task_i_u_j[task] = false;
    }
    int task_id = task_name_to_id_map[task];
    task_start_time.at(task_id) = latest_arrival_time;
  }
}

void MrtaDecentralizedHssSolver::getRobotsQueueAttendingTask(
    std::priority_queue<std::pair<std::string, double>,
                        std::vector<std::pair<std::string, double>>,
                        MrtaConfig::CompareSecond>
        &robot_arrival_time_queue_j_I,
    const std::string &task,
    const MrtaSolution::CompleteSolution &complete_solution) {

  for (const auto &robot : mrta_complete_config->setup.all_robot_names) {
    std::map<std::string, MrtaSolution::RobotTasksSchedule>::const_iterator
        robot_schedule_iter =
            complete_solution.robot_task_schedule_map.find(robot);
    if (robot_schedule_iter !=
        complete_solution.robot_task_schedule_map.end()) {
      std::map<std::string, double>::const_iterator arrival_time_iter =
          robot_schedule_iter->second.task_arrival_time_map.find(task);
      if (arrival_time_iter !=
          robot_schedule_iter->second.task_arrival_time_map.end()) {
        if (arrival_time_iter->second > 0.0) {
          robot_arrival_time_queue_j_I.emplace(
              std::make_pair(robot, arrival_time_iter->second));
        }
      }
    }
  }
}

int MrtaDecentralizedHssSolver::isMapSubsetOfSet(
    const std::map<std::string, double> &source_map,
    const std::set<std::string> &ref_set) {
  int common_elements = 0;
  int non_zero_elements = 0;
  for (const auto pair : source_map) {
    if (pair.second > 0.0) {
      ++non_zero_elements;
      if (ref_set.find(pair.first) != ref_set.end()) {
        ++common_elements;
      }
    }
  }
  return common_elements == non_zero_elements;
}

void MrtaDecentralizedHssSolver::addMapKeysToSet(
    const std::map<std::string, double> &source_map,
    std::set<std::string> &ref_set) {
  for (const auto &key_value_pair : source_map) {
    if (key_value_pair.second > 0.0) {
      ref_set.insert(key_value_pair.first);
    }
  }
}

bool MrtaDecentralizedHssSolver::hasOverallScheduleSettled(
    const MrtaSolution::CompleteSolution &complete_solution) {

  // robot_arrival_times_at_tasks
  if (last_timestep_robot_arrival_times_at_tasks.size() == 0) {
    last_timestep_robot_arrival_times_at_tasks = Eigen::MatrixXd::Zero(
        mrta_complete_config->setup.number_of_robots,
        mrta_complete_config->setup.number_of_destinations);
  }
  robot_arrival_times_at_tasks =
      Eigen::MatrixXd::Zero(mrta_complete_config->setup.number_of_robots,
                            mrta_complete_config->setup.number_of_destinations);

  for (const auto robot : mrta_complete_config->setup.all_robot_names) {
    for (const auto task : relevant_tasks_names) {
      int robot_num = robot_name_to_id_map[robot];
      int task_num = task_name_to_id_map[task];

      std::map<std::string, MrtaSolution::RobotTasksSchedule>::const_iterator
          robot_schedule_iter =
              complete_solution.robot_task_schedule_map.find(robot);
      if (robot_schedule_iter !=
          complete_solution.robot_task_schedule_map.end()) {
        std::map<std::string, double>::const_iterator
            robot_arrival_at_task_iter =
                robot_schedule_iter->second.task_arrival_time_map.find(task);
        if (robot_arrival_at_task_iter !=
            robot_schedule_iter->second.task_arrival_time_map.end()) {
          robot_arrival_times_at_tasks(robot_num, task_num) =
              robot_arrival_at_task_iter->second;
        }
      }
    }
  }

  double change_in_schedule_delta_Y =
      (last_timestep_robot_arrival_times_at_tasks -
       robot_arrival_times_at_tasks)
          .norm();
  last_timestep_robot_arrival_times_at_tasks = robot_arrival_times_at_tasks;

  return change_in_schedule_delta_Y < ARRIVAL_TIME_CHANGE_THRESHOLD_Y;
}
