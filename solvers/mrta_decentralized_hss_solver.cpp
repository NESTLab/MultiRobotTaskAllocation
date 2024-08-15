#include <mrta_solvers/mrta_decentralized_hss_solver.h>

bool MrtaDecentralizedHssSolver::solveOneIteration(
    const MrtaConfig::CompleteConfig &mrta_complete_config,
    MrtaSolution::CompleteSolution &ret_complete_solution) {
  taskInclusionPhase();
  commAndVarUpdatePhase();
  if ((schedule_convergence_timer > MAX_SCHEDULE_CONVERGENCE_TIME) &&
      areAllTasksSatisfied()) {
    converged = true;
  }
  return false;
}

void MrtaDecentralizedHssSolver::taskInclusionPhase() {
  std::vector<std::string> curr_path_i_A = {"START", "END"};
  ordered_tasks_i_B = {"Destination_1", "Destination_3"};
  for (const std::string &task : relevant_tasks_names) {
    std::pair<size_t, size_t> index_of_pred_succ =
        getIndexOfPredecessorSuccessorToTask(task, curr_path_i_A,
                                             ordered_tasks_i_B);
    std::cout << task << " preced - succs index " << index_of_pred_succ.first
              << " - " << index_of_pred_succ.second << std::endl;
    size_t task_proposed_index_l =
        getTaskInsertionIndex(curr_path_i_A, task, index_of_pred_succ);
    std::cout << "Insertion of " << task << " propsed at "
              << task_proposed_index_l << std::endl;
    if (getAfterTaskInsertionCost(curr_path_i_A, task, task_proposed_index_l) <
        getCost(curr_path_i_A)) {
      curr_path_i_A.insert(curr_path_i_A.begin() + task_proposed_index_l, task);
      std::cout << "Path updated accordingly" << std::endl;
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
    std::cout << "Insertion cost of " << task << " at " << index_l << " - "
              << insertion_cost << std::endl;
    if (insertion_cost < min_insertion_cost) {
      std::cout << "Min insertion updated" << std::endl;
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
    const std::vector<std::string> &curr_path_i_A) {
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
    if (robot_required_at_task_i_u_j[task]) {
      accumulated_cost -= LARGE_COST;
    } else {
      accumulated_cost += LARGE_COST;
    }
  }
  return accumulated_cost;
}
