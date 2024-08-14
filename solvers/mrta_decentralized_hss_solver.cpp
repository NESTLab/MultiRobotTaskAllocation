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
  std::vector<std::string> curr_path_i_A = {"START", "Destination_3", "END"};
  ordered_tasks_i_B = {"Destination_1","Destination_3"};
  for (const std::string &task : relevant_tasks_names) {
    std::pair<size_t, size_t> index_of_pred_succ =
        getIndexOfPredecessorSuccessorToTask(task, curr_path_i_A, ordered_tasks_i_B);
    std::cout<<task<<" preced - succs index "<<index_of_pred_succ.first<<" - "<<index_of_pred_succ.second<<std::endl;
    // size_t task_proposed_index =
    //     getTaskInsertionIndex(curr_path_i_A, task, index_of_pred_succ);
    // if (getAfterTaskInsertionCost(curr_path_i_A, task, task_proposed_index) <
    //     getCost(curr_path_i_A)) {
    //   curr_path_i_A.insert(curr_path_i_A.begin() + task_proposed_index, task);
    // }
  }
}

std::pair<size_t, size_t>
MrtaDecentralizedHssSolver::getIndexOfPredecessorSuccessorToTask(
    const std::string &task, const std::vector<std::string> &curr_path_i_A,
    const std::vector<std::string> &ordered_tasks_i_B) {
  int start_task_index = getIndexOfObjectInVector(MrtaConfig::StdTaskNames::START_TASK, curr_path_i_A);
  int end_task_index = getIndexOfObjectInVector(MrtaConfig::StdTaskNames::END_TASK, curr_path_i_A);
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
  return size_t(0);
}

double MrtaDecentralizedHssSolver::getAfterTaskInsertionCost(
    const std::vector<std::string> &curr_path_i_A, const std::string &task,
    size_t index) {
  return 0.0;
}

double MrtaDecentralizedHssSolver::getCost(
    const std::vector<std::string> &curr_path_i_A) {
  return 0.0;
}