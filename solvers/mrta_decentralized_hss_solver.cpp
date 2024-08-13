#include <mrta_solvers/mrta_decentralized_hss_solver.h>

bool MrtaDecentralizedHssSolver::solveOneIteration() {
  taskInclusionPhase();
  commAndVarUpdatePhase();
  if ((schedule_convergence_timer > MAX_SCHEDULE_CONVERGENCE_TIME) &&
      areAllTasksSatisfied()) {
    converged = true;
  }
  return false;
}

void MrtaDecentralizedHssSolver::taskInclusionPhase() {
  std::vector<std::string> curr_path_i_A;
  for (const std::string &task : relevant_tasks_names) {
    std::pair<size_t, size_t> index_of_pred_succ =
        getIndexOfPredecessorSuccessorToTask(task);
    size_t task_proposed_index =
        getTaskInsertionIndex(curr_path_i_A, task, index_of_pred_succ);
    if (getAfterTaskInsertionCost(curr_path_i_A, task, task_proposed_index) <
        getCost(curr_path_i_A)) {
      curr_path_i_A.insert(curr_path_i_A.begin() + task_proposed_index, task);
    }
  }
}

std::pair<size_t, size_t>
MrtaDecentralizedHssSolver::getIndexOfPredecessorSuccessorToTask(
    const std::string &task) {}

size_t MrtaDecentralizedHssSolver::getTaskInsertionIndex(
    const std::vector<std::string> &curr_path_i_A, const std::string &task,
    std::pair<size_t, size_t> index_of_pred_succ) {}

double MrtaDecentralizedHssSolver::getAfterTaskInsertionCost(
    const std::vector<std::string> &curr_path_i_A, const std::string &task,
    size_t index) {}

double MrtaDecentralizedHssSolver::getCost(
    const std::vector<std::string> &curr_path_i_A) {}