#include <mrta_solvers/mrta_sorted_solver.h>

std::shared_ptr<const MrtaSolution::CompleteSolution> const
MrtaSortedSolver::solveMrtaProblem() {
  MrtaSolution::CompleteSolution solution;

  solution.solution_quality.result_status = 0;
  solution.solution_quality.result_description = "success";
  solution.solution_quality.solver_runtime = "2H:45M:18S";
  solution.solution_quality.maximum_robot_schedule = 30.0;
  solution.solution_quality.sum_of_all_robot_schedules = 80.0;

  int cycle_robot_ids = 0;
  for (const auto &task_name : mrta_complete_config->setup.all_destination_names) {
    std::string robot_name =
        mrta_complete_config->setup.all_robot_names.at(cycle_robot_ids++);
    solution.robot_task_schedule_map[robot_name].robot_id = robot_name;

    std::map<std::string, int> &robot_task_seq_map =
        solution.robot_task_schedule_map[robot_name].task_sequence_map;

    robot_task_seq_map[task_name] = robot_task_seq_map.size() + 1;
 
    solution.robot_task_schedule_map[robot_name]
        .task_arrival_time_map[task_name] = 10 * robot_task_seq_map.size();

    if (cycle_robot_ids==mrta_complete_config->setup.number_of_robots) 
      cycle_robot_ids = 0;
  }

  std::shared_ptr<const MrtaSolution::CompleteSolution> const sol_ptr =
      std::make_shared<const MrtaSolution::CompleteSolution>(solution);
  return sol_ptr;
}