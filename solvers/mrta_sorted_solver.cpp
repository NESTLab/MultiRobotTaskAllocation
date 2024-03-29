#include <mrta_solvers/mrta_sorted_solver.h>

void MrtaSortedSolver::solveMrtaProblem(
    const MrtaConfig::CompleteConfig &mrta_complete_config,
    MrtaSolution::CompleteSolution &ret_complete_solution) {

  ret_complete_solution.solution_quality.result_status = 0;
  ret_complete_solution.solution_quality.result_description = "success";
  ret_complete_solution.solution_quality.solver_runtime = "2H:45M:18S";
  ret_complete_solution.solution_quality.maximum_robot_schedule = 30.0;
  ret_complete_solution.solution_quality.sum_of_all_robot_schedules = 80.0;

  int cycle_robot_ids = 0;
  for (size_t j = 1; j < mrta_complete_config.setup.number_of_destinations - 1;
       j++) {
    std::string robot_name =
        mrta_complete_config.setup.all_robot_names.at(cycle_robot_ids++);
    ret_complete_solution.robot_task_schedule_map[robot_name].robot_id = robot_name;

    std::vector<std::string> &robot_task_seq_vec =
        ret_complete_solution.robot_task_schedule_map[robot_name].task_attendance_sequence;

    const std::string &task_name =
        mrta_complete_config.setup.all_destination_names.at(j);
    robot_task_seq_vec.push_back(task_name);

    ret_complete_solution.robot_task_schedule_map[robot_name]
        .task_arrival_time_map[task_name] = 10 * robot_task_seq_vec.size();

    if (cycle_robot_ids == mrta_complete_config.setup.number_of_robots)
      cycle_robot_ids = 0;
  }
}