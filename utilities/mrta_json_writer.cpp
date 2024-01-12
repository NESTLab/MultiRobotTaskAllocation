#include <fstream>
#include <mrta_utilities/mrta_json_writer.h>

void MrtaJsonWriter::writeJsonFile(
    std::shared_ptr<const MrtaSolution::CompleteSolution> const solution,
    const std::string &output_file) {
  json json_file;

  json_file[json_robots_schedule] =
      addRobotSchedulesToJson(solution->robot_task_schedule_map);

  json_file[json_solution_quality] =
      addQualityValuesToJson(solution->solution_quality);

  // Write the JSON to a file
  std::ofstream file(output_file);
  file << std::setw(4) << json_file << std::endl;

  return;
}

json MrtaJsonWriter::addQualityValuesToJson(
    const MrtaSolution::SolutionQuality &solution_quality) {
  json quality_json;
  quality_json[json_result_status] = solution_quality.result_status;
  quality_json[json_result_description] = solution_quality.result_description;
  quality_json[json_maximum_robot_schedule] =
      solution_quality.maximum_robot_schedule;
  quality_json[json_sum_of_all_robot_schedules] =
      solution_quality.sum_of_all_robot_schedules;
  quality_json[json_solver_runtime] = solution_quality.solver_runtime;
  return quality_json;
}

json MrtaJsonWriter::addRobotSchedulesToJson(
    const std::map<std::string, MrtaSolution::RobotTasksSchedule>
        &solution_quality) {
  json schedule_json;
  for (const auto &robot_schedule : solution_quality) {
    schedule_json[robot_schedule.first][json_robot_id] = robot_schedule.first;
    for (const auto &task : robot_schedule.second.task_sequence_map) {
      schedule_json[robot_schedule.first][json_task_sequence_map][task.first] =
          task.second;
    }
    for (const auto &task : robot_schedule.second.task_arrival_time_map) {
      schedule_json[robot_schedule.first][json_task_arrival_time_map]
                   [task.first] = task.second;
    }
  }
  return schedule_json;
}