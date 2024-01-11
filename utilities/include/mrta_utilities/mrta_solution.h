#pragma once
#include <string>
#include <vector>
#include <map>

namespace MrtaSolution
{
  struct SolutionQuality {

    // 0: success
    // -1: error
    int result_status;
    
    // "Success"
    // "Model not feasible"
    std::string result_description;

    float maximum_robot_schedule;

    float sum_of_all_robot_schedules;

    float solver_runtime;
  };

  struct RobotTasksSchedule {
    std::string robot_id;

    std::map<std::string, int> task_sequence_map;
    std::map<std::string, float> task_arrival_time_map;
  };

  struct CompleteSolution
  {
    SolutionQuality solution_quality;
    std::map<std::string, RobotTasksSchedule> robots_schedule;
  };
};
