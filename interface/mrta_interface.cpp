#include <mrta_interface/mrta_interface.h>

MrtaInterface::MrtaInterface() {
  // Keep the config file for reference
}

MrtaInterface::~MrtaInterface() {}

void MrtaInterface::debugPrintConfigCompleteConfig(
    const std::shared_ptr<const MrtaConfig::CompleteConfig>
        mrta_complete_config) {
  debugPrintConfigSetup(mrta_complete_config->setup);
  debugPrintConfigTasksMap(mrta_complete_config->tasks_map);
  debugPrintConfigRobotsMap(mrta_complete_config->robots_map);
  debugPrintConfigEnvironment(mrta_complete_config->environment);
}

void MrtaInterface::debugPrintConfigSetup(const MrtaConfig::Setup &mrta_setup) {
  int indent_level = NUMBER_OF_INDENTS_PER_LEVEL;
  std::cout << std::endl << "config.setup:" << std::endl;
  debugPrintSingleLine("number_of_robots", mrta_setup.number_of_robots,
                       indent_level);
  debugPrintSingleLine("number_of_destinations",
                       mrta_setup.number_of_destinations, indent_level);
  debugPrintSingleLine("number_of_skills", mrta_setup.number_of_skills,
                       indent_level);
  debugPrintSingleLine("epsilon", mrta_setup.epsilon, indent_level);
  debugPrintSingleLine("mean_percent", mrta_setup.mean_percent, indent_level);
  debugPrintSingleLine("use_stochasticity", mrta_setup.use_stochasticity,
                       indent_level);
  debugPrintSingleLine("use_robot_ends", mrta_setup.use_robot_ends,
                       indent_level);

  debugPrintSingleLine("all_destination_names", "", indent_level);
  ++indent_level;
  for (const auto &task : mrta_setup.all_destination_names) {
    debugPrintSingleLine(task, "", indent_level);
  }
  --indent_level;

  debugPrintSingleLine("all_robot_names", "", indent_level);
  ++indent_level;
  for (const auto &robot : mrta_setup.all_robot_names) {
    debugPrintSingleLine(robot, "", indent_level);
  }
  --indent_level;

  debugPrintSingleLine("all_skill_names", "", indent_level);
  ++indent_level;
  for (const auto &skill : mrta_setup.all_skill_names) {
    debugPrintSingleLine(skill, "", indent_level);
  }
  --indent_level;
}

void MrtaInterface::debugPrintConfigTasksMap(
    const std::map<std::string, MrtaConfig::Task> &mrta_task_map) {
  std::cout << std::endl << "config.task_map:" << std::endl;
  for (const auto &task : mrta_task_map) {
    int indent_level = NUMBER_OF_INDENTS_PER_LEVEL;
    debugPrintSingleLine(task.first, "", indent_level);

    ++indent_level;
    debugPrintSingleLine("task_name", task.second.task_name, indent_level);
    debugPrintSingleLine("duration", task.second.duration, indent_level);

    debugPrintSingleLine("position", "", indent_level);
    ++indent_level;
    debugPrintSingleLine("pos_x", task.second.position.pos_x, indent_level);
    debugPrintSingleLine("pos_y", task.second.position.pos_y, indent_level);
    --indent_level;

    debugPrintSingleLine("skillset", "", indent_level);
    ++indent_level;
    for (const auto &skill : task.second.skillset) {
      debugPrintSingleLine(skill.first, skill.second, indent_level);
    }
    --indent_level;
  }
}

void MrtaInterface::debugPrintConfigRobotsMap(
    const std::map<std::string, MrtaConfig::Robot> &mrta_robot_map) {
  std::cout << std::endl << "config.robot_map:" << std::endl;
  for (const auto &robot : mrta_robot_map) {
    int indent_level = NUMBER_OF_INDENTS_PER_LEVEL;

    debugPrintSingleLine(robot.first, "", indent_level);
    ++indent_level;
    debugPrintSingleLine("robot_name", robot.second.robot_name, indent_level);

    debugPrintSingleLine("position", "", indent_level);
    ++indent_level;
    debugPrintSingleLine("pos_x", robot.second.position.pos_x, indent_level);
    debugPrintSingleLine("pos_y", robot.second.position.pos_y, indent_level);
    --indent_level;

    debugPrintSingleLine("desired_end_position", "", indent_level);
    ++indent_level;
    debugPrintSingleLine("pos_x", robot.second.desired_end_position.pos_x,
                         indent_level);
    debugPrintSingleLine("pos_y", robot.second.desired_end_position.pos_y,
                         indent_level);
    --indent_level;

    debugPrintSingleLine("skillset", "", indent_level);
    ++indent_level;
    for (const auto &skill : robot.second.skillset) {
      debugPrintSingleLine(skill.first, skill.second, indent_level);
    }
    --indent_level;
  }
}

void MrtaInterface::debugPrintConfigEnvironment(
    const MrtaConfig::Environment &mrta_environment) {
  std::cout << std::endl << "config.environment" << std::endl;

  int indent_level = NUMBER_OF_INDENTS_PER_LEVEL;

  debugPrintSingleLine("skill_degradation_rate_map", "", indent_level);
  ++indent_level;
  for (const auto &skill : mrta_environment.skill_degradation_rate_map) {
    debugPrintSingleLine(skill.first, skill.second, indent_level);
  }
  --indent_level;

  debugPrintSingleLine("path_stochasticity_sigma_values", "", indent_level);
  ++indent_level;
  for (const auto &path : mrta_environment.path_stochasticity_sigma_values) {
    debugPrintSingleLine(path.first, path.second, indent_level);
  }
  --indent_level;
}

void MrtaInterface::debugPrintSolution(
    std::shared_ptr<const MrtaSolution::CompleteSolution> const solution) {
  std::cout << std::endl << "Printing Solution:" << std::endl;
  debugPrintSolutionQuality(solution->solution_quality);
  debugPrintSolutionSchedule(solution->robot_task_schedule_map);
}

void MrtaInterface::debugPrintSolutionSchedule(
    const std::map<std::string, MrtaSolution::RobotTasksSchedule>
        robots_schedule) {
  std::cout << std::endl << "solution.robots_schedule:" << std::endl;
  for (const auto &robot : robots_schedule) {
    int indent_level = NUMBER_OF_INDENTS_PER_LEVEL;

    debugPrintSingleLine(robot.first, "", indent_level);

    ++indent_level;
    debugPrintSingleLine("robot_id", robot.second.robot_id, indent_level);

    debugPrintSingleLine("task_attendance_sequence", "", indent_level);

    ++indent_level;
    debugPrintSolutionTaskVec(robot.second.task_attendance_sequence, indent_level);
    --indent_level;

    debugPrintSingleLine("task_arrival_time_map", "", indent_level);

    ++indent_level;
    debugPrintSolutionTaskMap(robot.second.task_arrival_time_map, indent_level);
    --indent_level;
  }
}

void MrtaInterface::debugPrintSolutionTaskVec(
    const std::vector<std::string> &task_seq_vec, int indent_level) {
  for (size_t j = 0; j < task_seq_vec.size(); j++){
    debugPrintSingleLine(std::to_string(j), task_seq_vec.at(j), indent_level);
  }
}

template <typename T>
void MrtaInterface::debugPrintSolutionTaskMap(
    const std::map<std::string, T> task_map, int indent_level) {
  for (const auto &task : task_map) {
    debugPrintSingleLine(task.first, task.second, indent_level);
  }
}

void MrtaInterface::debugPrintSolutionQuality(
    const MrtaSolution::SolutionQuality &solution_quality) {
  std::cout << std::endl << "solution.solution_quality:" << std::endl;

  int indent_level = NUMBER_OF_INDENTS_PER_LEVEL;
  debugPrintSingleLine("result_status", solution_quality.result_status,
                       indent_level);
  debugPrintSingleLine("result_description",
                       solution_quality.result_description, indent_level);
  debugPrintSingleLine("maximum_robot_schedule",
                       solution_quality.maximum_robot_schedule, indent_level);
  debugPrintSingleLine("sum_of_all_robot_schedules",
                       solution_quality.sum_of_all_robot_schedules,
                       indent_level);
  debugPrintSingleLine("solver_runtime", solution_quality.solver_runtime,
                       indent_level);
}

template <typename T>
void MrtaInterface::debugPrintSingleLine(const std::string &field,
                                         const T &value,
                                         int number_of_indents) {
  for (int i = 0; i < number_of_indents - 1; i++)
    std::cout << "|" << std::string(NUMBER_OF_DASHES_PER_INDENT, ' ');

  std::cout << "|" << std::string(NUMBER_OF_DASHES_PER_INDENT, '-') << field
            << " : " << value << std::endl;
}

bool MrtaInterface::healthCheckConfig(
    const std::shared_ptr<const MrtaConfig::CompleteConfig>
        mrta_complete_config) {
  bool health_ok = true;

  // Check if setup has mandatory fields
  health_ok = health_ok && healthCheckMandatoryFields(mrta_complete_config);

  // Check if number of things in setup matches number of elements in maps
  health_ok = health_ok && healthCheckNumOfRobots(mrta_complete_config);
  health_ok = health_ok && healthCheckNumOfTasks(mrta_complete_config);

  return health_ok;
}

bool MrtaInterface::healthCheckMandatoryFields(
    const std::shared_ptr<const MrtaConfig::CompleteConfig>
        mrta_complete_config) {
  bool health_ok = true;
  health_ok = health_ok &&
              healthCheckField("number_of_robots",
                               mrta_complete_config->setup.number_of_robots);
  health_ok =
      health_ok &&
      healthCheckField("number_of_destinations",
                       mrta_complete_config->setup.number_of_destinations);
  health_ok = health_ok &&
              healthCheckField("number_of_skills",
                               mrta_complete_config->setup.number_of_skills);

  return health_ok;
}
bool MrtaInterface::healthCheckNumOfRobots(
    const std::shared_ptr<const MrtaConfig::CompleteConfig>
        mrta_complete_config) {
  bool health_ok = true;
  health_ok =
      health_ok && healthCheckNumOfElements(
                       "robots", "config.setup.all_robot_names",
                       mrta_complete_config->setup.number_of_robots,
                       mrta_complete_config->setup.all_robot_names.size());
  health_ok = health_ok && healthCheckNumOfElements(
                               "robots", "config.robots_map",
                               mrta_complete_config->setup.number_of_robots,
                               mrta_complete_config->robots_map.size());

  return health_ok;
}
bool MrtaInterface::healthCheckNumOfTasks(
    const std::shared_ptr<const MrtaConfig::CompleteConfig>
        mrta_complete_config) {
  bool health_ok = true;
  health_ok = health_ok &&
              healthCheckNumOfElements(
                  "tasks", "config.setup.all_destination_names",
                  mrta_complete_config->setup.number_of_destinations,
                  mrta_complete_config->setup.all_destination_names.size());
  health_ok =
      health_ok &&
      healthCheckNumOfElements(
          "tasks", "config.tasks_map",
          mrta_complete_config->setup.number_of_destinations,
          mrta_complete_config->tasks_map.size() + 1 // +1 FOR START TASK ID
              + 1);                                  // +1 FOR END TASK ID

  return health_ok;
}

template <typename T>
bool MrtaInterface::healthCheckField(const std::string &field, const T &value) {
  bool health_ok = value != 0;
  if (!health_ok)
    std::cerr << "[ERROR] | Mandatory field '" << field
              << "' in 'config.setup' has not been initialized." << std::endl;
  return health_ok;
}

bool MrtaInterface::healthCheckNumOfElements(const std::string &item,
                                             const std::string &item_detail,
                                             int expected_count,
                                             int received_count) {
  bool health_ok = expected_count == received_count;
  if (!health_ok)
    std::cerr << "[ERROR] | The number of " << item << " in " << item_detail
              << " does not match the number specified in config.setup"
              << std::endl;
  return health_ok;
}