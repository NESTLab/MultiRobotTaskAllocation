#include <mrta_interface/mrta_interface.h>

MrtaInterface::MrtaInterface() {
  // Keep the config file for reference
}

MrtaInterface::~MrtaInterface() {}

void MrtaInterface::debugPrintCompleteConfig(
    const MrtaConfig::CompleteConfig &mrta_complete_config) {
  debugPrintSetup(mrta_complete_config.setup);
  debugPrintTasksMap(mrta_complete_config.tasks_map);
  debugPrintRobotsMap(mrta_complete_config.robots_map);
  debugPrintEnvironment(mrta_complete_config.environment);
}

void MrtaInterface::debugPrintSetup(const MrtaConfig::Setup &mrta_setup) {
  int indent_level = NUMBER_OF_INDENTS_PER_LEVEL;
  std::cout << std::endl << "config.setup:" << std::endl;
  debugPrintSingleLine("number_of_robots", mrta_setup.number_of_robots,
                       indent_level);
  debugPrintSingleLine("number_of_tasks", mrta_setup.number_of_tasks,
                       indent_level);
  debugPrintSingleLine("number_of_skills", mrta_setup.number_of_skills,
                       indent_level);
  debugPrintSingleLine("epsilon", mrta_setup.epsilon, indent_level);
  debugPrintSingleLine("mean_percent", mrta_setup.mean_percent, indent_level);
  debugPrintSingleLine("use_stochasticity", mrta_setup.use_stochasticity,
                       indent_level);
  debugPrintSingleLine("use_robot_ends", mrta_setup.use_robot_ends,
                       indent_level);

  debugPrintSingleLine("all_task_names", "", indent_level);
  ++indent_level;
  for (const auto &task : mrta_setup.all_task_names) {
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

void MrtaInterface::debugPrintTasksMap(
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

void MrtaInterface::debugPrintRobotsMap(
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

    debugPrintSingleLine("skillset", "", indent_level);
    ++indent_level;
    for (const auto &skill : robot.second.skillset) {
      debugPrintSingleLine(skill.first, skill.second, indent_level);
    }
    --indent_level;
  }
}

void MrtaInterface::debugPrintEnvironment(
    const MrtaConfig::Environment &mrta_environment) {
  std::cout << "config.environment" << std::endl;

  int indent_level = NUMBER_OF_INDENTS_PER_LEVEL;

  debugPrintSingleLine("skill_degradation_rate_map", "", indent_level);
  ++indent_level;
  for (const auto& skill : mrta_environment.skill_degradation_rate_map)
  {
    debugPrintSingleLine(skill.first, skill.second, indent_level);
  }
  --indent_level;

  debugPrintSingleLine("path_stochasticity_sigma_values", "", indent_level);
  ++indent_level;
  for (const auto& path : mrta_environment.path_stochasticity_sigma_values)
  {
    debugPrintSingleLine(path.first, path.second, indent_level);
  }
  --indent_level;
  
}

template <typename T>
void MrtaInterface::debugPrintSingleLine(const std::string &field, T value,
                                         int number_of_indents) {
  for (int i = 0; i < number_of_indents - 1; i++)
    std::cout << "|" << std::string(NUMBER_OF_DASHES_PER_INDENT, ' ');

  std::cout << "|" << std::string(NUMBER_OF_DASHES_PER_INDENT, '-') << field
            << " : " << value << std::endl;
}