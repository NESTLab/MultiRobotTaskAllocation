#include "mrta_utilities/mrta_json_parser.h"

/**
 * @brief Constructs a new Config object.
 * @param json_file_name The name of the JSON file to load configurations from.
 */
void MrtaJsonParser::parseJsonFile(
    const std::string &json_file_name,
    MrtaConfig::CompleteConfig &ret_complete_config) {

  try {
    std::ifstream file(json_file_name);
    if (!file.is_open()) {
      std::cerr << "Failed to open JSON file." << std::endl;
    }
    json json_data;
    file >> json_data;
    file.close();
    checkMandatoryFieldsInJson(json_data);

    loadSetupFromJson(json_data, ret_complete_config.setup);
    // std::cout << "[INFO] All setup loaded!" << std::endl;
    loadSolverInfoFromJson(json_data, ret_complete_config.solver_info);
    // std::cout << "[INFO] Solver loaded!" << std::endl;
    loadTasksFromJson(json_data, ret_complete_config.setup,
                      ret_complete_config.tasks_map);
    // std::cout << "[INFO] All tasks loaded!" << std::endl;
    loadRobotsFromJson(json_data, ret_complete_config.setup,
                       ret_complete_config.robots_map);
    // std::cout << "[INFO] All robots loaded!" << std::endl;
    loadSkillDegradationFromJson(json_data, ret_complete_config.setup,
                                 ret_complete_config.environment);
    // std::cout << "[INFO] All skill degradation rates loaded!" << std::endl;
    // loadPathSigmas(json_data);

  } catch (const std::exception &e) {
    throw std::runtime_error("Error while loading data from json file. " +
                             std::string(e.what()));
  }
}

void MrtaJsonParser::checkMandatoryFieldsInJson(const json &json_data) {
  throwErrorIfKeyMissing(json_data, {json_setup, json_tasks, json_robots});
  throwErrorIfKeyMissing(json_data[json_setup],
                         {json_robots, json_tasks, json_num_skills});

  for (const auto &current_task_json_data : json_data[json_tasks]) {
    throwErrorIfKeyMissing(current_task_json_data,
                           {json_pos, json_duration, json_skillset});
    throwErrorIfKeyMissing(current_task_json_data[json_pos],
                           std::vector<std::string>({"x", "y"}));
  }

  for (const auto &current_robot_json_data : json_data[json_robots]) {
    std::string legacy_pos_name = LEGACY_MODE ? json_pose : json_pos;
    throwErrorIfKeyMissing(current_robot_json_data,
                           {legacy_pos_name, json_skillset});
    throwErrorIfKeyMissing(current_robot_json_data[legacy_pos_name],
                           std::vector<std::string>({"x", "y"}));
  }
}

/**
 * @brief Loads a setup configuration value from the JSON data.
 * @tparam T The type of the configuration value.
 * @param json_data The JSON data to load from.
 * @param field_name The name of the field to retrieve.
 * @param default_value The default value to use if the field is not found.
 * @return The loaded configuration value from json file.
 */
template <typename T>
T MrtaJsonParser::getSetupValueFromJson(const json &json_data,
                                        const std::string &field_name,
                                        const T &default_value) {
  T loaded_value = default_value;

  if (json_data[json_setup].contains(field_name)) {
    try {
      loaded_value = json_data[json_setup][field_name];
    } catch (const std::exception &e) {
      // Field not found or invalid type, using default value
      std::cout << "[WARN]"
                << "Field \"" << field_name
                << "\" not found in the input json file. Loading the default "
                   "value of: "
                << default_value << std::endl;
      loaded_value = default_value;
    }
  } else {
    // Field not found or invalid type, using default value
    std::cout << "[WARN]"
              << "JSON Error accessing setup field \"" << field_name
              << "\". Loading the default value of: " << default_value
              << std::endl;
    loaded_value = default_value;
  }

  return loaded_value;
}

void MrtaJsonParser::loadSolverInfoFromJson(
    const json &json_data, MrtaConfig::SolverInfo &mrta_config_solver_info) {
  bool solver_loaded = false;
  if (json_data.contains(json_solver_info)) {
    if (json_data[json_solver_info].contains(json_solver_type)) {
      std::string solver_type_str =
          json_data[json_solver_info][json_solver_type];
      std::map<std::string, MrtaConfig::SOLVER_TYPE>::const_iterator
          solver_type_map_iter = SOLVER_TYPE_MAP.find(solver_type_str);
      if (solver_type_map_iter != SOLVER_TYPE_MAP.end()) {
        mrta_config_solver_info.solver_type = solver_type_map_iter->second;
        solver_loaded = true;
      }
    }
  }
  if (!solver_loaded) {
    mrta_config_solver_info.solver_type = MrtaConfig::HEURISTIC_SOLVER;
  }
}

/**
 * @brief NEEDS COMMENTING
 *
 * @param json_data
 * @param mrta_config_setup
 */
void MrtaJsonParser::loadSetupFromJson(const json &json_data,
                                       MrtaConfig::Setup &mrta_config_setup) {

  mrta_config_setup.number_of_robots = json_data[json_setup][json_robots];
  mrta_config_setup.number_of_destinations =
      (int)(json_data[json_setup][json_tasks]) + 1 // +1 FOR START TASK ID
      + 1;                                         // +1 FOR END TASK ID

  mrta_config_setup.number_of_skills = json_data[json_setup][json_num_skills];

  if (json_data[json_tasks].size() + 1 // +1 FOR START TASK ID
          + 1                          // +1 FOR END TASK ID
      != (mrta_config_setup.number_of_destinations)) {
    throw std::runtime_error(
        "The number of number_of_destinations in 'setup' (" +
        std::to_string(mrta_config_setup.number_of_destinations) +
        ") must be 2 more than the number of tasks in the field 'tasks' to "
        "include START and END places as destinations. Currently, the number "
        "of given tasks are: (" +
        std::to_string(json_data[json_tasks].size()) + ")");
  }
  if (json_data[json_robots].size() != mrta_config_setup.number_of_robots) {
    throw std::runtime_error(
        "The number of robots in 'setup' (" +
        std::to_string(mrta_config_setup.number_of_robots) +
        ") does not match the length of the field 'robots' (" +
        std::to_string(json_data[json_robots].size()) + ")");
  }

  mrta_config_setup.use_robot_ends = getSetupValueFromJson(
      json_data, json_use_robot_ends, default_use_robot_ends);
  mrta_config_setup.epsilon =
      getSetupValueFromJson(json_data, json_epsilon, default_epsilon);
  mrta_config_setup.mean_percent =
      getSetupValueFromJson(json_data, json_mean_percent, default_mean_percent);
  mrta_config_setup.use_stochasticity = getSetupValueFromJson(
      json_data, json_use_stochasticity, default_use_stochasticity);
}

/**
 * @brief Loads task-related data from JSON into corresponding matrices.
 *
 * This function parses the provided JSON data and populates the task
 * requirements, task locations, and max completion time matrices based on the
 * information in the JSON. It expects the JSON data to have the following
 * structure:
 *
 * {
 *    "tasks": {
 *        "task_id_1": {
 *            "pos": {
 *                "x": 0.5,
 *                "y": 0.3
 *            },
 *            "duration": 2.5,
 *            "skillset": {
 *                "0": 75,
 *                "1": 0,
 *            }
 *        },
 *        "task_id_2": {
 *            ...
 *        },
 *        ...
 *    }
 * }
 *
 * The task requirements matrix represents the skills required for each task.
 * The tasks locations matrix holds the coordinates of each task in the task
 * arena. The max completion time matrix stores the maximum time allowed for
 * each task's completion. The START and END rows in the task requirements
 * matrix are reserved for special tasks.
 *
 * @param json_data The JSON data containing task information.
 *
 * @throw std::runtime_error If a required field is missing in the JSON data.
 */
void MrtaJsonParser::loadTasksFromJson(
    const json &json_data, MrtaConfig::Setup &mrta_config_setup,
    std::map<std::string, MrtaConfig::Task> &mrta_config_task_map) {
  int current_task_index = 0;
  try {
    // Add the START and END destination at the beginning.
    mrta_config_setup.all_destination_names.push_back(START_TASK_NAME);
    mrta_config_setup.all_destination_names.push_back(END_TASK_NAME);

    // Iterate over the tasks in the JSON data
    for (const auto &current_task_json_data : json_data[json_tasks].items()) {

      current_task_index++;
      MrtaConfig::Task mrta_config_task_current;
      std::string task_name = current_task_json_data.key();
      mrta_config_task_current.task_name = task_name;
      mrta_config_setup.all_destination_names.insert(
          mrta_config_setup.all_destination_names.begin() + current_task_index,
          task_name);

      double legacy_scaling_factor_if_present =
          getLegacyScalingFactorIfExists(json_data);

      // Retrieve and set the task's location
      mrta_config_task_current.position.pos_x =
          double(current_task_json_data.value()[json_pos]["x"]) *
          legacy_scaling_factor_if_present;
      mrta_config_task_current.position.pos_y =
          double(current_task_json_data.value()[json_pos]["y"]) *
          legacy_scaling_factor_if_present;

      // Set the maximum completion time for the task
      mrta_config_task_current.duration =
          double(current_task_json_data.value()[json_duration]) *
          legacy_scaling_factor_if_present;

      // Iterate over the skills required for the task
      for (const auto &c :
           current_task_json_data.value()[json_skillset].items()) {
        try {
          std::string skill_name = c.key();

          // Check if the current skill already exists in the all skills name
          // list.
          if (!(std::find(mrta_config_setup.all_skill_names.begin(),
                          mrta_config_setup.all_skill_names.end(),
                          skill_name) !=
                mrta_config_setup.all_skill_names.end()))
            // If not, add it
            mrta_config_setup.all_skill_names.push_back(skill_name);

          double legacy_adjusted_skill_value =
              getLegacyAdjustedSkillValueIfNeeded(c.key(), double(c.value()));

          mrta_config_task_current.skillset[c.key()] = legacy_adjusted_skill_value;
        } catch (const std::exception &e) {
          throw std::runtime_error(
              "skill id " + c.key() +
              " could not be inserted in vector of size " +
              std::to_string(mrta_config_setup.number_of_skills));
        }
      }
      mrta_config_task_map[mrta_config_task_current.task_name] =
          (mrta_config_task_current);
    }
  } catch (const std::exception &e) {
    // Handle exception when a required field is missing in the JSON data
    throw std::runtime_error("Missing/Incorrect Field " +
                             std::string(e.what()) +
                             " in json file for the task with index:" +
                             std::to_string(current_task_index));
  }
}

/**
 * @brief Loads robot-related data from JSON into corresponding matrices.
 *
 * This function parses the provided JSON data and populates the robot
 * skills, robot start locations, and robot end locations matrices based
 * on the information in the JSON. It expects the JSON data to have the
 * following structure:
 *
 * {
 *    "robots": {
 *        "robot_id_1": {
 *            "pose": {
 *                "x": 0.2,
 *                "y": 0.1
 *            },
 *            "skillset": {
 *                "0": 1,
 *                "1": 0,
 *            }
 *        },
 *        "robot_id_2": {
 *            ...
 *        },
 *        ...
 *    }
 * }
 *
 * The robot skills matrix represents the skills and their levels for each
 * robot. The robot start locations matrix holds the starting coordinates of
 * each robot in the task arena. The robot end locations matrix stores the
 * ending coordinates of each robot in the task arena.
 *
 * @param json_data The JSON data containing robot information.
 *
 * @throw std::runtime_error If a required field is missing in the JSON data.
 */
void MrtaJsonParser::loadRobotsFromJson(
    const json &json_data, MrtaConfig::Setup &mrta_config_setup,
    std::map<std::string, MrtaConfig::Robot> &mrta_config_robot_map) {
  // Reset robot requirements, robots locations, and max completion time
  // matrices
  int current_robot_index = 0;

  try {
    // Iterate over the robots in the JSON data
    for (const auto &current_robot_data : json_data[json_robots].items()) {

      current_robot_index++;
      MrtaConfig::Robot mrta_config_robot_current;
      std::string robot_name = current_robot_data.key();

      mrta_config_robot_current.robot_name = robot_name;
      mrta_config_setup.all_robot_names.push_back(robot_name);

      std::string legacy_pos_name = LEGACY_MODE ? json_pose : json_pos;
      double legacy_scaling_factor_if_present =
          getLegacyScalingFactorIfExists(json_data);

      // Retrieve and set the robot's location
      mrta_config_robot_current.position.pos_x =
          double(current_robot_data.value()[legacy_pos_name]["x"]) *
          legacy_scaling_factor_if_present;
      mrta_config_robot_current.position.pos_y =
          double(current_robot_data.value()[legacy_pos_name]["y"]) *
          legacy_scaling_factor_if_present;

      // Retrieve and set the robot's end location
      if (current_robot_data.value().contains(json_desired_end_position)) {
        mrta_config_robot_current.desired_end_position.pos_x =
            current_robot_data.value()[json_desired_end_position].contains("x")
                ? double(current_robot_data
                             .value()[json_desired_end_position]["x"])
                : 0.0;
        mrta_config_robot_current.desired_end_position.pos_y =
            current_robot_data.value()[json_desired_end_position].contains("y")
                ? double(current_robot_data
                             .value()[json_desired_end_position]["y"])
                : 0.0;
      } else {
        mrta_config_robot_current.desired_end_position.pos_x = 0.0;
        mrta_config_robot_current.desired_end_position.pos_y = 0.0;
      }

      mrta_config_robot_current.velocity =
          current_robot_data.value().contains(json_velocity)
              ? double(current_robot_data.value()[json_velocity])
              : default_velocity;

      // Iterate over the skills required for the robot
      for (const auto &c : current_robot_data.value()[json_skillset].items()) {
        try {
          std::string skill_name = c.key();

          double legacy_adjusted_skill_value =
              getLegacyAdjustedSkillValueIfNeeded(c.key(), double(c.value()));

          mrta_config_robot_current.skillset[c.key()] = legacy_adjusted_skill_value;

          // Check if the current skill already exists in the all skills name
          // list.
          if (!(std::find(mrta_config_setup.all_skill_names.begin(),
                          mrta_config_setup.all_skill_names.end(),
                          skill_name) !=
                mrta_config_setup.all_skill_names.end()))
            // If not, add it
            mrta_config_setup.all_skill_names.push_back(skill_name);
        } catch (const std::exception &e) {
          throw std::runtime_error(
              "skill id " + c.key() +
              " could not be inserted in vector of size " +
              std::to_string(mrta_config_setup.number_of_skills));
        }
      }
      mrta_config_robot_map[mrta_config_robot_current.robot_name] =
          (mrta_config_robot_current);
    }
  } catch (const std::exception &e) {
    // Handle exception when a required field is missing in the JSON data
    throw std::runtime_error(
        "Missing/Incorrect Field " + std::string(e.what()) +
        " in json file for the robot " + std::to_string(current_robot_index));
  }
}

// /**
//  * @brief Loads path sigma values from JSON into the sigma_percent matrix.
//  *
//  * This function parses the provided JSON data and populates the
//  sigma_percent matrix
//  * based on the information in the JSON. It expects the JSON data to have
//  the following structure:
//  *
//  * {
//  *    "sigma_percent": {
//  *        "row1,col1": value1,
//  *        "row2,col2": value2,
//  *        ...
//  *    }
//  * }
//  *
//  * The sigma_percent matrix represents the sigma values for each path in a
//  grid.
//  *
//  * @param json_data The JSON data containing path sigma information.
//  *
//  * @throw std::runtime_error If a required field is missing in the JSON
//  data.
//  */
// void Config::loadPathSigmas(const json& json_data) {
//     try {
//         // Iterate over the path sigma entries in the JSON data
//         for (const auto& entry : json_data.get_child(json_sigma_percent)) {
//             std::string key = entry.first;
//             double value = entry.second.get_value<double>();

//             // Extract the row and column from the key
//             int comma_pos = key.find(',');
//             int row = std::stoi(key.substr(0, comma_pos));
//             int col = std::stoi(key.substr(comma_pos + 1));

//             // Set the sigma value in the sigma_percent matrix
//             sigma_percent(row, col) = value;
//         }
//     } catch (const std::exception& e) {
//         // Handle exception when a required field is missing in the JSON
//         data throw std::runtime_error("Missing Field " +
//         std::string(e.what()) + " in json file");
//     }
// }

/**
 * @brief Loads skill degradation values from JSON into the
skill_degradation_rates map.
//  *
//  * This function parses the provided JSON data and populates the
sigma_percent matrix
//  * based on the information in the JSON. It expects the JSON data to have
the following structure:
//  *
//  * {
//  *    "skill_degradation_rates": {
//  *        "0": value1,
//  *        "1": value2,
//  *        ...
//  *    }
//  * }
//  *
//  * The skill_degradation_rates map represents the degradation values for
each of the skills.
//  *
//  * @param json_data The JSON data containing path sigma information.
//  *
//  * @throw std::runtime_error If a required field is missing in the JSON
data.
 *
 * @param json_data
 */
void MrtaJsonParser::loadSkillDegradationFromJson(
    const json &json_data, const MrtaConfig::Setup &mrta_config_setup,
    MrtaConfig::Environment &mrta_config_environment) {
  double default_value = 0.0;
  for (const std::string &skill : mrta_config_setup.all_skill_names) {

    if (json_data.contains(json_skill_degradation_rates) &&
        json_data[json_skill_degradation_rates].contains(skill)) {
      try {
        mrta_config_environment.skill_degradation_rate_map[skill] =
            json_data[json_skill_degradation_rates][skill];
      } catch (const std::exception &e) {
        // Field not found or invalid type, using default value
        std::cout << "[WARN]"
                  << "JSON Error accessing degradation rate for \"" << skill
                  << "\". Loading the default value of: " << default_value
                  << std::endl;
        mrta_config_environment.skill_degradation_rate_map[skill] =
            default_value;
      }
    } else {
      // Field not found or invalid type, using default value
      std::cout << "[WARN]"
                << "JSON Error accessing degradation rate for \"" << skill
                << "\". Loading the default value of: " << default_value
                << std::endl;
      mrta_config_environment.skill_degradation_rate_map[skill] = default_value;
    }
  }
}