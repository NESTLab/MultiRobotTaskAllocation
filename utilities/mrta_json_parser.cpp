#include "mrta_utilities/mrta_json_parser.h"

/**
 * @brief Constructs a new Config object.
 * @param json_file_name The name of the JSON file to load configurations from.
 */
MrtaConfig MrtaJsonParser::parseJsonFile(const std::string &json_file_name) {
  mrta_config_object = new MrtaConfig();
  try {
    std::ifstream file(json_file_name);
    if (!file.is_open()) {
      std::cerr << "Failed to open JSON file." << std::endl;
      return *mrta_config_object;
    }
    json json_data;
    file >> json_data;
    file.close();

    mrta_config_object->setup.number_of_robots =
        json_data[json_setup][json_robots];
    mrta_config_object->setup.number_of_tasks =
        json_data[json_setup][json_tasks];
    mrta_config_object->setup.number_of_tasks += 2;
    mrta_config_object->setup.number_of_capabilities =
        json_data[json_setup][json_num_skills];

    if (json_data[json_tasks].size() !=
        mrta_config_object->setup.number_of_tasks - 2) {
      throw std::runtime_error(
          "The number of tasks in 'setup' (" +
          std::to_string(mrta_config_object->setup.number_of_tasks - 2) +
          ") does not match the length of the field 'tasks' (" +
          std::to_string(json_data[json_tasks].size()) + ")");
    }
    if (json_data[json_robots].size() !=
        mrta_config_object->setup.number_of_robots) {
      throw std::runtime_error(
          "The number of robots in 'setup' (" +
          std::to_string(mrta_config_object->setup.number_of_robots) +
          ") does not match the length of the field 'robots' (" +
          std::to_string(json_data[json_robots].size()) + ")");
    }

    mrta_config_object->setup.plot_solution =
        loadSetupFromJson(json_data, json_plot_solution, default_plot_solution);
    mrta_config_object->setup.use_robot_ends = loadSetupFromJson(
        json_data, json_use_robot_ends, default_use_robot_ends);
    mrta_config_object->setup.save_plot_solution = loadSetupFromJson(
        json_data, json_save_plot_solution, default_save_plot_solution);
    mrta_config_object->setup.epsilon =
        loadSetupFromJson(json_data, json_epsilon, default_epsilon);
    mrta_config_object->setup.mean_percent =
        loadSetupFromJson(json_data, json_mean_percent, default_mean_percent);
    mrta_config_object->setup.task_arena_size = loadSetupFromJson(
        json_data, json_task_arena_size, default_task_arena_size);
    mrta_config_object->setup.use_stochasticity = loadSetupFromJson(
        json_data, json_use_stochasticity, default_use_stochasticity);

    loadTasksFromJson(json_data);
    loadRobotsFromJson(json_data);
    loadSkillDegradationFromJson(json_data);
    // loadPathSigmas(json_data);

  } catch (const std::exception &e) {
    throw std::runtime_error("Error while loading data from json file. " +
                             std::string(e.what()));
  }
  return *mrta_config_object;
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
T MrtaJsonParser::loadSetupFromJson(const json &json_data,
                                    const std::string &field_name,
                                    const T &default_value) {
  T loaded_value = default_value;

  if (json_data[json_setup].contains(field_name)) {
    try {
      std::cout << "Loading " << field_name << std::endl;
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
void MrtaJsonParser::loadTasksFromJson(const json &json_data) {
  // Reset task requirements, tasks locations, and max completion time matrices
  int current_task_index = 0;

  try {
    // Iterate over the tasks in the JSON data
    for (const auto &current_task_data : json_data[json_tasks].items()) {
      MrtaConfig::Task current_task;
      current_task.id = std::stoi(current_task_data.key());
      current_task_index = current_task.id;

      // Retrieve and set the task's location
      current_task.position.pos_x =
          double(current_task_data.value()[json_pos]["x"]) *
          mrta_config_object->setup.task_arena_size;
      current_task.position.pos_y =
          double(current_task_data.value()[json_pos]["y"]) *
          mrta_config_object->setup.task_arena_size;

      // Set the maximum completion time for the task
      current_task.duration = current_task_data.value()[json_duration];

      // Iterate over the skills required for the task
      for (const auto &c : current_task_data.value()[json_skillset].items()) {
        try {
          std::string skill_name = "";
          if (c.key().length() < 3)
            skill_name = SKILL_NAME_PREFIX + c.key();
          else
            skill_name = c.key();

          // Check if the current skill already exists in the all skills name
          // list.
          if (!(std::find(mrta_config_object->setup.skill_names.begin(),
                          mrta_config_object->setup.skill_names.end(),
                          skill_name) !=
                mrta_config_object->setup.skill_names.end()))
            // If not, add it
            mrta_config_object->setup.skill_names.push_back(skill_name);

          current_task.skillset[c.key()] = c.value();
        } catch (const std::exception &e) {
          throw std::runtime_error(
              "skill id " + c.key() +
              " could not be inserted in vector of size " +
              std::to_string(mrta_config_object->setup.number_of_capabilities));
        }
      }
      mrta_config_object->tasks.push_back(current_task);
    }
  } catch (const std::exception &e) {
    // Handle exception when a required field is missing in the JSON data
    throw std::runtime_error(
        "Missing/Incorrect Field " + std::string(e.what()) +
        " in json file for the task " + std::to_string(current_task_index));
  }
}

/**
 * @brief Loads robot-related data from JSON into corresponding matrices.
 *
 * This function parses the provided JSON data and populates the robot
 * capabilities, robot start locations, and robot end locations matrices based
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
 * The robot capabilities matrix represents the skills and their levels for each
 * robot. The robot start locations matrix holds the starting coordinates of
 * each robot in the task arena. The robot end locations matrix stores the
 * ending coordinates of each robot in the task arena.
 *
 * @param json_data The JSON data containing robot information.
 *
 * @throw std::runtime_error If a required field is missing in the JSON data.
 */
void MrtaJsonParser::loadRobotsFromJson(const json &json_data) {
  // Reset robot requirements, robots locations, and max completion time
  // matrices
  int current_robot_index = 0;

  try {
    // Iterate over the robots in the JSON data
    for (const auto &current_robot_data : json_data[json_robots].items()) {
      MrtaConfig::Robot current_robot;
      current_robot.id = std::stoi(current_robot_data.key());

      // Retrieve and set the robot's location
      current_robot.position.pos_x =
          double(current_robot_data.value()[json_pose]["x"]) *
          mrta_config_object->setup.task_arena_size;
      current_robot.position.pos_y =
          double(current_robot_data.value()[json_pose]["y"]) *
          mrta_config_object->setup.task_arena_size;

      // Iterate over the skills required for the robot
      for (const auto &c : current_robot_data.value()[json_skillset].items()) {
        try {
          std::string skill_name = "";
          if (c.key().length() < 3)
            skill_name = SKILL_NAME_PREFIX + c.key();
          else
            skill_name = c.key();

          current_robot.skillset[c.key()] = c.value();

          // Check if the current skill already exists in the all skills name
          // list.
          if (!(std::find(mrta_config_object->setup.skill_names.begin(),
                          mrta_config_object->setup.skill_names.end(),
                          skill_name) !=
                mrta_config_object->setup.skill_names.end()))
            // If not, add it
            mrta_config_object->setup.skill_names.push_back(skill_name);
        } catch (const std::exception &e) {
          throw std::runtime_error(
              "skill id " + c.key() +
              " could not be inserted in vector of size " +
              std::to_string(mrta_config_object->setup.number_of_capabilities));
        }
      }
      mrta_config_object->robots.push_back(current_robot);
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
//  * based on the information in the JSON. It expects the JSON data to have the
//  following structure:
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
//  * @throw std::runtime_error If a required field is missing in the JSON data.
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
//         // Handle exception when a required field is missing in the JSON data
//         throw std::runtime_error("Missing Field " + std::string(e.what()) + "
//         in json file");
//     }
// }

/**
 * @brief Loads skill degradation values from JSON into the
skill_degradation_rates map.
//  *
//  * This function parses the provided JSON data and populates the
sigma_percent matrix
//  * based on the information in the JSON. It expects the JSON data to have the
following structure:
//  *
//  * {
//  *    "skill_degradation_rates": {
//  *        "0": value1,
//  *        "1": value2,
//  *        ...
//  *    }
//  * }
//  *
//  * The skill_degradation_rates map represents the degradation values for each
of the skills.
//  *
//  * @param json_data The JSON data containing path sigma information.
//  *
//  * @throw std::runtime_error If a required field is missing in the JSON data.
 *
 * @param json_data
 */
void MrtaJsonParser::loadSkillDegradationFromJson(const json &json_data) {
  double default_value = 0.0;
  for (std::string skill : mrta_config_object->setup.skill_names) {

    auto res = std::mismatch(SKILL_NAME_PREFIX.begin(), SKILL_NAME_PREFIX.end(),
                             skill.begin());
    if (res.first == SKILL_NAME_PREFIX.end()) {
      skill = skill.substr(SKILL_NAME_PREFIX.length());
    }

    if (json_data[json_skill_degradation_rates].contains(skill)) {
      try {
        mrta_config_object->skill_degradation_rates[skill] =
            json_data[json_skill_degradation_rates][skill];
      } catch (const std::exception &e) {
        // Field not found or invalid type, using default value
        std::cout << "[WARN]"
                   << "JSON Error accessing degradation rate for \"" << skill
                   << "\". Loading the default value of: " << default_value
                   << std::endl;
        mrta_config_object->skill_degradation_rates[skill] = default_value;
      }
    } else {
      // Field not found or invalid type, using default value
      std::cout << "[WARN]"
                 << "JSON Error accessing degradation rate for \"" << skill
                 << "\". Loading the default value of: " << default_value
                 << std::endl;
      mrta_config_object->skill_degradation_rates[skill] = default_value;
    }
  }
}