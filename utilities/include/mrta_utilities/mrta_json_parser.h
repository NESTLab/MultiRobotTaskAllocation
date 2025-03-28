#pragma once
#include <mrta_utilities/mrta_config.h>
#include <3rdparty/json.hpp>

using json = nlohmann::json;

class MrtaJsonParser {
public:
  MrtaJsonParser(){};
  ~MrtaJsonParser(){};

  static void parseJsonFile(const std::string &json_file_name,
                            MrtaConfig::CompleteConfig &ret_complete_config);

private:
  // JSON field names
  inline const static std::string json_setup = "setup";
  inline const static std::string json_robots = "robots";
  inline const static std::string json_tasks = "tasks";
  inline const static std::string json_num_skills = "num_skills";
  inline const static std::string json_pos = "pos";
  inline const static std::string json_pose = "pose";
  inline const static std::string json_desired_end_position =
      "desired_end_position";
  inline const static std::string json_duration = "duration";
  inline const static std::string json_velocity = "velocity";
  inline const static std::string json_skillset = "skillset";
  inline const static std::string json_epsilon = "epsilon";
  inline const static std::string json_mean_percent = "mean_percent";
  inline const static std::string json_use_robot_ends = "use_robot_ends";
  inline const static std::string json_use_stochasticity = "use_stochasticity";
  inline const static std::string json_skill_degradation_rates =
      "skill_degradation_rates";

  inline const static std::string START_TASK_NAME = "START";
  inline const static std::string END_TASK_NAME = "END";

  // JSON solver config names
  inline const static std::string json_solver_info = "solver_info";
  inline const static std::string json_solver_type = "solver_type";
  inline const static std::string json_solver_config = "solver_config";

  // Solver config specific texts:
  inline const static std::string solver_config_type_decentralized_hss =
      "DECENTRALIZED_HSS";
  inline const static std::string solver_config_type_heuristic = "HEURISTIC";
  inline const static std::string solver_config_type_milp = "MILP";
  inline const static std::string solver_config_type_sorted = "SORTED";

  // Default values
  inline const static bool default_use_robot_ends = false;
  inline const static bool default_use_stochasticity = false;
  inline const static double default_epsilon = 0.95;
  inline const static double default_mean_percent = 10;
  inline const static double default_velocity = 1;

  //////////////////////////////////////////////////
  ////// Legacy json compatibility parameters //////
  //////////////////////////////////////////////////
  inline const static bool LEGACY_MODE = false;
  inline const static std::string legacy_json_task_arena_size =
      "task_arena_size";
  inline const static std::string legacy_battery_task_id = "0";

  inline const static double default_legacy_json_task_arena_size = 100.0;
  inline const static double default_legacy_battery_threshold = 50;

  inline static double getLegacyScalingFactorIfExists(const json &json_data) {
    if (!LEGACY_MODE)
      return 1.0;
    if (json_data[json_setup].contains(legacy_json_task_arena_size))
      return double(json_data[json_setup][legacy_json_task_arena_size]);
    else
      return default_legacy_json_task_arena_size;
  }

  inline static double
  getLegacyAdjustedSkillValueIfNeeded(const std::string &key, double value) {
    if (!LEGACY_MODE)
      return value;
    if (key != legacy_battery_task_id)
      return value;
    else
      return value >= default_legacy_battery_threshold;
  }
  //////////////////////////////////////////////////

  template <typename T>
  static T getSetupValueFromJson(const json &json_data,
                                 const std::string &field_name,
                                 const T &default_value);
  static void loadSetupFromJson(const json &json_data,
                                MrtaConfig::Setup &mrta_config_setup);

  static void
  loadSolverInfoFromJson(const json &json_data,
                         MrtaConfig::SolverInfo &mrta_config_solver_info);

  static void
  loadTasksFromJson(const json &json_data, MrtaConfig::Setup &mrta_config_setup,
                    std::map<std::string, MrtaConfig::Task> &mrta_config_task);

  static void loadRobotsFromJson(
      const json &json_data, MrtaConfig::Setup &mrta_config_setup,
      std::map<std::string, MrtaConfig::Robot> &mrta_config_robot);

  static void loadSkillDegradationFromJson(
      const json &json_data, const MrtaConfig::Setup &mrta_config_setup,
      MrtaConfig::Environment &mrta_config_environment);

  inline static void throwErrorIfKeyMissing(const json &json_data,
                                            const std::string &key) {
    if (!json_data.contains(key))
      throw std::invalid_argument(
          "The mandatory key \"" + key +
          "\" does not exist in the json structure. Please provide the key at "
          "its necessary place");
  };

  inline static void
  throwErrorIfKeyMissing(const json &json_data,
                         const std::vector<std::string> &keys) {
    for (const auto &key : keys) {
      throwErrorIfKeyMissing(json_data, key);
    }
  }

  static void checkMandatoryFieldsInJson(const json &json_data);

  inline const static std::map<std::string, MrtaConfig::SOLVER_TYPE>
      SOLVER_TYPE_MAP = {
          {solver_config_type_decentralized_hss,
           MrtaConfig::SOLVER_TYPE::DECENTRALIZED_HSS_SOLVER},
          {solver_config_type_heuristic,
           MrtaConfig::SOLVER_TYPE::HEURISTIC_SOLVER},
          {solver_config_type_milp, MrtaConfig::SOLVER_TYPE::MILP_SOLVER},
          {solver_config_type_sorted, MrtaConfig::SOLVER_TYPE::SORTED_SOLVER}};
};