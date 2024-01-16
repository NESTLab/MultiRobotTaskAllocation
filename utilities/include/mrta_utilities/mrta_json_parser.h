#pragma once
#include <mrta_utilities/mrta_config.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class MrtaJsonParser {
public:
  MrtaJsonParser(){};
  ~MrtaJsonParser(){};

  // static const MrtaConfig& MrtaJsonParser(const std::string& json_file_name);
  static std::shared_ptr<const MrtaConfig::CompleteConfig> const 
  parseJsonFile(const std::string &json_file_name);

private:
  // JSON field names
  inline const static std::string json_setup = "setup";
  inline const static std::string json_robots = "robots";
  inline const static std::string json_tasks = "tasks";
  inline const static std::string json_num_skills = "num_skills";
  inline const static std::string json_pos = "pos";
  inline const static std::string json_pose = "pose";
  inline const static std::string json_desired_end_position = "desired_end_position";
  inline const static std::string json_duration = "duration";
  inline const static std::string json_skillset = "skillset";
  inline const static std::string json_epsilon = "epsilon";
  inline const static std::string json_mean_percent = "mean_percent";
  inline const static std::string json_use_robot_ends = "use_robot_ends";
  inline const static std::string json_use_stochasticity = "use_stochasticity";
  inline const static std::string json_skill_degradation_rates =
      "skill_degradation_rates";

  inline const static std::string START_TASK_NAME = "START";
  inline const static std::string END_TASK_NAME = "END";
  // Default values
  inline const static bool default_use_robot_ends = false;
  inline const static bool default_use_stochasticity = false;
  inline const static double default_epsilon = 0.95;
  inline const static double default_mean_percent = 10;

  template <typename T>
  static T getSetupValueFromJson(const json &json_data,
                                 const std::string &field_name,
                                 const T &default_value);
  static void loadSetupFromJson(const json &json_data,
                                MrtaConfig::Setup &mrta_config_setup);

  static void
  loadTasksFromJson(const json &json_data, MrtaConfig::Setup &mrta_config_setup,
                    std::map<std::string, MrtaConfig::Task> &mrta_config_task);

  static void loadRobotsFromJson(
      const json &json_data, MrtaConfig::Setup &mrta_config_setup,
      std::map<std::string, MrtaConfig::Robot> &mrta_config_robot);

  static void loadSkillDegradationFromJson(
      const json &json_data, const MrtaConfig::Setup &mrta_config_setup,
      MrtaConfig::Environment &mrta_config_environment);
};