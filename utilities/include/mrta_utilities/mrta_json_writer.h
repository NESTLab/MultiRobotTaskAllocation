#pragma once
#include <mrta_utilities/mrta_solution.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class MrtaJsonWriter {
public:
  MrtaJsonWriter(){};
  ~MrtaJsonWriter(){};

  // static const MrtaConfig& MrtaJsonWriter(const std::string& json_file_name);
  static void writeJsonFile(const MrtaSolution::CompleteSolution &solution,
                            const std::string &output_file);

private:
  // JSON field names
  inline const static std::string json_solution_quality = "solution_quality";
  inline const static std::string json_result_status = "result_status";
  inline const static std::string json_result_description =
      "result_description";
  inline const static std::string json_maximum_robot_schedule =
      "maximum_robot_schedule";
  inline const static std::string json_sum_of_all_robot_schedules =
      "sum_of_all_robot_schedules";
  inline const static std::string json_solver_runtime = "solver_runtime";

  inline const static std::string json_robots_schedule = "robots_schedule";
  inline const static std::string json_robot_id = "robot_id";
  inline const static std::string json_task_sequence_map = "task_sequence_map";
  inline const static std::string json_task_arrival_time_map =
      "task_arrival_time_map";

  static json
  addQualityValuesToJson(const MrtaSolution::SolutionQuality &solution_quality);

  static json addRobotSchedulesToJson(
      const std::map<std::string, MrtaSolution::RobotTasksSchedule>
          &solution_quality);
  // template <typename T>
  // static T getSetupValueFromJson(const json &json_data,
  //                                const std::string &field_name,
  //                                const T &default_value);
  // static void loadSetupFromJson(const json &json_data,
  //                               MrtaConfig::Setup &mrta_config_setup);

  // static void
  // loadTasksFromJson(const json &json_data, MrtaConfig::Setup
  // &mrta_config_setup,
  //                   std::map<std::string, MrtaConfig::Task>
  //                   &mrta_config_task);

  // static void loadRobotsFromJson(
  //     const json &json_data, MrtaConfig::Setup &mrta_config_setup,
  //     std::map<std::string, MrtaConfig::Robot> &mrta_config_robot);

  // static void loadSkillDegradationFromJson(
  //     const json &json_data, const MrtaConfig::Setup &mrta_config_setup,
  //     MrtaConfig::Environment &mrta_config_environment);
};