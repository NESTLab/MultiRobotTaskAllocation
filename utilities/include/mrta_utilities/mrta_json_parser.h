#pragma once
#include <mrta_utilities/mrta_config.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class MrtaJsonParser {
public:
    MrtaJsonParser(){};

    // static const MrtaConfig& MrtaJsonParser(const std::string& json_file_name);
    MrtaConfig parseJsonFile(const std::string& json_file_name);

private:
    MrtaConfig* mrta_config_object;  

    // JSON field names
    const std::string json_setup = "setup";
    const std::string json_robots = "robots";
    const std::string json_tasks = "tasks";
    const std::string json_num_skills = "num_skills";
    const std::string json_plot_solution = "plot_solution";
    const std::string json_pos = "pos";
    const std::string json_pose = "pose";
    const std::string json_duration = "duration";
    const std::string json_skillset = "skillset";
    const std::string json_epsilon = "epsilon";
    const std::string json_mean_percent = "mean_percent";
    const std::string json_sigma_percent = "sigma_percent";
    const std::string json_use_robot_ends = "use_robot_ends";
    const std::string json_task_arena_size = "task_arena_size";
    const std::string json_save_plot_solution = "save_plot_solution";
    const std::string json_use_stochasticity = "use_stochasticity";
    const std::string json_skill_degradation_rates = "skill_degradation_rates";
    const std::string SKILL_NAME_PREFIX = "skill_";

    // Default values
    const bool default_plot_solution = false;
    const bool default_use_robot_ends = true;
    const bool default_save_plot_solution = false;
    const bool default_use_stochasticity = true;
    const double default_epsilon = 0.95;
    const double default_mean_percent = 10;
    const double default_task_arena_size = 1.0;
    const int json_battery_task = 0;

    template<typename T>
    T loadSetupFromJson(const json& json_data, const std::string& field_name, const T& default_value);
    void loadTasksFromJson(const json& json_data);
    void loadRobotsFromJson(const json& json_data);
    // void loadPathSigmas(const json& json_data);
    void loadSkillDegradationFromJson(const json& json_data);
};