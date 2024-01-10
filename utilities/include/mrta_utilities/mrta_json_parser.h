#pragma once
#include <mrta_utilities/mrta_config.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class MrtaJsonParser {
public:
    MrtaJsonParser(){};
    ~MrtaJsonParser(){};

    // static const MrtaConfig& MrtaJsonParser(const std::string& json_file_name);
    static MrtaConfig parseJsonFile(const std::string& json_file_name);

private:
    // JSON field names
    inline const static std::string json_setup = "setup";
    inline const static std::string json_robots = "robots";
    inline const static std::string json_tasks = "tasks";
    inline const static std::string json_num_skills = "num_skills";
    inline const static std::string json_plot_solution = "plot_solution";
    inline const static std::string json_pos = "pos";
    inline const static std::string json_pose = "pose";
    inline const static std::string json_duration = "duration";
    inline const static std::string json_skillset = "skillset";
    inline const static std::string json_epsilon = "epsilon";
    inline const static std::string json_mean_percent = "mean_percent";
    inline const static std::string json_sigma_percent = "sigma_percent";
    inline const static std::string json_use_robot_ends = "use_robot_ends";
    inline const static std::string json_task_arena_size = "task_arena_size";
    inline const static std::string json_save_plot_solution = "save_plot_solution";
    inline const static std::string json_use_stochasticity = "use_stochasticity";
    inline const static std::string json_skill_degradation_rates = "skill_degradation_rates";
    inline const static std::string SKILL_NAME_PREFIX = "skill_";

    // Default values
    inline const static bool default_plot_solution = false;
    inline const static bool default_use_robot_ends = true;
    inline const static bool default_save_plot_solution = false;
    inline const static bool default_use_stochasticity = true;
    inline const static double default_epsilon = 0.95;
    inline const static double default_mean_percent = 10;
    inline const static double default_task_arena_size = 1.0;
    inline const static int json_battery_task = 0;

    template<typename T>
    static T loadSetupFromJson(const json& json_data, const std::string& field_name, const T& default_value);
    static void loadTasksFromJson(const json& json_data, MrtaConfig& config_object);
    static void loadRobotsFromJson(const json& json_data, MrtaConfig& config_object);
    // static void loadPathSigmas(const json& json_data, MrtaConfig& config_object);
    static void loadSkillDegradationFromJson(const json& json_data, MrtaConfig& config_object);
};