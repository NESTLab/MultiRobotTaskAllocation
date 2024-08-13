#pragma once
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/CXX11/Tensor>
#include <memory>
#include <mrta_utilities/mrta_config.h>
#include <mrta_utilities/mrta_solution.h>

class MrtaGenericSolver {
public:
  MrtaGenericSolver(/* args */){};
  ~MrtaGenericSolver(){
    // delete mrta_complete_config;
  };

protected:
  virtual void
  solveMrtaProblem(const MrtaConfig::CompleteConfig &mrta_complete_config,
                   MrtaSolution::CompleteSolution &ret_complete_solution) = 0;

  virtual void updateWorldStatus() = 0;

  bool limited_info_mode = false;

  /**
   * @brief Set the Limited Info Mode object
   * Defines if this solver is being used by the CRA methods. If that is the
   * case, then the skill matrix calculation should be skipped and should be
   * taken as an input from the user.
   *
   * @param limited_info_mode_in
   */
  void setLimitedInfoMode(bool limited_info_mode_in) {
    limited_info_mode = limited_info_mode_in;
  }

  template <typename M_k, typename M_v>
  inline typename std::map<M_k, M_v>::const_iterator
  getMapIterator(const std::map<M_k, M_v> &const_map, const M_k &key) {
    return const_map.find(key);
  };

  int START_ID = 0;
  int END_ID = 0;
  bool config_initialized = false;

  void
  updateMrtaConfig(const MrtaConfig::CompleteConfig &mrta_complete_config_in) {
    try {
      config_initialized = true;
      mrta_complete_config = &mrta_complete_config_in;
      END_ID = mrta_complete_config->setup.number_of_destinations - 1;
      robot_task_id_attendance_sequence.resize(
          mrta_complete_config->setup.number_of_robots);
      for (auto &robot_task_att : robot_task_id_attendance_sequence) {
        robot_task_att = std::vector<int>(1, START_ID);
      }
      initializeTravelTimeTensor();
      task_start_time = std::vector<double>(
          mrta_complete_config->setup.number_of_destinations, 0.0);
      initializeTaskRequirementMatrix(mrta_complete_config_in);
    } catch (const std::exception &e) {
      std::cerr << e.what() << '\n';
    }
  };

  void initializeTaskRequirementMatrix(
      const MrtaConfig::CompleteConfig &mrta_complete_config_in) {
    int number_of_destinations =
        mrta_complete_config_in.setup.number_of_destinations;
    task_requirements_matrix = Eigen::MatrixXd::Zero(
        mrta_complete_config_in.setup.number_of_destinations,
        mrta_complete_config_in.setup.number_of_skills);
    for (int skill_id = 0;
         skill_id < mrta_complete_config->setup.all_skill_names.size();
         skill_id++) {
      std::string skill_name =
          mrta_complete_config->setup.all_skill_names.at(skill_id);
      for (int j = START_ID + 1;           // Skipping the START task
           j < number_of_destinations - 1; // Skipping the END task
           ++j) {
        std::map<std::string, MrtaConfig::Task>::const_iterator task_info_itr =
            mrta_complete_config_in.tasks_map.find(
                mrta_complete_config_in.setup.all_destination_names.at(j));
        if (task_info_itr != mrta_complete_config_in.tasks_map.end()) {
          std::map<std::string, double>::const_iterator task_skill_itr =
              task_info_itr->second.skillset.find(skill_name);
          if (task_skill_itr != task_info_itr->second.skillset.end()) {
            task_requirements_matrix(j, skill_id) = task_skill_itr->second;
          } else {
            task_requirements_matrix(j, skill_id) = 0.0;
          }
        } else {
          throw std::runtime_error(
              "Task " +
              mrta_complete_config_in.setup.all_destination_names.at(j) +
              " not found in MRTA Config");
        }
      }
      task_requirements_matrix(START_ID, skill_id) = 0;
      task_requirements_matrix(END_ID, skill_id) = 0;
    }
  }

  MrtaConfig::CompleteConfig const *mrta_complete_config;
  Eigen::MatrixXd task_requirements_matrix;
  std::vector<Eigen::MatrixXd> robot_travel_times_vector;

  std::vector<std::vector<int>> robot_task_id_attendance_sequence;
  std::vector<double> task_start_time;
  std::map<std::string, std::map<std::string, double>>
      robot_task_attendance_times_map;

  void initializeTravelTimeTensor();

  void putTravelTimesForRobot(int robot_id,
                            Eigen::MatrixXd &ret_i_travel_time_matrix);
  double getPureTravelTime(const MrtaConfig::Position &task_1_position,
                         const MrtaConfig::Position &task_2_position,
                         double velocity = 1.0);

  friend class MrtaInterface;
};
