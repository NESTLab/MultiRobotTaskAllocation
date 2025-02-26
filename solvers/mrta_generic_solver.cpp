#include <mrta_solvers/mrta_generic_solver.h>

void MrtaGenericSolver::initializeTravelTimeTensor() {
  int number_of_robots = mrta_complete_config->setup.number_of_robots;
  int number_of_destinations =
      mrta_complete_config->setup.number_of_destinations;
  robot_travel_times_vector.resize(number_of_robots);
  for (size_t i = 0; i < number_of_robots; i++) {
    robot_travel_times_vector.at(i) =
        Eigen::MatrixXd::Zero(number_of_destinations, number_of_destinations);
    putTravelTimesForRobot(i, robot_travel_times_vector.at(i));
  }
}

void MrtaGenericSolver::putTravelTimesForRobot(
    int robot_id, Eigen::MatrixXd &ret_i_travel_time_matrix) {
  int number_of_destinations =
      mrta_complete_config->setup.number_of_destinations;

  // FIRST and LAST destinations are START and END, hence skipping them
  for (size_t task_id = START_ID + 1; task_id < number_of_destinations - 1;
       task_id++) {
    std::map<std::string, MrtaConfig::Robot>::const_iterator robot_itr =
        mrta_complete_config->robots_map.find(
            mrta_complete_config->setup.all_robot_names.at(robot_id));
    std::map<std::string, MrtaConfig::Task>::const_iterator task_itr =
        mrta_complete_config->tasks_map.find(
            mrta_complete_config->setup.all_destination_names.at(task_id));
    double robot_velocity =
        (robot_itr->second.velocity != 0) ? robot_itr->second.velocity : 1;

    ret_i_travel_time_matrix(START_ID, task_id) = getPureTravelTime(
        robot_itr->second.position, task_itr->second.position, robot_velocity);
    ret_i_travel_time_matrix(END_ID, task_id) =
        getPureTravelTime(robot_itr->second.desired_end_position,
                          task_itr->second.position, robot_velocity);
    ret_i_travel_time_matrix(task_id, START_ID) =
        ret_i_travel_time_matrix(START_ID, task_id);
    ret_i_travel_time_matrix(task_id, END_ID) =
        ret_i_travel_time_matrix(END_ID, task_id);
    ret_i_travel_time_matrix(START_ID, START_ID) =
        std::numeric_limits<double>::infinity();
    ret_i_travel_time_matrix(END_ID, END_ID) =
        std::numeric_limits<double>::infinity();
    ret_i_travel_time_matrix(START_ID, END_ID) = getPureTravelTime(
        robot_itr->second.position, robot_itr->second.desired_end_position,
        robot_velocity);
    ret_i_travel_time_matrix(END_ID, START_ID) =
        ret_i_travel_time_matrix(START_ID, END_ID);
    ret_i_travel_time_matrix(task_id, task_id) =
        std::numeric_limits<double>::infinity();
    for (size_t second_task_id = task_id + 1;
         second_task_id < number_of_destinations - 1; second_task_id++) {
      std::map<std::string, MrtaConfig::Task>::const_iterator second_task_itr =
          mrta_complete_config->tasks_map.find(
              mrta_complete_config->setup.all_destination_names.at(
                  second_task_id));
      ret_i_travel_time_matrix(task_id, second_task_id) =
          getPureTravelTime(task_itr->second.position,
                            second_task_itr->second.position, robot_velocity);
      ret_i_travel_time_matrix(second_task_id, task_id) =
          ret_i_travel_time_matrix(task_id, second_task_id);
    }
  }
}

double MrtaGenericSolver::getPureTravelTime(
    const MrtaConfig::Position &task_1_position,
    const MrtaConfig::Position &task_2_position, double velocity) {
  double x_diff = task_1_position.pos_x - task_2_position.pos_x;
  double y_diff = task_1_position.pos_y - task_2_position.pos_y;
  double distance = std::sqrt(std::abs(x_diff * x_diff + y_diff * y_diff));
  return distance / velocity;
}
