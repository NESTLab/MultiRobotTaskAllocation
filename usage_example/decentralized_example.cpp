#include <mrta_interface/mrta_interface.h>
#include <mrta_solvers/mrta_heuristic_solver.h>
#include <mrta_solvers/mrta_sorted_solver.h>
#include <mrta_utilities/mrta_json_parser.h>
#include <mrta_utilities/mrta_json_writer.h>

class SingleRobot {
public:
  SingleRobot(const MrtaConfig::CompleteConfig &mrta_config,
              const std::string &robot_name, int robot_id)
      : mrta_config(mrta_config), robot_name(robot_name), robot_id(robot_id) {
    mrta_interface = new MrtaInterface();
    bool health_check = mrta_interface->healthCheckConfig(mrta_config);
    if (!health_check) {
      throw("[ERROR] | The Setup failed Health Check");
    }
    mrta_interface->setMrtaSolverMethod(mrta_config.solver_info, robot_name,
                                        robot_id);
  }

  ~SingleRobot() { delete mrta_interface; }

  void step(MrtaSolution::CompleteSolution &solution) const {
    mrta_interface->solveMrtaProblem(mrta_config, solution);
  };

  void updateNeighboursEstimates(std::vector<MrtaSolution::CompleteSolution>
                                     &neighbor_robots_solutions) const {
    for (int i = 0; i < mrta_config.setup.number_of_robots; ++i) {
      if (i == robot_id)
        continue;
      std::string neighbour_robot_name =
          mrta_config.setup.all_robot_names.at(i);
      neighbor_robots_solutions.at(robot_id)
          .robot_task_schedule_map[neighbour_robot_name] =
          neighbor_robots_solutions.at(i)
              .robot_task_schedule_map[neighbour_robot_name];
    }
  };

  inline bool hasConverged() const {
    return mrta_interface->checkConvergence();
  }

  // inline double getConvergenceThreshold() const {
  //   return convergance_threshold;
  // };

  // inline void setConvergenceThreshold(double threshold) {
  //   convergance_threshold = threshold;
  // };

  void
  debugPrintSolution(const MrtaSolution::CompleteSolution &solution) const {
    mrta_interface->debugPrintSolution(solution);
  }

private:
  // double convergance_threshold;
  MrtaInterface *mrta_interface;
  MrtaConfig::CompleteConfig mrta_config;
  const std::string robot_name;
  int robot_id;
};

int main(int argc, char const *argv[]) {

  // Get the config by parsing a json file
  MrtaConfig::CompleteConfig mrta_config;
  MrtaJsonParser::parseJsonFile("usage_example/usage_example.json",
                                mrta_config);
  std::vector<SingleRobot> robot_solver_vector;
  robot_solver_vector.reserve(mrta_config.setup.number_of_robots);
  for (int i = 0; i < mrta_config.setup.number_of_robots; ++i) {
    std::string robot_name = mrta_config.setup.all_robot_names.at(i);
    robot_solver_vector.emplace_back(mrta_config, robot_name, i);
  }

  std::vector<MrtaSolution::CompleteSolution> collective_solution(
      mrta_config.setup.number_of_robots);
  bool all_converged = false;
  bool once = true;
  while(!all_converged) {
  // while (once) {
    once = false;
    for (int i = 0; i < mrta_config.setup.number_of_robots; ++i) {
      robot_solver_vector.at(i).step(collective_solution.at(i));
      all_converged = all_converged && robot_solver_vector.at(i).hasConverged();
      robot_solver_vector.at(i).debugPrintSolution(collective_solution.at(i));
    }
    for (const auto &robot : robot_solver_vector) {
      robot.updateNeighboursEstimates(collective_solution);
    }
  }
}