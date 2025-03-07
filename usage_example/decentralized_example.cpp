#include <mrta_interface/mrta_interface.h>
#include <mrta_solvers/mrta_heuristic_solver.h>
#include <mrta_solvers/mrta_sorted_solver.h>
#include <mrta_utilities/mrta_json_parser.h>
#include <mrta_utilities/mrta_json_writer.h>

/**
 * @brief SingleRobot
 * This class simulates the behavior of each robot. It will calculate the
 * schedule for the current 'timestep'. The user will need to do three things
 * with this class:
 *
 * 1. Call `step` function to calculate the schedule. However, each robot will
 * only calculate its own schedule.
 *
 * 2. To inform each robot about the schedules calculated by its neighbours, the
 * user will need to implement a function similar to
 * `updateNeighboursEstimates()` where the solution object is populated with
 * each others' solutions.
 *
 * 3. Check if the robot scheudle has settled.
 *
 */
class SingleRobot {
public:
  /**
   * @brief Construct a new Single Robot object
   *
   * @param mrta_config config object to the problem descriptor config
   * @param robot_name  this specific robot's name
   * @param robot_id    index number for this robot in the config's robot vector
   */
  SingleRobot(const MrtaConfig::CompleteConfig &mrta_config,
              const std::string &robot_name, int robot_id)
      : mrta_config(mrta_config), robot_name(robot_name), robot_id(robot_id) {
    mrta_interface = new MrtaInterface();
    bool health_check = mrta_interface->healthCheckConfig(mrta_config);
    if (!health_check) {
      throw("[ERROR] | The Setup failed Health Check. Please make sure that "
            "the config file is correct");
    }
    mrta_interface->setMrtaSolverMethod(mrta_config.solver_info, robot_name,
                                        robot_id);
  }

  ~SingleRobot() { delete mrta_interface; }

  /**
   * @brief Step function to execute a step on a robot. In this example, the
   * step will only calculate the schedule.
   *
   * NOTE: Each robot will only populate its own schedule in the solution
   * object. User must implement functions to update robot's schedules to each
   * other
   *
   * @param solution Result solution will be updated in this argument
   */
  MrtaSolution::CompleteSolution
  step(MrtaSolution::CompleteSolution solution) const {
    // debugPrintSolution(solution);
    mrta_interface->solveMrtaProblem(mrta_config, solution);
    return solution;
  };

  void communicate(MrtaSolution::CompleteSolution &solution) {
    mrta_interface->communicateSolutionToAgents(solution);
  }

  /**
   * @brief Function to inform all robots each others' schedules
   *
   * @param neighbor_robots_solutions Vector to maintain individual solutions
   */
  void updateNeighboursEstimates(std::vector<MrtaSolution::CompleteSolution>
                                     neighbor_robots_solutions) const {
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

  /**
   * @brief Check if the robot schedule has converged
   *
   * @return true   Robot schedule has settled
   * @return false  Robot is still making changes
   */
  inline bool hasConverged() const {
    return mrta_interface->checkConvergence();
  }

  /**
   * @brief Debugging function to pring the solution
   *
   * @param solution Solution to be printed
   */
  void
  debugPrintSolution(const MrtaSolution::CompleteSolution &solution) const {
    mrta_interface->debugPrintSolution(solution);
  }

private:
  MrtaInterface *mrta_interface;
  MrtaConfig::CompleteConfig mrta_config;
  const std::string robot_name;
  int robot_id;
};

int main(int argc, char const *argv[]) {

  // Get the config by parsing a json file
  // NOTE: Alternatively, user can also input data to config without using the
  // json parser.
  MrtaConfig::CompleteConfig mrta_config;
  MrtaJsonParser::parseJsonFile("usage_example/usage_example.json",
                                mrta_config);

  // MrtaInterface interf;
  // interf.debugPrintConfigCompleteConfig(mrta_config);

  // Vector to maintain independent robot objects
  std::vector<SingleRobot> robot_solver_vector;
  robot_solver_vector.reserve(mrta_config.setup.number_of_robots);

  // Initializing robot objects in the vector
  for (int i = 0; i < mrta_config.setup.number_of_robots; ++i) {
    std::string robot_name = mrta_config.setup.all_robot_names.at(i);
    robot_solver_vector.emplace_back(mrta_config, robot_name, i);
  }

  // Vector to maintain the solution produced by each of the robot
  std::vector<MrtaSolution::CompleteSolution> collective_solution(
      mrta_config.setup.number_of_robots);
  MrtaSolution::CompleteSolution last_collective_solution;

  // Check if all the robots have converged.
  // Note: Convergence of ALL robots is necessary in some edge cases
  bool all_converged = false;
  while (!all_converged) {
    // This needs to be reset to true for the following AND condition to work.
    all_converged = true;
    for (int i = 0; i < mrta_config.setup.number_of_robots; ++i) {
      // This is where the actual schedule is calculated
      collective_solution.at(i) =
          robot_solver_vector.at(i).step(last_collective_solution);
      // Check if the robot thinks it has converged. Here, the AND condition is
      // necessary to make sure that all the robots have converged.
      all_converged = all_converged && robot_solver_vector.at(i).hasConverged();
      // Printing the solution for debugging purpose.
      // robot_solver_vector.at(i).debugPrintSolution(collective_solution.at(i));
    }
    
    // Inform all robots about others' solutions
    
    for (int i = 0; i < mrta_config.setup.number_of_robots; ++i) {
      std::string ith_name = mrta_config.setup.all_robot_names.at(i);
      last_collective_solution.robot_task_schedule_map[ith_name] =
      collective_solution.at(i).robot_task_schedule_map[ith_name];
    }
    for (int i = 0; i < mrta_config.setup.number_of_robots; ++i) {
      robot_solver_vector.at(i).communicate(last_collective_solution);
    }
    robot_solver_vector.at(0).debugPrintSolution(last_collective_solution);
    std::cout
    << "==============================================================="
        << std::endl;
  }

  MrtaJsonWriter::writeJsonFile(collective_solution.front(),
                                "usage_example/solution_file.json");
}