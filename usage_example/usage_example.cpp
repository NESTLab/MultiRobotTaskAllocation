#include <mrta_interface/mrta_interface.h>
#include <mrta_solvers/mrta_sorted_solver.h>
#include <mrta_solvers/mrta_heuristic_solver.h>
#include <mrta_solvers/mrta_sorted_solver.h>
#include <mrta_utilities/mrta_json_parser.h>
#include <mrta_utilities/mrta_json_writer.h>

int main(int argc, char const *argv[]) {

  // Get the config by parsing a json file
  std::shared_ptr<const MrtaConfig::CompleteConfig> const mrta_config_ptr =
      MrtaJsonParser::parseJsonFile("usage_example/usage_example.json");

  // Create an object for the interface class
  MrtaInterface mrta_interface;

  // Debug and check if the config file is being interpreted correctly
  // mrta_interface.debugPrintConfigCompleteConfig(mrta_config_ptr);

  // Make sure that the config file is good enough to be used for solving
  // std::cout << mrta_interface.healthCheckConfig(mrta_config_ptr) << std::endl;

  // Set which method do you want to use for solving the problem
  mrta_interface.setMrtaSolverMethod(std::make_shared<MrtaHeuristicSolver>());

  // Solve the problem and provide the solution
  std::shared_ptr<const MrtaSolution::CompleteSolution> const solution =
      mrta_interface.solveMrtaProblem(mrta_config_ptr);

  // Print and see what the result looks like
  mrta_interface.debugPrintSolution(solution);

  // Output the result to a json file
  MrtaJsonWriter::writeJsonFile(solution, "usage_example/solution_file.json");

  return 0;
}
