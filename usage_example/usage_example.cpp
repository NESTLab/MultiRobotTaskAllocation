#include <mrta_interface/mrta_interface.h>
#include <mrta_solvers/mrta_heuristic_solver.h>
#include <mrta_solvers/mrta_decentralized_hss_solver.h>
#include <mrta_solvers/mrta_sorted_solver.h>
#include <mrta_utilities/mrta_json_parser.h>
#include <mrta_utilities/mrta_json_writer.h>

int main(int argc, char const *argv[]) {

  // Get the config by parsing a json file
  MrtaConfig::CompleteConfig mrta_config;
  MrtaJsonParser::parseJsonFile("usage_example/usage_example.json",
                                mrta_config);

  // Create an object for the interface class
  MrtaInterface mrta_interface;

  // Debug and check if the config file is being interpreted correctly
  mrta_interface.debugPrintConfigCompleteConfig(mrta_config);

  // Make sure that the config file is good enough to be used for solving
  bool health_check = mrta_interface.healthCheckConfig(mrta_config);
  if (!health_check){
    std::cout<<"[ERROR] | The Setup failed Health Check"<<std::endl;
    return -1;
  }

  // Set which method do you want to use for solving the problem
  mrta_interface.setMrtaSolverMethod(mrta_config.solver_info, "Robot_3", 2);

  // Solve the problem and provide the solution
  MrtaSolution::CompleteSolution solution;
  mrta_interface.solveMrtaProblem(mrta_config, solution);

  // Print and see what the result looks like
  mrta_interface.debugPrintSolution(solution);

  // Output the result to a json file
  MrtaJsonWriter::writeJsonFile(solution, "usage_example/solution_file.json");

  return 0;
}
