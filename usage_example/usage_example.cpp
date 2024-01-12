#include <mrta_interface/mrta_interface.h>
#include <mrta_solvers/mrta_sorted_solver.h>
#include <mrta_utilities/mrta_json_parser.h>
// #include <mrta_solvers/mrta_milp_solver.h>

int main(int argc, char const *argv[]) {
  std::shared_ptr<MrtaConfig::CompleteConfig> mrta_config_ptr =
      MrtaJsonParser::parseJsonFile(
          "usage_example/usage_example.json");
  MrtaInterface mrta_interface;

  mrta_interface.debugPrintConfigCompleteConfig(mrta_config_ptr);
  std::cout << mrta_interface.healthCheckConfig(mrta_config_ptr) << std::endl;

  mrta_interface.setMrtaSolverMethod(std::make_shared<MrtaSortedSolver>());

  std::shared_ptr<const MrtaSolution::CompleteSolution> const solution =
      mrta_interface.solveMrtaProblem(mrta_config_ptr);

  mrta_interface.debugPrintSolution(solution);

  return 0;
}
