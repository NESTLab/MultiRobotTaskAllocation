#include <mrta_utilities/mrta_json_parser.h>
// #include <mrta_interface/mrta_interface.h>
// #include <mrta_solvers/mrta_milp_solver.h>

int main(int argc, char const *argv[])
{
  std::shared_ptr<MrtaConfig::CompleteConfig> mrta_config_ptr = MrtaJsonParser::parseJsonFile("experiments/testing_setups/test_json_parser.json");
  // MrtaInterface mrta_interface;

  // mrta_interface.setMrtaSolverMethod(std::make_shared<MrtaMilpSolver>());
  // mrta_interface.solveMrtaProblem(json_config);

  return 0;
}


