#include <mrta_solvers/mrta_milp_solver.h>

std::shared_ptr<MrtaSolution::CompleteSolution> MrtaMilpSolver::solveMrtaProblem() {
    MrtaSolution::CompleteSolution empty_sol;
    return std::make_shared<MrtaSolution::CompleteSolution>(empty_sol);
}