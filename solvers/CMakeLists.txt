add_library(mrta_solvers SHARED
  mrta_generic_solver.cpp
  mrta_sorted_solver.cpp
  mrta_heuristic_solver.cpp
  # mrta_milp_solver.cpp
  )

target_link_libraries(
  mrta_solvers
  mrta_utilities
  ${catkin_LIBRARIES}
)
