add_library(mrta_solvers MODULE
  mrta_generic_solver.cpp
  )

target_link_libraries(
  mrta_solvers
  mrta_utilities
  ${catkin_LIBRARIES}
)
