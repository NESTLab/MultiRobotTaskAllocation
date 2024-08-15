add_executable(mrta_example 
  usage_example.cpp
  )

target_link_libraries(
  mrta_example
  mrta_utilities
  mrta_interface
  mrta_solvers
  ${catkin_LIBRARIES}
)

add_executable(mrta_decentralized_example 
  decentralized_example.cpp
  )

target_link_libraries(
  mrta_decentralized_example
  mrta_utilities
  mrta_interface
  mrta_solvers
  ${catkin_LIBRARIES}
)

