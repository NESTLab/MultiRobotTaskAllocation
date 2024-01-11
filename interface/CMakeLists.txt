add_library(mrta_interface
  mrta_interface.cpp)

target_link_libraries(
  mrta_interface
  mrta_utilities
  ${catkin_LIBRARIES}
)
