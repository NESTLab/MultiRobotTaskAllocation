add_library(mrta_utilities
  mrta_json_parser.cpp
  )

target_link_libraries(
  mrta_utilities
  nlohmann_json
  ${catkin_LIBRARIES}
)
