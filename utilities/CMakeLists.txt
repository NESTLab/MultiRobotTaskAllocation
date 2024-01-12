add_library(mrta_utilities
  mrta_json_parser.cpp
  mrta_json_writer.cpp
  )

target_link_libraries(
  mrta_utilities
  nlohmann_json
  ${catkin_LIBRARIES}
)
