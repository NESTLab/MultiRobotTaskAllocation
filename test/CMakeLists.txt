add_executable(mrta_test
  test_json_parser.cpp
  )

target_link_libraries(
  mrta_test
  mrta_utilities
  ${catkin_LIBRARIES}
)
