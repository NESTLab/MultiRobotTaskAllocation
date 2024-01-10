include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_library(mrta_json_parser mrta_json_parser.cpp)

target_link_libraries(
  mrta_json_parser
  nlohmann_json
  ${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  )
