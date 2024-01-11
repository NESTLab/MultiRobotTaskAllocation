cmake_minimum_required(VERSION 3.0)
project(mrta_cpp)

option(CXX "enable C++ compilation" ON)
# Enforce C++-standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Force Release build if nothing was specified
if((NOT CMAKE_BUILD_TYPE) OR (CMAKE_BUILD_TYPE STREQUAL ""))
  set(CMAKE_BUILD_TYPE
    "Release" CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel ..."
    FORCE)
endif((NOT CMAKE_BUILD_TYPE) OR (CMAKE_BUILD_TYPE STREQUAL ""))

# Special behavior if debug builds were chosen
if(CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  # Enable Asan by default
  set(ASAN_COMPILER_FLAGS " -fsanitize=address -fno-optimize-sibling-calls -fno-omit-frame-pointer")
  set(ASAN_LINKER_FLAGS " -fsanitize=address")
  string(APPEND CMAKE_C_FLAGS_DEBUG            ${ASAN_COMPILER_FLAGS})
  string(APPEND CMAKE_CXX_FLAGS_DEBUG          ${ASAN_COMPILER_FLAGS})
  string(APPEND CMAKE_C_FLAGS_RELWITHDEBINFO   ${ASAN_COMPILER_FLAGS})
  string(APPEND CMAKE_CXX_FLAGS_RELWITHDEBINFO ${ASAN_COMPILER_FLAGS})
  set(CMAKE_EXE_LINKER_FLAGS_DEBUG             ${ASAN_LINKER_FLAGS})
  set(CMAKE_SHARED_LINKER_FLAGS_DEBUG          ${ASAN_LINKER_FLAGS})
  set(CMAKE_MODULE_LINKER_FLAGS_DEBUG          ${ASAN_LINKER_FLAGS})
  set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO    ${ASAN_LINKER_FLAGS})
  set(CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO ${ASAN_LINKER_FLAGS})
  set(CMAKE_MODULE_LINKER_FLAGS_RELWITHDEBINFO ${ASAN_LINKER_FLAGS})
endif(CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")

# Standard include directory
include_directories(${CMAKE_SOURCE_DIR})

find_package (nlohmann_json REQUIRED NO_MODULE)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(GUROBI REQUIRED)
include_directories(${GUROBI_INCLUDE_DIRS})
link_libraries(${GUROBI_CXX_LIBRARY} ${GUROBI_LIBRARY})

include_directories(
  utilities/include
  interface/include
  solvers/include
  ${catkin_INCLUDE_DIRS}
)

# Compile the code
add_subdirectory(utilities)
# add_subdirectory(interface)
# add_subdirectory(solvers)
add_subdirectory(test)
