
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was mrta_cppConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../usr/local" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
      message(STATUS "${_NAME} library NOT found")
    else(NOT ${_NAME}_${comp}_FOUND)
      message(STATUS "${_NAME} library found: ${${_NAME}_LIBRARY}")
    endif()
  endforeach()
endmacro()

####################################################################################

# Include directories for the library
set(mrta_cpp_INCLUDE_DIRS
    /usr/local/include
)
