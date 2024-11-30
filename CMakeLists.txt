cmake_minimum_required(VERSION 3.20)

set(PROJECT_NAME pinocchio-visualizers)
set(PROJECT_URL "")
set(PROJECT_DESCRIPTION "")
set(PROJECT_CUSTOM_HEADER_DIR ${PROJECT_NAME})
set(PROJECT_CUSTOM_HEADER_EXTENSION "hpp")
set(PROJECT_USE_CMAKE_EXPORT True)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS True)

include(cmake/base.cmake)

project(${PROJECT_NAME} ${PROJECT_ARGS})

set(CMAKE_VERBOSE_MAKEFILE False)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(
    CACHE CMAKE_BUILD_TYPE
    PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo"
  )
endif()

add_project_dependency(pinocchio 2.7.0 REQUIRED)
add_project_dependency(eigenpy) # not required

set(
  PINVIZ_SOURCES
  src/pinocchio-visualizers/base-visualizer.hpp
  src/pinocchio-visualizers/base-visualizer.cpp
)
message(STATUS "Source files: ${PINVIZ_SOURCES}")

add_library(${PROJECT_NAME} INTERFACE)
set_target_properties(
  ${PROJECT_NAME}
  PROPERTIES LINKER_LANGUAGE CXX PUBLIC_HEADER "${PINVIZ_SOURCES}"
)
target_link_libraries(${PROJECT_NAME} INTERFACE pinocchio::pinocchio)
target_include_directories(
  ${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}
)

if(eigenpy_FOUND)
  set(PY_HEADERS src/pinocchio-visualizers/python/visitor.hpp)
  add_library(${PROJECT_NAME}_python INTERFACE)
  add_library(${PROJECT_NAME}::python ALIAS ${PROJECT_NAME}_python)
  set_target_properties(
    ${PROJECT_NAME}_python
    PROPERTIES PUBLIC_HEADER "${PY_HEADERS}"
  )
  target_link_libraries(${PROJECT_NAME}_python INTERFACE eigenpy::eigenpy)

  install(
    TARGETS ${PROJECT_NAME}_python
    EXPORT ${TARGETS_EXPORT_NAME}
    PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}/python
  )
endif(eigenpy_FOUND)

add_subdirectory(tests)
