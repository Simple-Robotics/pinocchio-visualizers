cmake_minimum_required(VERSION 3.20)

set(PROJECT_NAME pinocchio_visualizers)
set(PROJECT_URL "")
set(PROJECT_DESCRIPTION "")
set(PROJECT_CUSTOM_HEADER_EXTENSION "hpp")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS True)

include(cmake-module/base.cmake)

project(${PROJECT_NAME} ${PROJECT_ARGS})

set(CMAKE_VERBOSE_MAKEFILE False)

find_package(pinocchio 2.7.0 REQUIRED)


set(HEADERS include/pinocchio-visualizers/base-visualizer.hpp
            include/pinocchio-visualizers/base-visualizer.hxx)

add_library(${PROJECT_NAME} INTERFACE ${HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME} INTERFACE pinocchio::pinocchio)

add_subdirectory(tests)
