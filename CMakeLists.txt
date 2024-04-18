cmake_minimum_required(VERSION 3.20)

set(PROJECT_NAME pinocchio_visualizers)
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

add_project_dependency(pinocchio 2.7.0 REQUIRED)

set(HEADERS include/pinocchio-visualizers/base-visualizer.hpp
            include/pinocchio-visualizers/base-visualizer.hxx)

add_library(${PROJECT_NAME} INTERFACE ${HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX PUBLIC_HEADER
                                                                     "${HEADERS}")
target_link_libraries(${PROJECT_NAME} INTERFACE pinocchio::pinocchio)
target_include_directories(${PROJECT_NAME} INTERFACE $<INSTALL_INTERFACE:include>)

install(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME}
        PUBLIC_HEADER DESTINATION include/${PROJECT_NAME})

add_subdirectory(tests)
