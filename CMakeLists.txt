cmake_minimum_required(VERSION 3.15...3.27)
project(cpp_project) 

find_package(Python 3.8 COMPONENTS Interpreter Development REQUIRED)
# if module cannot be loaded this might be due to the usage of a different version 
# of python than the one found by CMake


if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ext/pybind11)

# Minimal example python binding
pybind11_add_module(my_ext ./src/my_ext.cpp)
