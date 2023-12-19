project(my_project) # Replace 'my_project' with the name of your project
cmake_minimum_required(VERSION 3.15...3.27)
find_package(Python 3.8 COMPONENTS Interpreter Development REQUIRED)


if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ext/pybind11)

# C++ library for simulation

add_library(pbd_simulation 
  src/pbd/Context.cpp
  src/pbd/Context.h
  src/pbd/Particle.h
  src/pbd/Particle.cpp
  src/pbd/Vec2.cpp
  src/pbd/Vec2.h
)

target_include_directories(pbd_simulation PUBLIC src)

# Simple program to test the library 

add_executable(pbd_test
  src/main.cpp
) 

target_link_libraries(pbd_test
    PRIVATE
        pbd_simulation
)

# Python binding

pybind11_add_module(py_pbd_simulation 
  src/py_binding.cpp
)

target_link_libraries(py_pbd_simulation
    PRIVATE
        pbd_simulation
)

# Copy python script to build directory 

configure_file(src/particle_gui.py particle_gui.py COPYONLY)
