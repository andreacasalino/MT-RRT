cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

project(MT-RRT)

set(MT_RRT_SCRIPTS ${CMAKE_CURRENT_SOURCE_DIR}/script)

include(${MT_RRT_SCRIPTS}/Fetch.cmake)
include(${MT_RRT_SCRIPTS}/Terraform.cmake)

if(DEFINED PYTHON_EXECUTABLE)
    # indeed, this env var is defined by the setup.py script
    # if here the package is being built in order to generate a wheel
    set(PYTHON_CMD ${PYTHON_EXECUTABLE})
else()
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
    find_package(PythonLibs 3 REQUIRED)
    set(PYTHON_CMD "${Python3_EXECUTABLE}")
endif()
set(COMMON_KIT_PATH ${CMAKE_SOURCE_DIR}/script)

option(LIB_OPT "Compile shared libraries (ON) or static (OFF)" OFF)

option(BUILD_MT_RRT_TESTS "Build MT-RRT tests" OFF)
option(BUILD_MT_RRT_SAMPLES "Build MT-RRT samples" ON)

set(MT_RRT_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(extern)

Terraform(src)
Terraform(problems)

option(BUILD_MT_RRT_README_SAMPLE "" OFF)
if(BUILD_MT_RRT_README_SAMPLE)
    add_executable(ReadMeExample ReadMeExample.cpp)
    target_link_libraries(ReadMeExample MT-RRT-multi-threaded)
endif()
