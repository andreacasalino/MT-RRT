cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/MakeTest.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/MakeLibrary.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FetchJSONlib.cmake)

project(MT-RRT)

option(LIB_OPT "Compile shared libraries (ON) or static (OFF)" OFF)

option(BUILD_MT_RRT_TESTS "Build MT-RRT tests" OFF)
if(BUILD_MT_RRT_TESTS)
    message("fetching catch2") 
    include(FetchContent)
    FetchContent_Declare(
        catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        4ff8b27bb6bed4b8b309e56cd269b4f1fbc24e89
    )
    FetchContent_MakeAvailable(catch2)

    option(ENABLE_MT_RRT_TESTS_LOGGING "Enable log generations when running tests" OFF)
endif()

option(BUILD_MT_RRT_SAMPLES "Build MT-RRT samples" ON)

if(BUILD_MT_RRT_TESTS OR BUILD_MT_RRT_SAMPLES)
    FECTH_JSON_LIB()
    add_subdirectory(utils)
endif()

add_subdirectory(src)

if(BUILD_MT_RRT_SAMPLES)
   add_subdirectory(samples)
endif()
