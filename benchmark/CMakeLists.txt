file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.h ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_executable(MT-RRT-benchmark ${SOURCES})
target_link_libraries(MT-RRT-benchmark PUBLIC
    MT-RRT-test-scenarios
    gtest
    gtest_main
)

if (CMAKE_GENERATOR MATCHES "Visual Studio")
    SET(MT-RRT-benchmark-bin-path  "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/MT-RRT-benchmark" CACHE INTERNAL "MT-RRT-benchmark-bin-path")
else()
    SET(MT-RRT-benchmark-bin-path  "${CMAKE_CURRENT_BINARY_DIR}/MT-RRT-benchmark" CACHE INTERNAL "MT-RRT-benchmark-bin-path")
endif()

add_custom_target(MT-RRT-benchmark-StandardPlanner
DEPENDS MT-RRT-benchmark
COMMAND ${MT-RRT-benchmark-bin-path} --gtest_filter=*StandardPlanner*
# TODO results visualization commands if needed
)

#TODO custom command to create flame diagrams
