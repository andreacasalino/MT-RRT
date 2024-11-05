file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.h ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_executable(MT-RRT-benchmark ${SOURCES})
target_link_libraries(MT-RRT-benchmark PUBLIC
    MT-RRT-test-scenarios
    gtest
    gtest_main
)
