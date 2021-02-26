CollectSourceFiles(${CMAKE_CURRENT_SOURCE_DIR} SOURCES)
GroupSources(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(Sample01_Point2D ${SOURCES})
target_link_libraries(Sample01_Point2D PUBLIC MT-RRT-Problems MT-RRT-Samples-Geometry)