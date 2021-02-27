add_subdirectory(ProblemPoint)

add_executable(Sample01_Point2D Sample01.cpp ${SOURCES})
target_link_libraries(Sample01_Point2D PUBLIC MT-RRT-Samples-ProblemPoint)

add_executable(Sample02_Point2D Sample02.cpp ${SOURCES})
target_link_libraries(Sample02_Point2D PUBLIC MT-RRT-Samples-ProblemPoint)