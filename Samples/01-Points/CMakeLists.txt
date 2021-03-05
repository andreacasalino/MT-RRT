add_subdirectory(problem)

add_executable(Sample01_Points Sample01.cpp ${SOURCES})
target_link_libraries(Sample01_Points PUBLIC MT-RRT-Samples-Points)
CustomCommandSample(Sample01_Points PointProblem)

add_executable(Sample02_Points Sample02.cpp ${SOURCES})
target_link_libraries(Sample02_Points PUBLIC MT-RRT-Samples-Points)
CustomCommandSample(Sample02_Points PointProblem)
