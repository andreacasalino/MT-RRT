add_subdirectory(problem)

add_executable(Samples_Points_01 Sample01.cpp ${SOURCES})
target_link_libraries(Samples_Points_01 PUBLIC Samples-Points)
CustomCommandSample(Samples_Points_01 PointProblem)

add_executable(Samples_Points_02 Sample02.cpp ${SOURCES})
target_link_libraries(Samples_Points_02 PUBLIC Samples-Points)
CustomCommandSample(Samples_Points_02 PointProblem)
