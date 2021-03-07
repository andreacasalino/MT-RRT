add_subdirectory(problem)

add_executable(Samples_Points_open Sample-open.cpp ${SOURCES})
target_link_libraries(Samples_Points_open PUBLIC Samples-Points)
CustomCommandSample(Samples_Points_open PointProblem)

add_executable(Samples_Points_cluttered Sample-cluttered.cpp ${SOURCES})
target_link_libraries(Samples_Points_cluttered PUBLIC Samples-Points)
CustomCommandSample(Samples_Points_cluttered PointProblem)
