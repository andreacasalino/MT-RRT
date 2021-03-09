add_subdirectory(problem)

add_executable(Samples_Points_open Sample-open.cpp ${SOURCES})
target_link_libraries(Samples_Points_open PUBLIC Samples-Points)
CopyPythonCode(Samples_Points_open)

add_executable(Samples_Points_cluttered Sample-cluttered.cpp ${SOURCES})
target_link_libraries(Samples_Points_cluttered PUBLIC Samples-Points)
CopyPythonCode(Samples_Points_cluttered)
