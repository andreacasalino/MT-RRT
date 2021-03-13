add_subdirectory(problem)

add_executable(SamplePoints-01-single-steer SamplePoints-01-single-steer.cpp ${SOURCES})
target_link_libraries(SamplePoints-01-single-steer PUBLIC Samples-Points)
CopyPythonCode(SamplePoints-01-single-steer)

add_executable(SamplePoints-02-multi-steer SamplePoints-02-multi-steer.cpp ${SOURCES})
target_link_libraries(SamplePoints-02-multi-steer PUBLIC Samples-Points)
CopyPythonCode(SamplePoints-02-multi-steer)
