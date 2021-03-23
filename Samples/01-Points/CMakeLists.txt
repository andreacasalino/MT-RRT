project(MT-RRT-SamplePoints)

add_subdirectory(problem)

add_executable(SamplePoints-01 SamplePoints-01.cpp)
target_link_libraries(SamplePoints-01 PUBLIC Samples-Points)
CopyPythonVisualizer(SamplePoints-01)

add_executable(SamplePoints-02-compare SamplePoints-02-compare.cpp)
target_link_libraries(SamplePoints-02-compare PUBLIC Samples-Points)
CopyPythonVisualizer(SamplePoints-02-compare)
