project(MT-RRT-SamplePoints)

add_subdirectory(problem)

add_executable(SamplePointsA SamplePointsA.cpp)
target_link_libraries(SamplePointsA PUBLIC Samples-Points)
CopyPythonVisualizer(SamplePointsA)

add_executable(SamplePointsA-profile SamplePointsA-profile.cpp)
target_link_libraries(SamplePointsA-profile PUBLIC Samples-Points)
CopyPythonVisualizer(SamplePointsA-profile)
