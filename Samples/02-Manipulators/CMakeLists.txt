add_subdirectory(problem)

add_executable(Sample01_Manipulator Sample01.cpp ${SOURCES})
target_link_libraries(Sample01_Manipulator PUBLIC MT-RRT-Samples-Manipulators)
CustomCommandSample(Sample01_Manipulator ManipulatorProblem)

add_executable(Sample02_Manipulator Sample02.cpp ${SOURCES})
target_link_libraries(Sample02_Manipulator PUBLIC MT-RRT-Samples-Manipulators)
CustomCommandSample(Sample02_Manipulator ManipulatorProblem)
