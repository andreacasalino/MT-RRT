add_subdirectory(problem)

add_executable(Samples_Manipulators_single Sample-single-arm.cpp ${SOURCES})
target_link_libraries(Samples_Manipulators_single PUBLIC Samples-Manipulators)
CopyPythonCode(Samples_Manipulators_single)

add_executable(Samples_Manipulators_multi Sample-multi-arms.cpp ${SOURCES})
target_link_libraries(Samples_Manipulators_multi PUBLIC Samples-Manipulators)
CopyPythonCode(Samples_Manipulators_multi)
