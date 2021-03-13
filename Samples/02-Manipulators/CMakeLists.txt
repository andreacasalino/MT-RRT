add_subdirectory(problem)

add_executable(SampleManipulators-01-single-arm SampleManipulators-01-single-arm.cpp ${SOURCES})
target_link_libraries(SampleManipulators-01-single-arm PUBLIC Samples-Manipulators)
CopyPythonCode(SampleManipulators-01-single-arm)

add_executable(SampleManipulators-02-multi-arms SampleManipulators-02-multi-arms.cpp ${SOURCES})
target_link_libraries(SampleManipulators-02-multi-arms PUBLIC Samples-Manipulators)
CopyPythonCode(SampleManipulators-02-multi-arms)
