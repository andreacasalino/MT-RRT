project(MT-RRT-SampleNavigation)

add_subdirectory(problem)

# macro(ADD_CONFIG_FOLDER SAMPLE_NAME)
# target_compile_definitions(${SAMPLE_NAME}
# PUBLIC
	# -D CONFIG_FOLDER="${CMAKE_CURRENT_SOURCE_DIR}"
# )
# endmacro()

# add_executable(SampleManipulators-01-single-arm SampleManipulators-01-single-arm.cpp ${SOURCES})
# target_link_libraries(SampleManipulators-01-single-arm PUBLIC Samples-Manipulators)
# CopyPythonVisualizer(SampleManipulators-01-single-arm)
# ADD_CONFIG_FOLDER(SampleManipulators-01-single-arm)

# add_executable(SampleManipulators-02-single-arm-compare SampleManipulators-02-single-arm-compare.cpp ${SOURCES})
# target_link_libraries(SampleManipulators-02-single-arm-compare PUBLIC Samples-Manipulators)
# CopyPythonVisualizer(SampleManipulators-02-single-arm-compare)
# ADD_CONFIG_FOLDER(SampleManipulators-02-single-arm-compare)

# add_executable(SampleManipulators-03-multi-arms SampleManipulators-03-multi-arms.cpp ${SOURCES})
# target_link_libraries(SampleManipulators-03-multi-arms PUBLIC Samples-Manipulators)
# CopyPythonVisualizer(SampleManipulators-03-multi-arms)
# ADD_CONFIG_FOLDER(SampleManipulators-03-multi-arms)
