project(MT-RRT-SampleManipulators)

add_subdirectory(problem)

macro(ADD_CONFIG_FOLDER SAMPLE_NAME)
target_compile_definitions(${SAMPLE_NAME}
PUBLIC
	-D CONFIG_FOLDER="${CMAKE_CURRENT_SOURCE_DIR}"
)
endmacro()

add_executable(SampleManipulatorsA SampleManipulatorsA.cpp)
target_link_libraries(SampleManipulatorsA PUBLIC Samples-Manipulators)
CopyPythonVisualizer(SampleManipulatorsA)
ADD_CONFIG_FOLDER(SampleManipulatorsA)

add_executable(SampleManipulatorsA-profile SampleManipulatorsA-profile.cpp)
target_link_libraries(SampleManipulatorsA-profile PUBLIC Samples-Manipulators)
CopyPythonVisualizer(SampleManipulatorsA-profile)
ADD_CONFIG_FOLDER(SampleManipulatorsA-profile)

add_executable(SampleManipulatorsB SampleManipulatorsB.cpp)
target_link_libraries(SampleManipulatorsB PUBLIC Samples-Manipulators)
CopyPythonVisualizer(SampleManipulatorsB)
ADD_CONFIG_FOLDER(SampleManipulatorsB)
