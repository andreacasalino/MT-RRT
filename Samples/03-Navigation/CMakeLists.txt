project(MT-RRT-SampleNavigation)

add_subdirectory(problem)

macro(ADD_CONFIG_FOLDER SAMPLE_NAME)
target_compile_definitions(${SAMPLE_NAME}
PUBLIC
	-D CONFIG_FOLDER="${CMAKE_CURRENT_SOURCE_DIR}"
)
endmacro()

add_executable(SampleNavigation-01-open SampleNavigation-01-open.cpp ${SOURCES})
target_link_libraries(SampleNavigation-01-open PUBLIC Samples-Navigation)
CopyPythonVisualizer(SampleNavigation-01-open)
ADD_CONFIG_FOLDER(SampleNavigation-01-open)

add_executable(SampleNavigation-02-cluttered SampleNavigation-02-cluttered.cpp ${SOURCES})
target_link_libraries(SampleNavigation-02-cluttered PUBLIC Samples-Navigation)
CopyPythonVisualizer(SampleNavigation-02-cluttered)
ADD_CONFIG_FOLDER(SampleNavigation-02-cluttered)
