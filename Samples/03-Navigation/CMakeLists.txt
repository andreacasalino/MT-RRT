project(MT-RRT-SampleNavigation)

add_subdirectory(problem)

macro(ADD_CONFIG_FOLDER SAMPLE_NAME)
target_compile_definitions(${SAMPLE_NAME}
PUBLIC
	-D CONFIG_FOLDER="${CMAKE_CURRENT_SOURCE_DIR}"
)
endmacro()

option(BUILD_DEBUG_NAVIGATION_TRAJ "compile the application to debug the navigation trajectories" OFF)
if(BUILD_DEBUG_NAVIGATION_TRAJ)
add_executable(SampleNavigation-00-traj-gen SampleNavigation-00-traj-gen.cpp)
target_link_libraries(SampleNavigation-00-traj-gen PUBLIC Samples-Navigation)
CopyPythonVisualizer(SampleNavigation-00-traj-gen)
CopyFile(SampleNavigation-00-traj-gen   DebugLog.py   DebugLog.py)
ADD_CONFIG_FOLDER(SampleNavigation-00-traj-gen)
endif()

add_executable(SampleNavigation-01-open SampleNavigation-01-open.cpp)
target_link_libraries(SampleNavigation-01-open PUBLIC Samples-Navigation)
CopyPythonVisualizer(SampleNavigation-01-open)
ADD_CONFIG_FOLDER(SampleNavigation-01-open)

add_executable(SampleNavigation-02-open-compare SampleNavigation-02-open-compare.cpp)
target_link_libraries(SampleNavigation-02-open-compare PUBLIC Samples-Navigation)
CopyPythonVisualizer(SampleNavigation-02-open-compare)
ADD_CONFIG_FOLDER(SampleNavigation-02-open-compare)

add_executable(SampleNavigation-03-cluttered SampleNavigation-03-cluttered.cpp)
target_link_libraries(SampleNavigation-03-cluttered PUBLIC Samples-Navigation)
CopyPythonVisualizer(SampleNavigation-03-cluttered)
ADD_CONFIG_FOLDER(SampleNavigation-03-cluttered)
