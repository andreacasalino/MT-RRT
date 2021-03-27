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
add_executable(SampleNavigation-debug SampleNavigation-debug.cpp)
target_link_libraries(SampleNavigation-debug PUBLIC Samples-Navigation)
CopyPythonVisualizer(SampleNavigation-debug)
CopyFile(SampleNavigation-debug   DebugLog.py   DebugLog.py)
ADD_CONFIG_FOLDER(SampleNavigation-debug)
endif()

add_executable(SampleNavigationA SampleNavigationA.cpp)
target_link_libraries(SampleNavigationA PUBLIC Samples-Navigation)
CopyPythonVisualizer(SampleNavigationA)
ADD_CONFIG_FOLDER(SampleNavigationA)

add_executable(SampleNavigationA-profile SampleNavigationA-profile.cpp)
target_link_libraries(SampleNavigationA-profile PUBLIC Samples-Navigation)
CopyPythonVisualizer(SampleNavigationA-profile)
ADD_CONFIG_FOLDER(SampleNavigationA-profile)

add_executable(SampleNavigationB SampleNavigationB.cpp)
target_link_libraries(SampleNavigationB PUBLIC Samples-Navigation)
CopyPythonVisualizer(SampleNavigationB)
ADD_CONFIG_FOLDER(SampleNavigationB)
