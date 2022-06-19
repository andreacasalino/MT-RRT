set(LIB_NAME MT-RRT-core)

MakeTestableLibrary(${LIB_NAME})

target_include_directories(${LIB_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

option(MT_RRT_SHOW_PLANNER_PROGRESS "when enabled show progresses in the console" OFF)
if(MT_RRT_SHOW_PLANNER_PROGRESS)
	target_compile_definitions(${LIB_NAME} PUBLIC SHOW_PLANNER_PROGRESS)
endif()

target_link_libraries(${LIB_NAME} PUBLIC MT-RRT-carpet)

if(BUILD_MT_RRT_TESTS)
    target_link_libraries(${LIB_NAME}-tests PUBLIC MT-RRT-utils)
endif()
