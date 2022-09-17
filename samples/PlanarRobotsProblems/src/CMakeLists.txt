set(LIB_NAME MT-RRT-sample-PlanarRobotsProblem-lib)

MakeTestableLibrary(${LIB_NAME})
if(ENABLE_MT_RRT_TESTS_LOGGING)
    target_compile_definitions(${LIB_NAME} PUBLIC TEST_LOGGING)
endif()

target_link_libraries(${LIB_NAME} PUBLIC 
    MT-RRT-sample-utils
)

target_compile_definitions(${LIB_NAME} PUBLIC
    -D PLANAR_ROBOTS_PROBLEM_PYTHON_SCRIPT="${CMAKE_CURRENT_SOURCE_DIR}/../PlanarRobotsShow.py"
)
