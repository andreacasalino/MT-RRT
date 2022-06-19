set(LIB_NAME MT-RRT-utils)

MakeTestableLibrary(${LIB_NAME})
if(ENABLE_MT_RRT_TESTS_LOGGING)
    target_compile_definitions(${LIB_NAME} PUBLIC TEST_LOGGING)
endif()

target_link_libraries(${LIB_NAME} PUBLIC 
    MT-RRT-core
    nlohmann_json::nlohmann_json
)

target_compile_definitions(${LIB_NAME} PUBLIC
    -D POINT_PROBLEM_SHOW_PYTHON_SCRIPT="${CMAKE_CURRENT_SOURCE_DIR}/src/PointProblemShow.py"
)
