set(LIB_NAME MT-RRT-sample-utils)

MakeTestableLibrary(${LIB_NAME})
target_link_libraries(${LIB_NAME} PUBLIC 
    MT-RRT-multi-threaded # this already links to MT-RRT-core
    nlohmann_json::nlohmann_json
)

target_compile_definitions(${LIB_NAME} PUBLIC
    -D PYTHON_SHOW_PRE="${CMAKE_CURRENT_SOURCE_DIR}/src/ShowPre.py"
    -D PYTHON_SHOW_POST="${CMAKE_CURRENT_SOURCE_DIR}/src/ShowPost.py"
)
