set(LIB_NAME MT-RRT-test-scenarios)

MakeLibrary(${LIB_NAME})
target_link_libraries(${LIB_NAME} PUBLIC 
    MT-RRT-core
    TrivialProblem-lib
)
