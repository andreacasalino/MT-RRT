function(ADD_TRIVIALPROBLEM_SAMPLE SCENARIO)
    set(NAME MT-RRT-sample-TrivalProblem-${SCENARIO})

    add_executable(${NAME} Scenario-template.cpp)
    target_link_libraries(${NAME} PUBLIC 
       MT-RRT-multi-threaded # this already links to MT-RRT-core
       TrivialProblem-lib # all the knowledge needed to model and handle trivial problems
       MT-RRT-framework # utilities to parse a config file (json) in order to build the correct planner
    )

    target_compile_definitions(${NAME} PUBLIC
        -D SAMPLE_NAME="planar-robots-${SCENARIO}"
        -D SAMPLE_JSON="${CMAKE_CURRENT_SOURCE_DIR}/${SCENARIO}.json"
    )
endfunction()

ADD_TRIVIALPROBLEM_SAMPLE(Scenario-01)
ADD_TRIVIALPROBLEM_SAMPLE(Scenario-02)
