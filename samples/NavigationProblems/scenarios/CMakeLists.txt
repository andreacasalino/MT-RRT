function(ADD_NAVIGATION_PROBLEM_SAMPLE SCENARIO)
    set(NAME NavigationProblem-${SCENARIO})

    add_executable(${NAME} Scenario-template.cpp)
    target_link_libraries(${NAME} PUBLIC 
       MT-RRT-multi-threaded # this already links to MT-RRT-core
       NavigationProblem-lib # all the knowledge needed to model and handle trivial problems
       MT-RRT-framework # utilities to parse a config file (json) in order to build the correct planner
    )

    target_compile_definitions(${NAME} PUBLIC
        -D SAMPLE_NAME="navigation-${SCENARIO}"
        -D SAMPLE_JSON="${CMAKE_CURRENT_SOURCE_DIR}/${SCENARIO}.json"
    )
endfunction()

ADD_NAVIGATION_PROBLEM_SAMPLE(Scenario-01)
