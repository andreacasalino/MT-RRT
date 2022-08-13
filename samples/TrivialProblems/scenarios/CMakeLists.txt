function(ADD_TRIVIALPROBLEM_SAMPLE SCENARIO)
    set(NAME TrivalProblem-${SCENARIO})

    add_executable(${NAME} ${SCENARIO}.cpp)
    target_link_libraries(${NAME} PUBLIC 
       MT-RRT-multi-threaded # this already links to MT-RRT-core
       TrivialProblem-lib # all the knowledge needed to model and handle trivial problems
       MT-RRT-configurations # utilities to parse a config file (json) in order to build the correct planner
    )
endfunction()

ADD_TRIVIALPROBLEM_SAMPLE(Scenario-01)
ADD_TRIVIALPROBLEM_SAMPLE(Scenario-02)
