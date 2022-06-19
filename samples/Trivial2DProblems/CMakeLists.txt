function(ADD_TRIVIALPROBLEM_SAMPLE SCENARIO)
    set(NAME Trival2DProblem-${SCENARIO})

    add_executable(${NAME} 
       ${SCENARIO}.cpp
       TrivialProblem.h
       TrivialProblem.cpp
    )
    target_link_libraries(${NAME} PUBLIC 
       MT-RRT-multi-threaded # this already links to MT-RRT-core
       MT-RRT-utils # definition of point problem that the trivial 2D problem extends
       MT-RRT-configurations # utilities to parse a config file (json) in order to build the correct planner
    )
endfunction()

ADD_TRIVIALPROBLEM_SAMPLE(Scenario-01)
ADD_TRIVIALPROBLEM_SAMPLE(Scenario-02)
