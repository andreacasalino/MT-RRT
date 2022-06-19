function(ADD_PLANARROBOT_SAMPLE SCENARIO)
    set(NAME PlanarRobotsProblem-${SCENARIO})

    add_executable(${NAME} 
       ${SCENARIO}.cpp
       PlanarRobotsProblem.h
       PlanarRobotsProblem.cpp
    )
    target_link_libraries(${NAME} PUBLIC 
       MT-RRT-multi-threaded # this already links to MT-RRT-core
       MT-RRT-utils # definition of point problem that the trivial 2D problem extends
       MT-RRT-configurations # utilities to parse a config file (json) in order to build the correct planner
    )
endfunction()

ADD_PLANARROBOT_SAMPLE(Scenario-01)
