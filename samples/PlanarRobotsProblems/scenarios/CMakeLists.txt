function(ADD_PLANAROBOTS_PROBLEM_SAMPLE SCENARIO)
    set(NAME MT-RRT-sample-PlanarRobots-${SCENARIO})

    add_executable(${NAME} Scenario-template.cpp)
    target_link_libraries(${NAME} PUBLIC 
       MT-RRT-sample-PlanarRobotsProblem-lib # all the knowledge needed to model and handle trivial problems
    )

    target_compile_definitions(${NAME} PUBLIC
        -D SAMPLE_NAME="planar-robots-${SCENARIO}"
        -D SAMPLE_JSON="${CMAKE_CURRENT_SOURCE_DIR}/${SCENARIO}.json"
    )
endfunction()

ADD_PLANAROBOTS_PROBLEM_SAMPLE(Scenario-01)
ADD_PLANAROBOTS_PROBLEM_SAMPLE(Scenario-02)
ADD_PLANAROBOTS_PROBLEM_SAMPLE(Scenario-03)
ADD_PLANAROBOTS_PROBLEM_SAMPLE(Scenario-04)
