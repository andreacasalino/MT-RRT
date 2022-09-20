function(ADD_NAVIGATION_PROBLEM_SAMPLE SCENARIO)
    set(NAME MT-RRT-sample-NavigationProblem-${SCENARIO})

    add_executable(${NAME} Scenario-template.cpp)
    target_link_libraries(${NAME} PUBLIC 
       MT-RRT-sample-NavigationProblem-lib # all the knowledge needed to model and handle trivial problems
    )

    target_compile_definitions(${NAME} PUBLIC
        -D SAMPLE_NAME="navigation-${SCENARIO}"
        -D SAMPLE_JSON="${CMAKE_CURRENT_SOURCE_DIR}/${SCENARIO}.json"
    )
endfunction()

ADD_NAVIGATION_PROBLEM_SAMPLE(Scenario-01)
ADD_NAVIGATION_PROBLEM_SAMPLE(Scenario-02)
ADD_NAVIGATION_PROBLEM_SAMPLE(Scenario-03)
