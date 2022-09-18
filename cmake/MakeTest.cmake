function(MAKE_TEST LIB_NAME)
    file(GLOB SOURCES 
        ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp 
        ${CMAKE_CURRENT_SOURCE_DIR}/test/*.h 
    )

    set(TEST_NAME "${LIB_NAME}-tests")

    add_executable(${TEST_NAME} ${SOURCES})

    target_link_libraries(${TEST_NAME} PUBLIC
        Catch2::Catch2
        Catch2::Catch2WithMain
        ${LIB_NAME}
    )

    target_compile_definitions(${TEST_NAME} PUBLIC
        -D TEST_TAG="[${LIB_NAME}]"
        -D TEST_FOLDER="${CMAKE_CURRENT_SOURCE_DIR}/test/"
    )

    install(TARGETS ${TEST_NAME})

endfunction()
