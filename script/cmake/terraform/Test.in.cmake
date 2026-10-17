set(SOURCES "")
GlobSources(${CMAKE_CURRENT_SOURCE_DIR})
add_executable($$THE_NAME ${SOURCES})
install(TARGETS $$THE_NAME)

target_link_libraries($$THE_NAME PUBLIC
$$THE_PARENT_NAME
GTest::gtest
GTest::gtest_main
)

target_compile_definitions($$THE_NAME PUBLIC
-D TEST_FOLDER="${CMAKE_CURRENT_SOURCE_DIR}"
)
