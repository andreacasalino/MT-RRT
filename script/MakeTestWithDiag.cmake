function(MakeTestWithDiag TARGET_TEST TEST_CASE_NAME DIAG_SCRIPT ARGS_TEST)

set(ARGS_SCRIPT ${ARGN}) 
set(SCRIPT_PATH "${TESTS_BIN_PATH}/${TARGET_TEST}-${TEST_CASE_NAME}-run.sh")

# create the command
# TODO adapt to the host system (Linux, Windows)
file(WRITE "${SCRIPT_PATH}" 
"
export PYTHONPATH='${COMMON_KIT_PATH}'
export MT_RRT_LOG_PATH='${MT_RRT_LOG_PATH}'

${PYTHON_CMD} ${DIAG_SCRIPT} ${CMAKE_BINARY_DIR}/CMakeCache.txt ${ARGS_SCRIPT}
")

add_custom_target(${TARGET_TEST}-${TEST_CASE_NAME} 
DEPENDS ${TARGET_TEST}
COMMAND ${TESTS_BIN_PATH}/${TARGET_TEST} --gtest_filter=${ARGS_TEST}
COMMAND sh ${SCRIPT_PATH}
SOURCES ${SCRIPT_PATH} ${DIAG_SCRIPT}
)

endfunction()
