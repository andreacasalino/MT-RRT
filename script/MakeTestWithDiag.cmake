function(MakeTestWithDiag TARGET_TEST TEST_CASE_NAME DIAG_SCRIPT PLATFORM ARGS_TEST)

set(ARGS_SCRIPT ${ARGN}) 

if(PLATFORM STREQUAL "Linux")
    set(SCRIPT_PATH "${TESTS_BIN_PATH}/${TARGET_TEST}-${TEST_CASE_NAME}-run.sh")
    set(SCRIPT_RUN_CMD "${SCRIPT_PATH}")

    file(WRITE "${SCRIPT_PATH}" 
    "#!/bin/bash
    
export PYTHONPATH='${COMMON_KIT_PATH}'
export MT_RRT_LOG_PATH='${MT_RRT_LOG_PATH}'

${PYTHON_CMD} ${DIAG_SCRIPT} ${ARGS_SCRIPT}
")

    execute_process(
        COMMAND chmod +x ${SCRIPT_PATH}
        RESULT_VARIABLE chmod_result
    )

    if(NOT chmod_result EQUAL 0)
        message(WARNING "Unable to set setup the diag script to be executable")
    endif()

elseif(PLATFORM STREQUAL "Windows")
    # TODO

else()
    message(FATAL "Platform not supported")
endif()

add_custom_target(${TARGET_TEST}-${TEST_CASE_NAME} 
DEPENDS ${TARGET_TEST}
COMMAND ${TESTS_BIN_PATH}/${TARGET_TEST} ${ARGS_TEST}
COMMAND ${SCRIPT_RUN_CMD}
SOURCES ${SCRIPT_PATH} ${DIAG_SCRIPT}
)

endfunction()
