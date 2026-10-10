function(Terraform SUBDIR)

execute_process(
    COMMAND ${PYTHON_CMD} "${MT_RRT_SCRIPTS}/terraform/Terraform.py"
    "-m" "${CMAKE_CURRENT_SOURCE_DIR}/${SUBDIR}"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_subdirectory(${SUBDIR})

endfunction()
