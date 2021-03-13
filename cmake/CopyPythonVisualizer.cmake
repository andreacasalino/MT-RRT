macro(CopyPythonVisualizer SAMPLE_NAME)
	add_custom_command(TARGET ${SAMPLE_NAME} PRE_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/VisualizeResult.py ${CMAKE_CURRENT_BINARY_DIR}/VisualizeResult.py)
	add_custom_command(TARGET ${SAMPLE_NAME} PRE_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/../Utils/Visualizer.py ${CMAKE_CURRENT_BINARY_DIR}/Visualizer.py)
endmacro()
