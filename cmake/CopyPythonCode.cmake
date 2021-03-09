macro(CopyPythonCode SAMPLE_NAME)
	add_custom_command(TARGET ${SAMPLE_NAME} PRE_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/ResultMaker.py ${CMAKE_CURRENT_BINARY_DIR}/ResultMaker.py)
	add_custom_command(TARGET ${SAMPLE_NAME} PRE_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/../Utils/Visualizer.py ${CMAKE_CURRENT_BINARY_DIR}/Visualizer.py)
endmacro()
