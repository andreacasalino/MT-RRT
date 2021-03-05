add_subdirectory(problem)

add_executable(Sample01_Points Sample01.cpp ${SOURCES})
target_link_libraries(Sample01_Points PUBLIC MT-RRT-Samples-Points)
add_custom_command(TARGET Sample01_Points PRE_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Visualizer.py ${CMAKE_CURRENT_BINARY_DIR}/Visualizer.py)
add_custom_command(TARGET Sample01_Points PRE_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/../Utils/Parser.py ${CMAKE_CURRENT_BINARY_DIR}/Parser.py)

add_executable(Sample02_Points Sample02.cpp ${SOURCES})
target_link_libraries(Sample02_Points PUBLIC MT-RRT-Samples-Points)
add_custom_command(TARGET Sample02_Points PRE_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Visualizer.py ${CMAKE_CURRENT_BINARY_DIR}/Visualizer.py)
add_custom_command(TARGET Sample02_Points PRE_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/../Utils/Parser.py ${CMAKE_CURRENT_BINARY_DIR}/Parser.py)
