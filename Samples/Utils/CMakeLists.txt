MakeLib("Samples-utils")

target_include_directories(${PROJECT_SHORTNAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/header>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/header/geometry>
)

target_link_libraries(${PROJECT_SHORTNAME}
	PUBLIC
		MT-RRT
)
