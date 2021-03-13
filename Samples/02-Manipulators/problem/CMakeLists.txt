MakeLib("Samples-Manipulators")

target_include_directories(${PROJECT_SHORTNAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/header>
)

target_link_libraries(${PROJECT_SHORTNAME}
	PUBLIC
		Samples-utils
)
