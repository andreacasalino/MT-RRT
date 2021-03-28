MakeLib("MT-RRT")

option(SHOW_PROGRESS_OPT "show extension progresses enabled (ON) or disabled (OFF)" OFF)
if(SHOW_PROGRESS_OPT)
	target_compile_definitions(${PROJECT_SHORTNAME}
	PRIVATE
		SHOW_PROGRESS
	)
endif()

target_include_directories(${PROJECT_SHORTNAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/header>
	PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/extn/header>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/tree/header>
)

find_package(Threads)
find_package(OpenMP)
target_link_libraries(${PROJECT_SHORTNAME} 
	PUBLIC
		${CMAKE_THREAD_LIBS_INIT} 
	PRIVATE 
		OpenMP::OpenMP_CXX
)
