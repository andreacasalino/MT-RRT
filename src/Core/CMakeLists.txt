set(PROJECT_SHORTNAME "MT-RRT-Core")
project(${PROJECT_SHORTNAME} VERSION ${VERSION} LANGUAGES CXX)

CollectSourceFiles(${CMAKE_CURRENT_SOURCE_DIR} SOURCES)
GroupSources(${CMAKE_CURRENT_SOURCE_DIR})

if(LIB_OPT)
	add_library(${PROJECT_NAME} SHARED ${SOURCES})
else()
	add_library(${PROJECT_NAME} STATIC ${SOURCES})
endif()
add_library(MT::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_compile_features(${PROJECT_NAME}
    PUBLIC cxx_auto_type
    PRIVATE cxx_variadic_templates
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/header>
        $<INSTALL_INTERFACE:header>
	PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/extn/header>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/tree/header>
)

find_package(Threads)
find_package(OpenMP)
target_link_libraries(${PROJECT_NAME} 
	PUBLIC
		${CMAKE_THREAD_LIBS_INIT} 
	PRIVATE 
		OpenMP::OpenMP_CXX
)
