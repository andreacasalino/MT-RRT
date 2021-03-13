macro(MakeLib LIB_NAME)

set(PROJECT_SHORTNAME "${LIB_NAME}")

CollectSourceFiles(${CMAKE_CURRENT_SOURCE_DIR} SOURCES)
GroupSources(${CMAKE_CURRENT_SOURCE_DIR})

if(LIB_OPT)
	add_library(${PROJECT_SHORTNAME} SHARED ${SOURCES})
else()
	add_library(${PROJECT_SHORTNAME} STATIC ${SOURCES})
endif()

target_compile_features(${PROJECT_SHORTNAME}
    PUBLIC cxx_auto_type
    PRIVATE cxx_variadic_templates
)

endmacro()