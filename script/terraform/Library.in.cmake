
GlobSources("header;src")

if(LIB_OPT)
    if (WIN32)
        set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
    endif ()	
    
    add_library($$THE_NAME SHARED ${SOURCES})
else()
    add_library($$THE_NAME STATIC ${SOURCES})
endif()

target_include_directories($$THE_NAME PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/header/
)

install(TARGETS $$THE_NAME)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/header/ DESTINATION include/$$THE_NAME FILES_MATCHING PATTERN "*.h*")
