macro(GlobSources SUB_DIRS)
    foreach(ITEM IN LISTS SUB_DIRS)
        file(GLOB_RECURSE SUB_SOURCES 
            ${CMAKE_CURRENT_SOURCE_DIR}/${ITEM}/*.h
            ${CMAKE_CURRENT_SOURCE_DIR}/${ITEM}/*.hxx
            ${CMAKE_CURRENT_SOURCE_DIR}/${ITEM}/*.cpp
        )
        list(APPEND SOURCES ${SUB_SOURCES})
    endforeach()
endmacro()
