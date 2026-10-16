macro(GlobSources DIR)
    file(GLOB_RECURSE SUB_SOURCES 
        ${DIR}/*.h
        ${DIR}/*.hxx
        ${DIR}/*.cpp
    )
    list(APPEND SOURCES ${SUB_SOURCES})
endmacro()

macro(GlobSourcesSubdirs SUB_DIRS)
    foreach(ITEM IN LISTS SUB_DIRS)
        GlobSources(${CMAKE_CURRENT_SOURCE_DIR}/${ITEM})
    endforeach()
endmacro()
