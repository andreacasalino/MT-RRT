macro(GlobSources DIR)
    file(GLOB_RECURSE SUB_SOURCES 
        ${DIR}/*.h
        ${DIR}/*.hxx
        ${DIR}/*.cpp
    )
    list(APPEND SOURCES ${SUB_SOURCES})
endmacro()

macro(GlobLibSources)
    GlobSources(${CMAKE_CURRENT_SOURCE_DIR}/header)
    GlobSources(${CMAKE_CURRENT_SOURCE_DIR}/src)
endmacro()
