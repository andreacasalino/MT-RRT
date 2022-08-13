set(LIB_NAME MT-RRT-multi-threaded)

MakeTestableLibrary(${LIB_NAME})

target_include_directories(${LIB_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)
target_link_libraries(${LIB_NAME} 
    PUBLIC 
        MT-RRT-core
    PRIVATE
        OpenMP::OpenMP_CXX
        ${CMAKE_THREAD_LIBS_INIT}
)

if(BUILD_MT_RRT_TESTS)
    target_link_libraries(${LIB_NAME}-tests PUBLIC 
    MT-RRT-utils
    MT-RRT-test-scenarios
)
endif()
