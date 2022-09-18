set(LIB_NAME MT-RRT-carpet)

MakeTestableLibrary(${LIB_NAME})

if(BUILD_MT_RRT_TESTS)
    find_package(OpenMP)
    target_link_libraries(${LIB_NAME}-tests PUBLIC OpenMP::OpenMP_CXX)
endif()
