if(BUILD_MT_RRT_SAMPLES)
    set(CONFIG_LIB_NAME MT-RRT-framework)

    MakeLibrary(${CONFIG_LIB_NAME})

    target_link_libraries(${CONFIG_LIB_NAME} PUBLIC 
        MT-RRT-utils
        MT-RRT-multi-threaded
    )
endif()
