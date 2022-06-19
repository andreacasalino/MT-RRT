set(CONFIG_LIB_NAME MT-RRT-configurations)

MakeLibrary(${CONFIG_LIB_NAME})

target_link_libraries(${CONFIG_LIB_NAME} PUBLIC 
    nlohmann_json::nlohmann_json
    MT-RRT-multi-threaded
)
