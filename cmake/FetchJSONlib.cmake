function(FECTH_JSON_LIB)
    if (NOT TARGET nlohmann_json::nlohmann_json)
        message("Fetching nlohmann")
        include(FetchContent)
        FetchContent_Declare(
        json_lib
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG        v3.10.5
        )
        FetchContent_MakeAvailable(json_lib)
    endif()    
endfunction()
