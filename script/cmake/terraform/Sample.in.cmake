add_executable($$THE_NAME ${CMAKE_CURRENT_SOURCE_DIR}/Main.cpp)

target_link_libraries($$THE_NAME PUBLIC
$$THE_PARENT_NAME
)
