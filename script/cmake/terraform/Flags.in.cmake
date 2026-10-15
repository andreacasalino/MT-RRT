option($$THE_NAME $$THE_DESCRIPTION $$THE_VALUE_DEFAULT)
if($$THE_NAME)
    target_compile_definitions($$THE_TARGET $$THE_SCOPE $$THE_NAME)
endif()
