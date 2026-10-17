option($$THE_OPTION_NAME "$$THE_DESCRIPTION" $$THE_DEFAULT_VALUE)
if($$THE_OPTION_NAME)
    target_compile_definitions($$THE_NAME $$THE_SCOPE $$THE_OPTION_NAME)
endif()
